/*
 * aligned_pbufs.c
 *
 * NOTE: There are two distinct pools:
 *   pool 0 (dma_aligned_pbuf_pool) holds active custom pbuf entries,
 *     each with p->ref == 1 when handed to LwIP,
 *   pool 1 (dma_aligned_buf_ring) holds raw DMA buffers,
 *     which have ref == 0 until wrapped via dma_aligned_pbuf_alloc().
 *
 * When dumping or tracing, only inspect dma_aligned_pbuf_pool[]
 * to avoid confusing raw buffers with active pbufs.
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "nrc_sdk.h"
#include "aligned_pbufs.h"

#ifdef NRC7394_DMA_MEMPOOL

// #define DMA_ALIGNED_DEBUG 1
#ifdef DMA_ALIGNED_DEBUG
    static int total_buffers = 0;
    static int total_in_use = 0;
    #define DBG_PRINT(fmt, ...) nrc_usr_print("[DMA_DBG] %s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__)
#else
    #define DBG_PRINT(fmt, ...)
#endif

#ifndef delay_us
    #define delay_us(x) vTaskDelay(pdMS_TO_TICKS(x) / 1000)
#endif

typedef struct {
    struct pbuf_custom pbuf_cust;
    uint8_t *payload;  // external DMA buffer
    uint8_t in_use;
} dma_aligned_pbuf_entry_t;

static dma_aligned_pbuf_entry_t *dma_aligned_pbuf_pool = NULL;
static dma_aligned_pbuf_entry_t *dma_aligned_entry_ring[DMA_ALIGNED_PBUF_POOL_SIZE];
static size_t dma_aligned_head = 0, dma_aligned_tail = 0, dma_aligned_count = 0;

// === payload-> entry lookup table ===
static dma_aligned_pbuf_entry_t *payload_lookup_table[DMA_ALIGNED_PBUF_POOL_SIZE];

ATTR_NC __attribute__((optimize("O3"))) void *aligned_malloc(size_t size, size_t alignment)
{
    int retries = 15;
    uintptr_t raw;
	
    DBG_PRINT("allocating %d bytes\n", size);

    if (alignment < sizeof(void*)) alignment = sizeof(void*);
    do {
	raw = (uintptr_t)malloc(size + alignment - 1 + sizeof(void*));
	if (!raw){
		DBG_PRINT("malloc failure for %d bytes, retries %d\n", size, retries);
	}
	delay_us(10);
    } while(!raw && --retries > 0);

    if(!raw) {
	return NULL;
    }

    uintptr_t aligned = (raw + sizeof(void*) + alignment - 1) & ~(alignment - 1);
    ((void**)aligned)[-1] = (void*)raw;
    return (void*)aligned;
}

ATTR_NC __attribute__((optimize("O3"))) void aligned_free(void *ptr)
{
    DBG_PRINT("[%s]\n", __func__);
    if (!ptr) return;
    uintptr_t aligned = (uintptr_t)ptr;
    uintptr_t raw = ((uintptr_t *)aligned)[-1];
    free((void *)raw);
}

ATTR_NC __attribute__((optimize("O3"))) inline void* unalign_aligned_buffer(uint8_t *aligned_buffer)
{
    DBG_PRINT("[%s]\n", __func__);
    return ((void**)aligned_buffer)[-1];
}

ATTR_NC __attribute__((optimize("O3"))) inline struct pbuf *pbuf_from_payload(void *payload, uint32_t len)
{
    DBG_PRINT("[%s]\n", __func__);

    if (!payload) return NULL;

    for (int i = 0; i < DMA_ALIGNED_PBUF_POOL_SIZE; ++i) 
    {
        if (payload_lookup_table[i] && payload_lookup_table[i]->payload == payload) {
	    struct pbuf *p = (struct pbuf *)&payload_lookup_table[i]->pbuf_cust;
	    if(len > 0)
		p->len = p->tot_len = len;
	    return p;
	}
    }

    return NULL;
}

ATTR_NC __attribute__((optimize("O3"))) void dma_aligned_payload_free(void *payload) 
{
    DBG_PRINT("[%s]\n", __func__);
    struct pbuf *p = pbuf_from_payload(payload, 0);
    if (p) {
        dma_aligned_pbuf_free(p);
    }
}

int dma_aligned_pbuf_pool_init(void)
{
    DBG_PRINT("[%s]\n", __func__);

    // Allocate array of entries
    dma_aligned_pbuf_pool = calloc(DMA_ALIGNED_PBUF_POOL_SIZE, sizeof(dma_aligned_pbuf_entry_t));
    if (!dma_aligned_pbuf_pool) return -1;

    // Build ring: all entries start free, payloads NULL
    for (size_t i = 0; i < DMA_ALIGNED_PBUF_POOL_SIZE; ++i) 
    {
        dma_aligned_pbuf_pool[i].payload = NULL;
        dma_aligned_pbuf_pool[i].in_use = 0;
        dma_aligned_entry_ring[i] = &dma_aligned_pbuf_pool[i];
#ifdef DMA_ALIGNED_DEBUG
	total_buffers++;
#endif
    }

    dma_aligned_head  = 0;
    dma_aligned_tail  = 0;
    dma_aligned_count = DMA_ALIGNED_PBUF_POOL_SIZE;

    return 0;
}

void dma_aligned_pbuf_pool_deinit(void)
{
    DBG_PRINT("[%s]\n", __func__);

    if (!dma_aligned_pbuf_pool) return;

    // Free any allocated payloads
    for (size_t i = 0; i < DMA_ALIGNED_PBUF_POOL_SIZE; ++i) {
        if (dma_aligned_pbuf_pool[i].payload) {
            aligned_free(dma_aligned_pbuf_pool[i].payload);
	    dma_aligned_pbuf_pool[i].payload = NULL;
        }
    }
    free(dma_aligned_pbuf_pool);
    dma_aligned_pbuf_pool = NULL;
    dma_aligned_count = dma_aligned_head = dma_aligned_tail = 0;
}

ATTR_NC __attribute__((optimize("O3"))) struct pbuf *dma_aligned_pbuf_alloc(u16_t len)
{
    DBG_PRINT("[%s]\n", __func__);

    // Save index before popping
    size_t index = dma_aligned_head;

    // No entries free
    if (dma_aligned_count == 0) {
        E(TT_NET, "%s: no free pbuf entries\n", __func__);
        return NULL;
    }

    // Pop entry
    dma_aligned_pbuf_entry_t *entry = dma_aligned_entry_ring[index];
    dma_aligned_head = (dma_aligned_head + 1) % DMA_ALIGNED_PBUF_POOL_SIZE;
    dma_aligned_count--;

    // Lazy-allocate payload buffer
    if (!entry->payload) {
        entry->payload = aligned_malloc(len, DMA_ALIGNMENT);
	DBG_PRINT("allocated NEW payload buffer at index=%zu\n", index);
        if (!entry->payload) {
            E(TT_NET, "%s: payload alloc failed\n", __func__);
            // push back and return NULL
            dma_aligned_entry_ring[index] = entry;
            dma_aligned_count++;
            return NULL;
        }
    }
    else {
        DBG_PRINT("REUSING payload buffer at index=%zu, payload=%p\n", index, entry->payload);
    }
    
#ifdef DMA_ALIGNED_DEBUG
    if (entry->in_use) {
        DBG_PRINT("warning — reusing entry already marked in_use!\n");
    }
#endif

    entry->in_use = 1;
    struct pbuf *p = (struct pbuf *)&entry->pbuf_cust;
    // memset(&entry->pbuf_cust, 0, sizeof(entry->pbuf_cust));

#ifdef DMA_ALIGNED_DEBUG
    if (((uintptr_t)entry->payload & (DMA_ALIGNMENT - 1)) != 0) {
        DBG_PRINT("payload buffer not aligned (ptr=%p)\n", entry->payload);
    }
#endif

    p->payload            = entry->payload;
    p->len = p->tot_len   = len;
    p->type_internal      = PBUF_REF;
    p->flags 		 |= PBUF_FLAG_IS_CUSTOM;
    p->ref                = 1;
    p->next               = NULL;
    entry->pbuf_cust.custom_free_function = dma_aligned_pbuf_free;

    // for O(1) payload->entry lookup
    payload_lookup_table[index] = entry;

#ifdef DMA_ALIGNED_DEBUG
	DBG_PRINT("total buffers %d total in use %d\n", total_buffers, ++total_in_use);
#endif

    return p;
}

ATTR_NC __attribute__((optimize("O3"))) void dma_aligned_pbuf_free(struct pbuf *p)
{
    DBG_PRINT("[%s]\n", __func__);

    // Retrieve entry via container_of
    dma_aligned_pbuf_entry_t *entry = (dma_aligned_pbuf_entry_t *)((char *)p - offsetof(dma_aligned_pbuf_entry_t, pbuf_cust));

    if (p->ref != 0) {
        // CRITICAL CHECK - LOG if this happens
        E(TT_NET, "Attempt to free pbuf (payload=%p) with ref=%d\n", p->payload, p->ref);
        return;
    }

    entry->in_use = 0;
#ifdef DMA_ALIGNED_DEBUG
    bool found = false;
#endif
    // remove hash from our O(1) payload->entry lookup
    for (int i = 0; i < DMA_ALIGNED_PBUF_POOL_SIZE; ++i) 
    {
        if (payload_lookup_table[i] == entry) {
            payload_lookup_table[i] = NULL;
#ifdef DMA_ALIGNED_DEBUG	    
	    found = true;
	    DBG_PRINT("cleared payload_lookup_table at index=%d\n", i);
#endif
	    break;
	}
    }
#ifdef DMA_ALIGNED_DEBUG
    if (!found) {
        DBG_PRINT("WARNING: entry not found in payload_lookup_table\n");
    }
#endif
    // Reset fields (optional)
    p->ref = p->len = p->tot_len = 0;
    p->next = NULL;

    // Recycle entry
#ifdef DMA_ALIGNED_DEBUG
    if (dma_aligned_count >= DMA_ALIGNED_PBUF_POOL_SIZE) {
        E(TT_NET, "error — ring buffer overflow\n");
    }
#endif

    // Recycle entry
    dma_aligned_entry_ring[dma_aligned_tail] = entry;
    dma_aligned_tail = (dma_aligned_tail + 1) % DMA_ALIGNED_PBUF_POOL_SIZE;
    dma_aligned_count++;
    
    DBG_PRINT("entry returned to ring at tail=%zu\n", dma_aligned_tail);
    DBG_PRINT("total buffers %d total in use %d\n", total_buffers, --total_in_use);
}

#endif /* NRC7394_DMA_MEMPOOL */
