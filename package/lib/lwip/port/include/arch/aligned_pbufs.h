#pragma once
#include <stddef.h>
#include <stdint.h>
#include "lwip/pbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DMA_ALIGNED_PBUF_POOL_SIZE
# define DMA_ALIGNED_PBUF_POOL_SIZE 25
#endif

#ifdef MEM_ALIGNMENT
# define DMA_ALIGNMENT MEM_ALIGNMENT
#elif !defined(DMA_ALIGNMENT)
# define DMA_ALIGNMENT 4
#endif

#ifndef PBUF_POOL_BUFSIZE
# define PBUF_POOL_BUFSIZE 1600
#endif

void *aligned_malloc(size_t size, size_t alignment);
void aligned_free(void *ptr);
void* unalign_aligned_buffer(uint8_t *aligned_buffer);

void *dma_aligned_malloc(size_t size);
void dma_aligned_free(void *ptr);

struct pbuf *dma_aligned_pbuf_alloc(u16_t len);
void dma_aligned_pbuf_free(struct pbuf *p);

struct pbuf *pbuf_from_payload(void *payload);

int dma_aligned_pbuf_pool_init(void);
void dma_aligned_pbuf_pool_deinit(void);

#ifdef DMA_ALIGNED_DEBUG
void dma_aligned_pbuf_print_stats(void);
#endif
	
#ifdef NRC7394_DMA_MEMPOOL
#define MEM_MALLOC(len)                                                      \
    ({ struct pbuf *__p = dma_aligned_pbuf_alloc((len));                    \
       void *__buf = NULL;                                                  \
       if (__p) {                                                           \
           __buf = (void *)__p->payload;                                    \
       }                                                                    \
       __buf; })

#define MEM_FREE(ptr)                                                       \
    do {                                                                     \
        if (ptr) {                                                           \
            struct pbuf *__p = pbuf_from_payload((ptr));                     \
            if (__p) {                                                       \
                /* zero-copy path: will unref and free in pool */            \
                dma_aligned_pbuf_free(__p);                               \
            } else {                                                         \
                /* raw-buffer path */                                        \
                aligned_free((ptr));              	                       \
            }                                                                \
        }                                                                    \
    } while (0)
// void mem_free(void *m) { MEM_FREE(m); }
#else
#define MEM_MALLOC(len) malloc(len)
#define MEM_FREE(ptr) free(ptr)
#endif

#ifdef __cplusplus
}
#endif

