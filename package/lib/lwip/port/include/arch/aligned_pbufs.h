#pragma once
#include <stddef.h>
#include <stdint.h>
#include "lwip/pbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DMA_ALIGNED_PBUF_POOL_SIZE
# define DMA_ALIGNED_PBUF_POOL_SIZE 30
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

struct pbuf *pbuf_from_payload(void *payload, uint32_t len);

int dma_aligned_pbuf_pool_init(void);
void dma_aligned_pbuf_pool_deinit(void);
	
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
            struct pbuf *__p = pbuf_from_payload((ptr), 0);                     \
            if (__p) {                                                       \
                /* zero-copy path: will unref and free in pool */            \
                dma_aligned_pbuf_free(__p);                               \
            } else {                                                         \
                /* raw-buffer path */                                        \
                aligned_free((ptr));              	                       \
            }                                                                \
        }                                                                    \
    } while (0)
#else
#define MEM_MALLOC(len) malloc(len)
#define MEM_FREE(ptr) free(ptr)
#endif

#ifdef __cplusplus
}
#endif

