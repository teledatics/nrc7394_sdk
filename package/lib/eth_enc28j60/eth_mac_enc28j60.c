/*
 * SPDX-FileCopyrightText: 2019-2021 Espressif Systems (Shanghai) CO LTD
 *                         2022 Teledatics Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "eth_mac.h"
#include "enc28j60.h"
#include "nrc_sdk.h"
#include "util_trace.h"

#include "api_dma.h"
#include "api_spi_dma.h"

#include <cmsis_gcc.h>

#define DUMP_HEX(title, buf, len)                                \
  do {                                                           \
    printf("[DUMP] %s (%d bytes):\n", title, (int)(len));        \
    for (int i = 0; i < (int)(len); i++) {                       \
      printf("%02X ", ((uint8_t *)(buf))[i]);                    \
      if ((i + 1) % 16 == 0) printf("\n");                       \
    }                                                            \
    if ((len) % 16 != 0) printf("\n");                           \
  } while (0)

#ifdef NRC7394_DMA_MEMPOOL
#define DMA_ALIGNED_PBUF_POOL_SIZE 30
#include "arch/aligned_pbufs.h"
// for custom pbufs code, see lwip/port/arch/aligned_pufs.c
#endif

#ifdef ETHERNET_SPI_DMA
#undef ETHERNET_DYNAMIC_BUFFERS

// NOTE: do not change these values
#define DMA_TX_CHANNEL 0
#define DMA_RX_CHANNEL 1

extern dma_peri_t spi_tx_peri;
extern dma_peri_t spi_rx_peri;
static dma_desc_t tx_desc;
static dma_desc_t rx_desc;

static SemaphoreHandle_t tx_sem;
static SemaphoreHandle_t rx_sem;

typedef enum {
    DMA_DIR_NONE = 0,
    DMA_DIR_RX,
    DMA_DIR_TX
} dma_dir_t;

static volatile bool spi_dma_busy = false;

#ifdef SPI_DMA_CHAINED

#define DMA_CHAIN_LIMIT 3

volatile uint8_t current_pkt_count = 0;

// packet processing events
#define EVT_IDLE                0x00
#define EVT_RX_PAYLOAD_DONE     0x04
#define EVT_TX_START            0x08
#define EVT_TX_DONE             0x10
#define EVT_TX_ERR              0x20
#define EVT_RX_ERR              0x40
#define EVT_RX_DISCARD_PKT      0x80

volatile uint8_t event_flags = EVT_IDLE;

static volatile int dma_chain_depth = 0;

#define SPI_IDLE_POLL_CYCLES 100

typedef enum {
    SPI_OWNER_NONE = 0,
    SPI_OWNER_DMA_ISR,
    SPI_OWNER_TASK,
} spi_owner_t;

static volatile spi_owner_t spi_owner = SPI_OWNER_NONE;

typedef enum {
    RX_NODE_IDLE = 0,
    RX_NODE_DMA_IN_PROGRESS,
    RX_NODE_READY_FOR_LWIP
} rx_node_state_t;

#define HEADER_SIZE    6

typedef struct {
    rx_node_state_t state;
    uint8_t __attribute__((aligned(4))) header[HEADER_SIZE];
    uint8_t *aligned_packet_buffer;
    uint16_t packet_length;
} rx_node_t;

#define RX_NODE_COUNT 8

static rx_node_t rx_nodes[RX_NODE_COUNT];
static rx_node_t *current_dma_node = NULL;

/* [Modified] TX frame tracking for DMA-based transmit */
static volatile uint16_t tx_frame_len = 0;    // current TX frame length (no CRC)
static volatile uint8_t *tx_frame_data = NULL; // pointer to TX frame buffer (with control for small frames)

// task read pkt queue
static QueueHandle_t rx_ready_queue;
#endif

#endif /* #ifdef ETHERNET_SPI_DMA */

#ifndef ETHERNET_DYNAMIC_BUFFERS
static __attribute__((aligned(4))) uint8_t spi_transfer_buf[SPI_TRANSFER_BUF_LEN];
#else
uint8_t *spi_transfer_buf;
#endif

#define MIN_ETH_FRAME_SIZE 64
#define MAX_ETH_FRAME_SIZE 1518

#define TX_WAIT_TIMEOUT_MS  25
#define MAX_RETRY_COUNT     10
#define MAX_REG_RETRY_COUNT 10

#define delay_us(x) vTaskDelay(pdMS_TO_TICKS(x) / 1000)

static const char* TAG = "enc28j60";

// #define ENC28J60_DEBUG 1

#ifdef ENC28J60_DEBUG
#define MAC_CHECK(a, str, goto_tag, ret_value, ...)                            \
  do {                                                                         \
    int retry = MAX_REG_RETRY_COUNT;                                               \
    if (!(a) || !(--retry)) {                                                  \
      E(TT_NET, "%s (%d): " str "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);  \
      ret = ret_value;                                                         \
      goto goto_tag;                                                           \
    }                                                                          \
    delay_us(1);                                                              \
  } while (0)

#define MAC_CHECK_NO_RET(a, str, goto_tag, ...)                                \
  do {                                                                         \
    int retry = MAX_REG_RETRY_COUNT;                                               \
    if (!(a) || !(--retry)) {                                                  \
      E(TT_NET, "%s (%d):" str  "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);  \
      goto goto_tag;                                                           \
    }                                                                          \
    delay_us(1);                                                              \
  } while (0)

#else
#define MAC_CHECK(a, str, goto_tag, ret_value, ...)                            \
  do {                                                                         \
    int retry = MAX_REG_RETRY_COUNT;                                               \
    if (!(a) || !(--retry)) {                                                  \
      ret = ret_value;                                                         \
      goto goto_tag;                                                           \
    }                                                                          \
    delay_us(1);                                                              \
  } while (0)

#define MAC_CHECK_NO_RET(a, str, goto_tag, ...)                                \
  do {                                                                         \
    int retry = MAX_REG_RETRY_COUNT;                                               \
    if (!(a) || !(--retry)) {                                                  \
      goto goto_tag;                                                           \
    }                                                                          \
    delay_us(1);                                                              \
  } while (0)
#endif

#define ENC28J60_SPI_LOCK_TIMEOUT_MS (50)
#define ENC28J60_REG_TRANS_LOCK_TIMEOUT_MS (150)
#define ENC28J60_PHY_OPERATION_TIMEOUT_US (150)
#define ENC28J60_SYSTEM_RESET_ADDITION_TIME_US (1000)
#define ENC28J60_TX_READY_TIMEOUT_MS (2000)

#define ENC28J60_BUFFER_SIZE (0x2000) // 8KB built-in buffer
/**
 *  ______
 * |__TX__| TX: 2 KB : [0x1800, 0x2000)
 * |      |
 * |  RX  | RX: 6 KB : [0x0000, 0x1800)
 * |______|
 *
 */
#define ENC28J60_BUF_RX_START (0)
#define ENC28J60_BUF_RX_END (ENC28J60_BUF_TX_START - 1)
#define ENC28J60_BUF_TX_START ((ENC28J60_BUFFER_SIZE / 4) * 3)
#define ENC28J60_BUF_TX_END (ENC28J60_BUFFER_SIZE - 1)

#define ENC28J60_RSV_SIZE (6) // Receive Status Vector Size
#define ENC28J60_TSV_SIZE (6) // Transmit Status Vector Size

typedef struct
{
  uint8_t next_packet_low;
  uint8_t next_packet_high;
  uint8_t length_low;
  uint8_t length_high;
  uint8_t status_low;
  uint8_t status_high;
} enc28j60_rx_header_t;

typedef struct
{
  uint16_t byte_cnt;

  uint8_t collision_cnt : 4;
  uint8_t crc_err : 1;
  uint8_t len_check_err : 1;
  uint8_t len_out_range : 1;
  uint8_t tx_done : 1;

  uint8_t multicast : 1;
  uint8_t broadcast : 1;
  uint8_t pkt_defer : 1;
  uint8_t excessive_defer : 1;
  uint8_t excessive_collision : 1;
  uint8_t late_collision : 1;
  uint8_t giant : 1;
  uint8_t underrun : 1;

  uint16_t bytes_on_wire;

  uint8_t ctrl_frame : 1;
  uint8_t pause_ctrl_frame : 1;
  uint8_t backpressure_app : 1;
  uint8_t vlan_frame : 1;
} enc28j60_tsv_t;

typedef struct
{
  esp_eth_mac_t parent;
  esp_eth_mediator_t* eth;
  const eth_mac_config_t* mac_config;
  spi_device_t spi_hdl;
  volatile uint32_t spi_lock;
  SemaphoreHandle_t reg_trans_lock;
  SemaphoreHandle_t tx_ready_sem;
  TaskHandle_t rx_task_hdl;
  uint32_t sw_reset_timeout_ms;
  volatile uint32_t next_packet_ptr;
  uint32_t last_tsv_addr;
  int int_gpio_num;
  int interrupt_vector;
  uint8_t addr[6];
  uint8_t last_bank;
  bool packets_remain;
  bool run_task;
  uint32_t tx_start_ticks;
  eth_enc28j60_rev_t revision;
} emac_enc28j60_t;

static emac_enc28j60_t* gemac = NULL;

static TaskHandle_t waiting_task = NULL;

ATTR_NC __attribute__((optimize("O3"))) static inline bool
enc28j60_spi_lock(emac_enc28j60_t* emac)
{
    bool acquired = false;

    
    while (1) 
    {
        __disable_irq();
        if (emac->spi_lock == 0) {
            emac->spi_lock = 1;
            __enable_irq();
            acquired = true;
            break;
        }
        waiting_task = xTaskGetCurrentTaskHandle();
        __enable_irq();
#ifdef ENC28J60_DEBUG        
        nrc_usr_print("[%s] yielding...\n",__func__);
#endif
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    return acquired;
}

ATTR_NC __attribute__((optimize("O3"))) static inline bool
enc28j60_spi_unlock(emac_enc28j60_t* emac)
{
    __disable_irq();
    emac->spi_lock = 0;
    if (waiting_task) {
      xTaskNotifyGive(waiting_task);
    }
    waiting_task = NULL;
    __enable_irq();

    return true;
}

ATTR_NC __attribute__((optimize("O3"))) static inline bool
enc28j60_reg_trans_lock(emac_enc28j60_t* emac)
{
  return xSemaphoreTake(emac->reg_trans_lock, pdMS_TO_TICKS(ENC28J60_REG_TRANS_LOCK_TIMEOUT_MS)) == pdTRUE;
}

ATTR_NC __attribute__((optimize("O3"))) static inline bool
enc28j60_reg_trans_unlock(emac_enc28j60_t* emac)
{
  return xSemaphoreGive(emac->reg_trans_lock) == pdTRUE;
}

/**
 * @brief ERXRDPT need to be set always at odd addresses
 */
ATTR_NC __attribute__((optimize("O3"))) static inline uint32_t
enc28j60_next_ptr_align_odd(uint32_t next_packet_ptr,
                            uint32_t start,
                            uint32_t end)
{
  uint32_t erxrdpt;

  if ((next_packet_ptr - 1 < start) || (next_packet_ptr - 1 > end)) {
    erxrdpt = end;
  } else {
    erxrdpt = next_packet_ptr - 1;
  }

  return erxrdpt;
}

/**
 * @brief Calculate wrap around when reading beyond the end of the RX buffer
 */
ATTR_NC __attribute__((optimize("O3"))) static inline uint32_t
enc28j60_rx_packet_start(uint32_t start_addr, uint32_t off)
{
  if (start_addr + off > ENC28J60_BUF_RX_END) {
    return (start_addr + off) -
           (ENC28J60_BUF_RX_END - ENC28J60_BUF_RX_START + 1);
  } else {
    return start_addr + off;
  }
}

ATTR_NC __attribute__((optimize("O3"))) static u8
spi_read_op(spi_device_t* spi, u8 op, u8 addr)
{
  u8 tx_buf[2];
  u8 rx_buf[4];
  u8 val = 0;
  int slen = SPI_OPLEN;

  /* do dummy read if needed */
  if (addr & SPRD_MASK)
    slen++;

  tx_buf[0] = op | (addr & ADDR_MASK);

  enc28j60_spi_lock(gemac);
  nrc_spi_start_xfer(spi);
  nrc_spi_xfer(spi, tx_buf, rx_buf, 1);
  memset(tx_buf, 0, sizeof(tx_buf));
  nrc_spi_xfer(spi, tx_buf, rx_buf, slen);
  nrc_spi_stop_xfer(spi);
  enc28j60_spi_unlock(gemac);
  
  return val = rx_buf[slen - 1];
}

ATTR_NC __attribute__((optimize("O3"))) static int
spi_write_op(spi_device_t* spi, u8 op, u8 addr, u8 val)
{
  int ret;

  memset(spi_transfer_buf, 0, SPI_TRANSFER_BUF_LEN);

  spi_transfer_buf[0] = op | (addr & ADDR_MASK);
  spi_transfer_buf[1] = val;

  enc28j60_spi_lock(gemac);
  nrc_spi_start_xfer(spi);
  ret = nrc_spi_xfer(spi, spi_transfer_buf, spi_transfer_buf, 2);
  nrc_spi_stop_xfer(spi);
  enc28j60_spi_unlock(gemac);
  
  return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static int
spi_write(spi_device_t* spi, u8* data, int len)
{
  int ret = NRC_SUCCESS;

  enc28j60_spi_lock(gemac);
  nrc_spi_start_xfer(spi);
  ret = nrc_spi_xfer(spi, data, data, len);
  nrc_spi_stop_xfer(spi);
  enc28j60_spi_unlock(gemac);
  
  return ret;
}

#ifdef ETHERNET_SPI_DMA
ATTR_NC __attribute__((optimize("O3"))) static void spi_dma_start_xfer(spi_device_t *spi)
{
  // nrc_usr_print("[%s]\n",__func__);
  spi_dma_busy = true;
  nrc_ssp_flush(spi->controller);
  nrc_gpio_outputb(spi->pin_cs, 0);
}

ATTR_NC __attribute__((optimize("O3"))) static void spi_dma_stop_xfer(spi_device_t *spi)
{
  // nrc_usr_print("[%s]\n",__func__);
  nrc_gpio_outputb(spi->pin_cs, 1);
}

static int init_dma_desc(bool tx, uint32_t addr, size_t size)
{
	dma_peri_t *peri;
	dma_desc_t *desc = NULL;
	int i = 0;

        // nrc_usr_print("[%s] for %s.\n",__func__, tx? "TX": "RX");
	if (tx) {
		peri = &spi_tx_peri;
		desc = &tx_desc;
	} else {
		peri = &spi_rx_peri;
		desc = &rx_desc;
	}

	if (tx) {
		nrc_dma_desc_init(desc, addr, spi_tx_peri.Addr, size);
		nrc_dma_desc_set_addr_inc(desc, true, false);
		nrc_dma_desc_set_ahb_master(desc, NRC_DMA_AHB_M1, NRC_DMA_AHB_M1);
	} else {
		nrc_dma_desc_init(desc, spi_rx_peri.Addr, addr, size);
		nrc_dma_desc_set_addr_inc(desc, false, true);
		nrc_dma_desc_set_ahb_master(desc, NRC_DMA_AHB_M2, NRC_DMA_AHB_M2);
	}
	nrc_dma_desc_set_width(desc, NRC_DMA_WIDTH_8, NRC_DMA_WIDTH_8);
	nrc_dma_desc_set_bsize(desc, NRC_DMA_BSIZE_1, NRC_DMA_BSIZE_1);
	nrc_dma_desc_set_inttc(desc, true);
	nrc_dma_desc_set_protection(desc, false, false, false);
	desc->Next = NULL;
	//	nrc_usr_print("[%s] returns for %s.\n",__func__, tx? "TX": "RX");

	return 0;
}

#define SPI_BSY_BIT    (1 << 4)
static inline bool spi_is_idle(void)
{
    return (RegSSP_SR(SSP0_BASE_ADDR) & SPI_BSY_BIT) == 0;
}

#if defined(ETHERNET_SPI_DMA) && defined(SPI_DMA_CHAINED)
static bool start_next_dma_transfer(void) 
{
    for (int i = 0; i < RX_NODE_COUNT; i++) {
        if (rx_nodes[i].state == RX_NODE_IDLE) {
            rx_nodes[i].aligned_packet_buffer = MEM_MALLOC(ALIGNED_MAX_PACKET_SIZE);
            if (!rx_nodes[i].aligned_packet_buffer) {
                return false;  // Allocation failure
            }

            rx_nodes[i].state = RX_NODE_DMA_IN_PROGRESS;
            current_dma_node = &rx_nodes[i];

            spi_dma_read(current_dma_node->header, HEADER_SIZE);
            uint16_t packet_length = enc28j60_peek_packet_length();
            spi_dma_read(current_dma_node->aligned_packet_buffer, packet_length);

            return true;
        }
    }
    return false;  // No idle nodes available
}
#endif

// Called from ISR (RX/TX DMA completion)
static void dma_done_isr(dma_dir_t dir)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    
#if defined(ETHERNET_SPI_DMA) && defined(SPI_DMA_CHAINED)

    if (current_dma_node == NULL || current_dma_node->state != RX_NODE_DMA_IN_PROGRESS) {
        // Invalid state, reset RX logic
        enc28j60_reset_rx_logic();
        return;
    }

    // Extract packet length from header (first 2 bytes after status bytes)
    uint16_t rx_packet_length = current_dma_node->header[2] | (current_dma_node->header[3] << 8);
    current_dma_node->packet_length = rx_packet_length;

    // Validate packet length (standard Ethernet frame min and max lengths)
    if (rx_packet_length < 60 || rx_packet_length > 1518) {
        current_dma_node->state = RX_NODE_IDLE;
        enc28j60_advance_rx_read_pointer(rx_packet_length);
    } else {
        current_dma_node->state = RX_NODE_READY_FOR_LWIP;
        // Signal LwIP task that packet is ready
        xSemaphoreGiveFromISR(rx_packet_ready_semaphore, &higher_priority_task_woken);
    }

    // Immediately start DMA for next packet if available
    if (enc28j60_has_pending_packet() && start_next_dma_transfer()) {
        // next packet DMA initiated successfully
    } else {
        current_dma_node = NULL; // No packets pending or error
    }
#endif
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

ATTR_NC __attribute__((optimize("O3"))) static void enc28j60_spi_rx_dma_isr (int channel)
{
	BaseType_t	xHigherPriorityTaskWoken = pdFALSE;
	// nrc_usr_print("[%s] channel (%d).\n\n",__func__, channel);
	xSemaphoreGiveFromISR(rx_sem, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken == pdTRUE)
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  
      dma_done_isr(DMA_DIR_RX);
}

ATTR_NC __attribute__((optimize("O3"))) static void enc28j60_spi_rx_dma_err_isr (int channel)
{
	BaseType_t	xHigherPriorityTaskWoken = pdFALSE;
	// nrc_usr_print("[%s] channel (%d) failed.\n\n",__func__, channel);
	xSemaphoreGiveFromISR(rx_sem, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken == pdTRUE)
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

ATTR_NC __attribute__((optimize("O3"))) static void enc28j60_spi_tx_dma_isr (int channel)
{
	BaseType_t	xHigherPriorityTaskWoken = pdFALSE;
	// nrc_usr_print("[%s] channel (%d).\n\n",__func__, channel);
	xSemaphoreGiveFromISR(tx_sem, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken == pdTRUE)
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  
    dma_done_isr(DMA_DIR_TX);
}

ATTR_NC __attribute__((optimize("O3"))) static void enc28j60_spi_tx_dma_err_isr (int channel)
{
	BaseType_t	xHigherPriorityTaskWoken = pdFALSE;
	// nrc_usr_print("[%s] spi_tx_dma_err_isr(%d) failed.\n\n",__func__, channel);
	xSemaphoreGiveFromISR(tx_sem, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken == pdTRUE)
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

ATTR_NC __attribute__((optimize("O3"))) void enc28j60_spi_dma_read(uint8_t *addr, uint8_t *data, uint32_t size)
{
	int err = 1;

	init_dma_desc(true, (uint32_t) addr, size);
	nrc_dma_desc_set_inttc(&tx_desc, false);
	init_dma_desc(false, (uint32_t) data, size);
	err =  nrc_dma_start(DMA_RX_CHANNEL, &rx_desc);
	if (err) {
		nrc_usr_print("[%s] nrc_dma_start() failed with err %d.\n",__func__, err);
		return;
	}

	spi_dma_start_xfer(&gemac->spi_hdl);
	err =  nrc_dma_start(DMA_TX_CHANNEL, &tx_desc);
	if (err) {
		nrc_usr_print("[%s] nrc_dma_start() failed with err %d.\n",__func__, err);
		return;
	}
	nrc_ssp_dma(gemac->spi_hdl.controller, SSP_DMA_TX | SSP_DMA_RX, true);

	xSemaphoreTake(rx_sem, portMAX_DELAY);
	nrc_dma_stop(DMA_TX_CHANNEL);
	nrc_dma_stop(DMA_RX_CHANNEL);
	nrc_ssp_dma(gemac->spi_hdl.controller, SSP_DMA_TX | SSP_DMA_RX, false);

	spi_dma_stop_xfer(&gemac->spi_hdl);
}

ATTR_NC __attribute__((optimize("O3"))) void enc28j60_spi_dma_write(uint8_t *data, uint32_t size)
{
	int err = 1;

	init_dma_desc(true, (uint32_t) data, size);

	spi_dma_start_xfer(&gemac->spi_hdl);
	err =  nrc_dma_start(DMA_TX_CHANNEL, &tx_desc);
	if (err) {
		nrc_usr_print("[%s] nrc_dma_start() failed with err %d.\n",__func__, err);
		return;
	}

	nrc_ssp_dma(gemac->spi_hdl.controller, SSP_DMA_TX, true);
	xSemaphoreTake(tx_sem, portMAX_DELAY);
	nrc_dma_stop(DMA_TX_CHANNEL);
	nrc_ssp_dma(gemac->spi_hdl.controller, SSP_DMA_TX, false);
	spi_dma_stop_xfer(&gemac->spi_hdl);
}
#endif /* #ifdef ETHERNET_SPI_DMA */

/* NOTE: enc28j60 writes on rising edge and reads on falling edge, */
/*       Newracom SDK hides HW so this capability is not exposed    */
/*       code below mitigates this issue                           */
ATTR_NC __attribute__((optimize("O3"))) static int
spi_read_buf(spi_device_t* spi, int len, u8* data)
{
  int ret = NRC_SUCCESS;
  uint8_t cmd = ENC28J60_READ_BUF_MEM;  // 0x3A command for read buffer
    
  if (len > SPI_TRANSFER_BUF_LEN) {
    return NRC_FAIL;
  }
    
  memset(spi_transfer_buf, 0, SPI_TRANSFER_BUF_LEN);

#ifdef ETHERNET_SPI_DMA
  // DMA is only for large transfers
  // if(len > 8) {
    // Use DMA to send RBM command and read len+1 bytes (includes dummy)
    enc28j60_spi_lock(gemac);
    enc28j60_spi_dma_read(&cmd, spi_transfer_buf, len + 1);
    // The first byte in spi_transfer_buf is dummy (from sending cmd), actual data starts at index 1
    memcpy(data, &spi_transfer_buf[1], len);
    enc28j60_spi_unlock(gemac);
  // } else {
  //   E(TT_NET, "using slow read for %d bytes instead of DMA\n", len);
  //   nrc_spi_read_values(spi, cmd, spi_transfer_buf, len);
  //   memcpy(data, spi_transfer_buf, len);
  // }
#else
    // Fallback: byte-by-byte SPI transfer (mitigates timing issues if DMA is unavailable)
    enc28j60_spi_lock(gemac);
    nrc_spi_start_xfer(spi);
    spi_transfer_buf[0] = cmd;
    // clock out one dummy + len bytes by transferring len+1 bytes total
    // (Using a loop or blocking xfer for each byte if needed to match timing)
    for (int i = 0; i < len + 1; ++i) {
        uint8_t tx = (i == 0 ? cmd : 0x00);
        uint8_t rx = 0x00;
        nrc_spi_xfer(spi, &tx, &rx, 1);  // transfer 1 byte
        spi_transfer_buf[i] = rx;
    }
    nrc_spi_stop_xfer(spi);
    memcpy(data, &spi_transfer_buf[1], len);
    enc28j60_spi_unlock(gemac);
#endif
    return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static int
spi_write_buf(spi_device_t* spi, int len, const u8* data)
{
  int ret = NRC_SUCCESS;
  
  if (len > SPI_TRANSFER_BUF_LEN - 1 || len <= 0) {
    ret = NRC_FAIL;
  }
  
  enc28j60_spi_lock(gemac);
  spi_transfer_buf[0] = ENC28J60_WRITE_BUF_MEM;  // 0x7A command for write buffer
  memcpy(&spi_transfer_buf[1], data, len);
#ifdef ETHERNET_SPI_DMA
    // if(len > 8) {
    // Use DMA to send command + data in one transfer
    enc28j60_spi_dma_write(spi_transfer_buf, len + 1);
    ret = NRC_SUCCESS;  // assume success if dma call returns
    // } else {
// }
#else
    // Fallback to blocking SPI transfer if DMA is disabled or fails
    ret = spi_write(spi, spi_transfer_buf, len + 1);
#endif
    enc28j60_spi_unlock(gemac);
    return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
spi_device_transmit(spi_device_t spi_hdl,
                            spi_transaction_t* trans)
{
  nrc_err_t retval = NRC_FAIL;

  // E(TT_NET, "[%s]\n", __func__);
  
  switch (trans->cmd) {
    case ENC28J60_SPI_CMD_WCR:
    // E(TT_NET, "[%s] case ENC28J60_SPI_CMD_WCR\n", __func__);
      if (spi_write_op(&spi_hdl,
                       ENC28J60_WRITE_CTRL_REG,
                       trans->addr,
                       trans->tx_data[0]) == NRC_SUCCESS) {
        retval = NRC_SUCCESS;
      }
      break;
    case ENC28J60_SPI_CMD_RCR:

      // E(TT_NET, "[%s] case ENC28J60_SPI_CMD_RCR\n", __func__);
      
      enc28j60_spi_lock(gemac);
      nrc_spi_writebyte_value(&spi_hdl, (trans->addr & ADDR_MASK) | ENC28J60_READ_CTRL_REG, 0);
      enc28j60_spi_unlock(gemac);
      
      if (trans->length / 8 == sizeof(uint8_t)) {
        enc28j60_spi_lock(gemac);
        if (nrc_spi_readbyte_value(&spi_hdl,
                                   (trans->addr & ADDR_MASK) |
                                     ENC28J60_READ_CTRL_REG,
                                   &trans->rx_data[0]) == NRC_SUCCESS) {
          retval = NRC_SUCCESS;
        }
         enc28j60_spi_unlock(gemac);
      } else if (trans->length / 8 == 2 * sizeof(uint8_t)) {

        enc28j60_spi_lock(gemac);
        if (nrc_spi_readbyte_value(&spi_hdl,
                                   (trans->addr & ADDR_MASK) |
                                     ENC28J60_READ_CTRL_REG,
                                   &trans->rx_data[0]) == NRC_SUCCESS) {
          retval = NRC_SUCCESS;
        }
        enc28j60_spi_unlock(gemac);
        
        enc28j60_spi_lock(gemac);
        if (nrc_spi_readbyte_value(&spi_hdl,
                                   (trans->addr & ADDR_MASK) |
                                     ENC28J60_READ_CTRL_REG,
                                   &trans->rx_data[1]) == NRC_SUCCESS) {
          retval = NRC_SUCCESS;
        }
        enc28j60_spi_unlock(gemac);
      } else if (trans->length / 8 > 2 * sizeof(uint8_t)) {
        enc28j60_spi_lock(gemac);
        if (nrc_spi_read_values(&spi_hdl,
                                (trans->addr & ADDR_MASK) |
                                  ENC28J60_READ_CTRL_REG,
                                trans->rx_data,
                                trans->length / 8) == NRC_SUCCESS) {
          retval = NRC_SUCCESS;
        }
        enc28j60_spi_unlock(gemac);
      }

      break;
    case ENC28J60_SPI_CMD_BFS:
      // E(TT_NET, "[%s] case ENC28J60_SPI_CMD_BFS\n", __func__);
      if (spi_write_op(
            &spi_hdl, ENC28J60_BIT_FIELD_SET, trans->addr, trans->tx_data[0]) ==
          NRC_SUCCESS) {
        retval = NRC_SUCCESS;
      }
      break;
    case ENC28J60_SPI_CMD_BFC:
      // E(TT_NET, "[%s] case ENC28J60_SPI_CMD_BFC\n", __func__);
      if (spi_write_op(
            &spi_hdl, ENC28J60_BIT_FIELD_CLR, trans->addr, trans->tx_data[0]) ==
          NRC_SUCCESS) {
        retval = NRC_SUCCESS;
      }
      break;
    case ENC28J60_SPI_CMD_WBM:
      // E(TT_NET, "[%s] case ENC28J60_SPI_CMD_WBM\n", __func__);
      if (trans->length / 8 == 1 &&
          spi_write_op(
            &spi_hdl, ENC28J60_WRITE_BUF_MEM, 0, *(uint8_t*)trans->tx_buffer) ==
            NRC_SUCCESS) {
        retval = NRC_SUCCESS;
      } else if (spi_write_buf(&spi_hdl, trans->length / 8, trans->tx_buffer) ==
                 NRC_SUCCESS) {
        retval = NRC_SUCCESS;
      }

      break;
    case ENC28J60_SPI_CMD_RBM:
      // E(TT_NET, "[%s] case ENC28J60_SPI_CMD_RBM\n", __func__);
      if (spi_read_buf(&spi_hdl, trans->length / 8, trans->rx_buffer) ==
          NRC_SUCCESS) {
        retval = NRC_SUCCESS;
      }
      break;
    case ENC28J60_SPI_CMD_SRC:
      // E(TT_NET, "[%s] case ENC28J60_SPI_CMD_SRC\n", __func__);
      enc28j60_spi_lock(gemac);
      if (nrc_spi_writebyte_value(
            &spi_hdl, (trans->addr & ADDR_MASK) | ENC28J60_SOFT_RESET, 0) ==
          NRC_SUCCESS) {
        retval = NRC_SUCCESS;
      }
      enc28j60_spi_unlock(gemac);
      break;
  }


  return retval;
}

ATTR_NC __attribute__((optimize("O3"))) static int
gpio_get_level(int gpio_num)
{
  nrc_err_t err;
  int level;

  err = nrc_gpio_inputb(gpio_num, &level);

  if (err != NRC_SUCCESS) {
    return -1;
  }

  return level;
}

ATTR_NC __attribute__((optimize("O3"))) static void
enc28j60_gpio_config(NRC_GPIO_PIN gpio_num, bool input)
{
  NRC_GPIO_CONFIG config;

  config.gpio_pin = gpio_num;
  if (input) {
    config.gpio_dir = GPIO_INPUT;
  } else {
    config.gpio_dir = GPIO_OUTPUT;
  }

  config.gpio_mode = GPIO_FLOATING;
  config.gpio_alt = GPIO_FUNC;

  nrc_gpio_config(&config);
}

ATTR_NC __attribute__((optimize("O3"))) static void
enc28j60_init_spi(spi_device_t* spi_dev)
{
  int ret;

  ret = nrc_spi_init_cs(spi_dev->pin_cs);
  ret = nrc_spi_master_init(spi_dev);
  ret = nrc_spi_enable(spi_dev, true);
  _delay_ms(100);
}

/**
 * @brief SPI operation wrapper for writing ENC28J60 internal register
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_do_register_write(emac_enc28j60_t* emac,
                           uint8_t reg_addr,
                           uint8_t value)
{
  nrc_err_t ret = NRC_SUCCESS;
  spi_transaction_t trans = { .cmd =
                                ENC28J60_SPI_CMD_WCR, // Write control register
                              .addr = reg_addr,
                              .length = 8,
                              .flags = SPI_TRANS_USE_TXDATA,
                              .tx_data = { [0] = value } };

  if (spi_device_transmit(emac->spi_hdl, &trans) != NRC_SUCCESS) {
    // ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
    ret = NRC_FAIL;
  }
  
  return ret;
}

/**
 * @brief SPI operation wrapper for reading ENC28J60 internal register
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_do_register_read(emac_enc28j60_t* emac,
                          bool is_eth_reg,
                          uint8_t reg_addr,
                          uint8_t* value)
{
  nrc_err_t ret = NRC_SUCCESS;
  spi_transaction_t trans = {
    .cmd = ENC28J60_SPI_CMD_RCR, // Read control register
    .addr = reg_addr,
    .length = is_eth_reg ? 8 : 16, // read operation is different for ETH
                                   // register and non-ETH register
    .flags = SPI_TRANS_USE_RXDATA
  };

  // E(TT_NET, "[%s]\n", __func__);
  if (spi_device_transmit(emac->spi_hdl, &trans) != NRC_SUCCESS) {
    // ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
    ret = NRC_FAIL;
  } else {
    *value = is_eth_reg ? trans.rx_data[0] : trans.rx_data[1];
  }
  
  return ret;
}

/**
 * @brief SPI operation wrapper for bitwise setting ENC28J60 internal register
 * @note can only be used for ETH registers
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_do_bitwise_set(emac_enc28j60_t* emac, uint8_t reg_addr, uint8_t mask)
{
  nrc_err_t ret = NRC_SUCCESS;
  spi_transaction_t trans = { .cmd = ENC28J60_SPI_CMD_BFS, // Bit field set
                              .addr = reg_addr,
                              .length = 8,
                              .flags = SPI_TRANS_USE_TXDATA,
                              .tx_data = { [0] = mask } };

  if (spi_device_transmit(emac->spi_hdl, &trans) != NRC_SUCCESS) {
      // ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
    ret = NRC_FAIL;
  }
  
  return ret;
}

/**
 * @brief SPI operation wrapper for bitwise clearing ENC28J60 internal register
 * @note can only be used for ETH registers
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_do_bitwise_clr(emac_enc28j60_t* emac, uint8_t reg_addr, uint8_t mask)
{
  nrc_err_t ret = NRC_SUCCESS;
  spi_transaction_t trans = { .cmd = ENC28J60_SPI_CMD_BFC, // Bit field clear
                              .addr = reg_addr,
                              .length = 8,
                              .flags = SPI_TRANS_USE_TXDATA,
                              .tx_data = { [0] = mask } };

  if (spi_device_transmit(emac->spi_hdl, &trans) != NRC_SUCCESS) {
    // ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
    ret = NRC_FAIL;
  }
  
  return ret;
}

/**
 * @brief SPI operation wrapper for writing ENC28J60 internal memory
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_do_memory_write(emac_enc28j60_t* emac, uint8_t* buffer, uint32_t len)
{
  nrc_err_t ret = NRC_SUCCESS;
  spi_transaction_t trans = { .cmd =
                                ENC28J60_SPI_CMD_WBM, // Write buffer memory
                              .addr = 0x1A,
                              .length = len * 8,
                              .tx_buffer = buffer };

  if (spi_device_transmit(emac->spi_hdl, &trans) != NRC_SUCCESS) {
    ret = NRC_FAIL;
  }
    
  return ret;
}

/**
 * @brief SPI operation wrapper for reading ENC28J60 internal memory
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_do_memory_read(emac_enc28j60_t* emac, uint8_t* buffer, uint32_t len)
{
  nrc_err_t ret = NRC_SUCCESS;
  spi_transaction_t trans = { .cmd = ENC28J60_SPI_CMD_RBM, // Read buffer memory
                              .addr = 0x1A,
                              .length = len * 8,
                              .rx_buffer = buffer };

  if (spi_device_transmit(emac->spi_hdl, &trans) != NRC_SUCCESS) {
      // ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
      ret = NRC_FAIL;
  }
  
  return ret;
}

/**
 * @brief Switch ENC28J60 register bank
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_switch_register_bank(emac_enc28j60_t* emac, uint8_t bank)
{
  nrc_err_t ret = NRC_SUCCESS;

  if (bank != emac->last_bank) {

    MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, 0x03) == NRC_SUCCESS,
              "clear ECON1[1:0] failed",
              out,
              NRC_FAIL);
    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, bank & 0x03) ==
                NRC_SUCCESS,
              "set ECON1[1:0] failed",
              out,
              NRC_FAIL);
    emac->last_bank = bank;
  }
out:
  return ret;
}

/**
 * @brief Write ENC28J60 register
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_register_write(emac_enc28j60_t* emac, uint16_t reg_addr, uint8_t value)
{
  nrc_err_t ret = NRC_SUCCESS;

  if (enc28j60_reg_trans_lock(emac)) {
    MAC_CHECK(enc28j60_switch_register_bank(emac, (reg_addr & 0xF00) >> 8) ==
                NRC_SUCCESS,
              "switch bank failed",
              out,
              NRC_FAIL);
    MAC_CHECK(enc28j60_do_register_write(emac, reg_addr & 0xFF, value) ==
                NRC_SUCCESS,
              "write register failed",
              out,
              NRC_FAIL);
    enc28j60_reg_trans_unlock(emac);
  } else {
    ret = NRC_FAIL;
  }
out:
  enc28j60_reg_trans_unlock(emac);
  return ret;
}

/**
 * @brief Read ENC28J60 register
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_register_read(emac_enc28j60_t* emac, uint16_t reg_addr, uint8_t* value)
{
  nrc_err_t ret = NRC_SUCCESS;

  // E(TT_NET, "[%s]\n", __func__);
  
  if (enc28j60_reg_trans_lock(emac)) {
    // E(TT_NET, "[%s] after enc28j60_reg_trans_lock\n", __func__);
    MAC_CHECK(enc28j60_switch_register_bank(emac, (reg_addr & 0xF00) >> 8) ==
                NRC_SUCCESS,
              "switch bank failed",
              out,
              NRC_FAIL);
    // E(TT_NET, "[%s] after enc28j60_switch_register_bank\n", __func__);
    MAC_CHECK(enc28j60_do_register_read(
                emac, !(reg_addr & 0xF000), reg_addr & 0xFF, value) == NRC_SUCCESS,
              "read register failed",
              out,
              NRC_FAIL);
    // E(TT_NET, "[%s] after enc28j60_do_register_read\n", __func__);
    enc28j60_reg_trans_unlock(emac);
  } else {
    ret = NRC_FAIL;
  }
out:
  // enc28j60_reg_trans_unlock(emac);
  return ret;
}


/**
 * @brief SPI operation wrapper for resetting ENC28J60
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_reset(emac_enc28j60_t* emac)
{
  nrc_err_t ret = NRC_SUCCESS;
  uint8_t estat = 0;
  int retry = MAX_REG_RETRY_COUNT;
    
  spi_transaction_t trans = {
    .cmd = ENC28J60_SPI_CMD_SRC, // Soft reset
    .addr = 0x1F,
  };

  
  if (spi_device_transmit(emac->spi_hdl, &trans) != NRC_SUCCESS) {
      ret = NRC_FAIL;
  }

  _delay_ms(2);
  
  do {
      MAC_CHECK(enc28j60_register_read(emac, ENC28J60_ESTAT, &estat) == NRC_SUCCESS, "Read ESTAT failed after reset", out, NRC_FAIL);
      if (estat & ESTAT_CLKRDY) {
          E(TT_NET, "[%s] success...\n", __func__);
          return NRC_SUCCESS; // Reset successful, chip ready
      }
      delay_us(1);
  } while (--retry);

  E(TT_NET, "[%s] failed...\n", __func__);
out:

  return ret;
}

/**
 * @brief Read ENC28J60 internal memroy
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_read_packet(emac_enc28j60_t* emac,
                     uint32_t addr,
                     uint8_t* packet,
                     uint32_t len)
{
  nrc_err_t ret = NRC_SUCCESS;

  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERDPTL, addr & 0xFF) == NRC_SUCCESS, "write ERDPTL failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERDPTH, (addr & 0xFF00) >> 8) == NRC_SUCCESS, "write ERDPTH failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_do_memory_read(emac, packet, len) == NRC_SUCCESS, "read memory failed", out,NRC_FAIL);

out:
  return ret;
}

/**
 * @brief Write ENC28J60 internal PHY register
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_write_phy_reg(esp_eth_mac_t* mac,
                            uint32_t phy_addr,
                            uint32_t phy_reg,
                            uint32_t reg_value)
{
  nrc_err_t ret = NRC_SUCCESS;

  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);
  uint8_t mii_status = 0;

  /* check if phy access is in progress */
  MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status) == NRC_SUCCESS, "read MISTAT failed", out, NRC_FAIL);
  MAC_CHECK(!(mii_status & MISTAT_BUSY), "phy is busy", out, NRC_FAIL);

  /* tell the PHY address to write */
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MIREGADR, phy_reg & 0xFF) == NRC_SUCCESS, "write MIREGADR failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MIWRL, reg_value & 0xFF) == NRC_SUCCESS, "write MIWRL failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MIWRH, (reg_value & 0xFF00) >> 8) == NRC_SUCCESS, "write MIWRH failed", out, NRC_FAIL);

  /* polling the busy flag */
  uint32_t to = 0;
  do {
    delay_us(5);
    MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status) == NRC_SUCCESS, "read MISTAT failed", out, NRC_FAIL);
    to += 5;
  } while ((mii_status & MISTAT_BUSY) && to < ENC28J60_PHY_OPERATION_TIMEOUT_US);
  MAC_CHECK(!(mii_status & MISTAT_BUSY), "phy is busy", out, NRC_FAIL);
out:
  return ret;
}

/**
 * @brief Read ENC28J60 internal PHY register
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_read_phy_reg(esp_eth_mac_t* mac,
                           uint32_t phy_addr,
                           uint32_t phy_reg,
                           uint32_t* reg_value)
{
  nrc_err_t ret = NRC_SUCCESS;

  MAC_CHECK(reg_value, "can't set reg_value to null", out, NRC_FAIL);
  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);
  uint8_t mii_status = 0;
  uint8_t mii_cmd;

  /* check if phy access is in progress */
  MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status) == NRC_SUCCESS, "read MISTAT failed", out, NRC_FAIL);
  MAC_CHECK(!(mii_status & MISTAT_BUSY), "phy is busy", out, NRC_FAIL);

  /* tell the PHY address to read */
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MIREGADR, phy_reg & 0xFF) == NRC_SUCCESS, "write MIREGADR failed", out, NRC_FAIL);
  mii_cmd = MICMD_MIIRD;
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MICMD, mii_cmd) == NRC_SUCCESS, "write MICMD failed", out, NRC_FAIL);

  MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status) == NRC_SUCCESS, "read MISTAT failed", out, NRC_FAIL);
  
  /* polling the busy flag */
  uint32_t to = 0;
  while ((mii_status & MISTAT_BUSY) &&
          to < ENC28J60_PHY_OPERATION_TIMEOUT_US) {
    delay_us(5);
    MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status) == NRC_SUCCESS, "read MISTAT failed", out, NRC_FAIL);
    to += 5;
  }
  
  MAC_CHECK(!(mii_status & MISTAT_BUSY), "phy is busy", out, NRC_FAIL);

  mii_cmd &= (~MICMD_MIIRD);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MICMD, mii_cmd) == NRC_SUCCESS, "write MICMD failed", out, NRC_FAIL);

  uint8_t value_l = 0;
  uint8_t value_h = 0;
  MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MIRDL, &value_l) == NRC_SUCCESS, "read MIRDL failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MIRDH, &value_h) == NRC_SUCCESS, "read MIRDH failed", out, NRC_FAIL);
  *reg_value = (value_h << 8) | value_l;
out:
  return ret;
}

/**
 * @brief Set mediator for Ethernet MAC
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_set_mediator(esp_eth_mac_t* mac, esp_eth_mediator_t* eth)
{
  nrc_err_t ret = NRC_SUCCESS;

  MAC_CHECK(eth, "can't set mac's mediator to null", out, NRC_FAIL);
  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);
  emac->eth = eth;
out:
  return ret;
}

/**
 * @brief Verify chip revision ID
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_verify_id(emac_enc28j60_t* emac)
{
  nrc_err_t ret = NRC_SUCCESS;

  enc28j60_switch_register_bank(emac, 0x03);
  MAC_CHECK(enc28j60_register_read(emac, ENC28J60_EREVID, (uint8_t*)&emac->revision) == NRC_SUCCESS, "read EREVID failed", out, NRC_FAIL);
  
  MAC_CHECK(emac->revision >= ENC28J60_REV_B1 && emac->revision <= ENC28J60_REV_B7, "wrong chip ID", out, NRC_FAIL);
out:
  return ret;
}

/**
 * @brief Write mac address to internal registers
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_set_mac_addr(emac_enc28j60_t* emac)
{
  nrc_err_t ret = NRC_SUCCESS;

  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MAADR6, emac->addr[5]) ==
              NRC_SUCCESS,
            "write MAADR6 failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MAADR5, emac->addr[4]) ==
              NRC_SUCCESS,
            "write MAADR5 failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MAADR4, emac->addr[3]) ==
              NRC_SUCCESS,
            "write MAADR4 failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MAADR3, emac->addr[2]) ==
              NRC_SUCCESS,
            "write MAADR3 failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MAADR2, emac->addr[1]) ==
              NRC_SUCCESS,
            "write MAADR2 failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MAADR1, emac->addr[0]) ==
              NRC_SUCCESS,
            "write MAADR1 failed",
            out,
            NRC_FAIL);
out:
  return ret;
}

/**
 * @brief   Clear multicast hash table
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_clear_multicast_table(emac_enc28j60_t* emac)
{
  nrc_err_t ret = NRC_SUCCESS;

  for (int i = 0; i < 7; i++) {
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_EHT0 + i, 0x00) == NRC_SUCCESS, "write ENC28J60_EHT%d failed", out, NRC_FAIL, i);
  }
out:
  return ret;
}

/**
 * @brief Default setup for ENC28J60 internal registers
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_setup_default(emac_enc28j60_t* emac)
{
  nrc_err_t ret = NRC_SUCCESS;

  // set up receive buffer start + end
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXSTL, ENC28J60_BUF_RX_START & 0xFF) == NRC_SUCCESS, "write ERXSTL failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXSTH, (ENC28J60_BUF_RX_START & 0xFF00) >> 8) == NRC_SUCCESS, "write ERXSTH failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXNDL, ENC28J60_BUF_RX_END & 0xFF) == NRC_SUCCESS, "write ERXNDL failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXNDH, (ENC28J60_BUF_RX_END & 0xFF00) >> 8) == NRC_SUCCESS, "write ERXNDH failed", out, NRC_FAIL);
  uint32_t erxrdpt = enc28j60_next_ptr_align_odd(ENC28J60_BUF_RX_START, ENC28J60_BUF_RX_START, ENC28J60_BUF_RX_END);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXRDPTL, erxrdpt & 0xFF) == NRC_SUCCESS, "write ERXRDPTL failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXRDPTH, (erxrdpt & 0xFF00) >> 8) == NRC_SUCCESS, "write ERXRDPTH failed", out, NRC_FAIL);

  // set up transmit buffer start + end
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ETXSTL, ENC28J60_BUF_TX_START & 0xFF) == NRC_SUCCESS, "write ETXSTL failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ETXSTH, (ENC28J60_BUF_TX_START & 0xFF00) >> 8) == NRC_SUCCESS, "write ETXSTH failed", out, NRC_FAIL);

  // set up default filter mode: (unicast OR broadcast OR multicast) AND crc
  // valid
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_BCEN | ERXFCON_MCEN) == NRC_SUCCESS, "write ERXFCON failed", out, NRC_FAIL);

  // enable MAC receive, enable pause control frame on Tx and Rx path
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MACON1, MACON1_MARXEN | MACON1_RXPAUS | MACON1_TXPAUS) == NRC_SUCCESS, "write MACON1 failed", out, NRC_FAIL);
  // enable automatic padding, append CRC, check frame length, half duplex by
  // default (can update at runtime)
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN) == NRC_SUCCESS, "write MACON3 failed", out, NRC_FAIL);
  // enable defer transmission (effective only in half duplex)
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MACON4, MACON4_DEFER) == NRC_SUCCESS, "write MACON4 failed", out, NRC_FAIL);
  // set inter-frame gap (back-to-back)
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MABBIPG, 0x12) == NRC_SUCCESS, "write MABBIPG failed", out, NRC_FAIL);
  // set inter-frame gap (non-back-to-back)
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MAIPGL, 0x12) == NRC_SUCCESS, "write MAIPGL failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MAIPGH, 0x0C) == NRC_SUCCESS, "write MAIPGH failed", out, NRC_FAIL);

out:
  return ret;
}

/**
 * @brief Start enc28j60: enable interrupt and start receive
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_start(esp_eth_mac_t* mac)
{
  nrc_err_t ret = NRC_SUCCESS;

  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);
  
  /* enable interrupt */
  MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, 0xFF) == NRC_SUCCESS, "clear EIR failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_PKTIE | EIE_INTIE | EIE_TXERIE | EIE_RXERIE) == NRC_SUCCESS, "set EIE [PKTIE|INTIE|TXERIE|RXERIE] failed", out, NRC_FAIL);
  
  /* enable rx logic */
  MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_RXEN) == NRC_SUCCESS, "set ECON1.RXEN failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERDPTL, 0x00) == NRC_SUCCESS, "write ERDPTL failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERDPTH, 0x00) == NRC_SUCCESS, "write ERDPTH failed", out, NRC_FAIL);
  delay_us(100);
  
#if defined(ETHERNET_SPI_DMA) && defined(SPI_DMA_CHAIN)
  dma_aligned_pbuf_pool_init();

  for (int i = 0; i < RX_NODE_COUNT; i++) 
  {
    rx_nodes[i].state = NODE_FREE;
    rx_nodes[i].aligned_packet_buffer = NULL;
    rx_nodes[i].packet_length = 0;
  }

  // Schedule the first DMA
  current_dma_node = &rx_nodes[0];
  current_dma_node->aligned_packet_buffer = MEM_MALLOC(PBUF_POOL_BUFSIZE);
  current_dma_node->state = NODE_DMA_BUSY;

  current_dma_node->dma_dir = DMA_DIR_RX;
  enc28j60_schedule_dma(emac, current_dma_node->aligned_packet_buffer, PBUF_POOL_BUFSIZE);
#endif
  
out:
  return ret;
}

/**
 * @brief   Stop enc28j60: disable interrupt and stop receiving packets
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_stop(esp_eth_mac_t* mac)
{
  nrc_err_t ret = NRC_SUCCESS;

  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);
  /* disable interrupt */
  MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_EIE, 0xFF) == NRC_SUCCESS, "clear EIE failed", out, NRC_FAIL);
  /* disable rx */
  MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_RXEN) == NRC_SUCCESS, "clear ECON1.RXEN failed", out, NRC_FAIL);
#if defined(ETHERNET_SPI_DMA) && defined(SPI_DMA_CHAINED)
  dma_aligned_pbuf_pool_deinit();

  for (int i = 0; i < RX_NODE_COUNT; i++) 
  {
    if (rx_nodes[i].aligned_packet_buffer) {
      MEM_FREE(rx_nodes[i].aligned_packet_buffer);
      rx_nodes[i].aligned_packet_buffer = NULL;
    }
    rx_nodes[i].state = NODE_FREE;
  }
#endif
out:
  return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_set_addr(esp_eth_mac_t* mac, uint8_t* addr)
{
  nrc_err_t ret = NRC_SUCCESS;

  MAC_CHECK(addr, "can't set mac addr to null", out, NRC_FAIL);
  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);
  memcpy(emac->addr, addr, 6);
  MAC_CHECK(enc28j60_set_mac_addr(emac) == NRC_SUCCESS,
            "set mac address failed",
            out,
            NRC_FAIL);
out:
  return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_get_addr(esp_eth_mac_t* mac, uint8_t* addr)
{
  nrc_err_t ret = NRC_SUCCESS;

  MAC_CHECK(addr, "can't set mac addr to null", out, NRC_FAIL);
  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);
  memcpy(addr, emac->addr, 6);
out:
  return ret;
}

static nrc_err_t enc28j60_configure_registers(emac_enc28j60_t *emac) 
{
    nrc_err_t ret = NRC_SUCCESS;

    // Verify chip ID first
    MAC_CHECK(enc28j60_verify_id(emac) == NRC_SUCCESS, "Chip verification failed after reset", out, NRC_FAIL);

    // Set up default registers and buffer pointers
    MAC_CHECK(enc28j60_setup_default(emac) == NRC_SUCCESS, "Default setup failed after reset", out, NRC_FAIL);

    // Clear multicast hash table
    MAC_CHECK(enc28j60_clear_multicast_table(emac) == NRC_SUCCESS, "Clear multicast table failed after reset", out, NRC_FAIL);

    // Clear power-saving modes
    MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_ECON2, ECON2_PWRSV | ECON2_VRPS) == NRC_SUCCESS, "Clear ECON2 power-saving bits failed", out, NRC_FAIL);

    // Re-enable interrupts for packet reception, TX errors, and global interrupt
    MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, 0xFF) == NRC_SUCCESS, "Clear EIR failed after reset", out, NRC_FAIL);
    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_PKTIE | EIE_INTIE | EIE_TXERIE) == NRC_SUCCESS, "Set EIE interrupts failed after reset", out, NRC_FAIL);

    // Set the MAC address again (assuming emac->addr contains a valid MAC)
    MAC_CHECK(enc28j60_set_mac_addr(emac) == NRC_SUCCESS, "Set MAC address failed after reset", out, NRC_FAIL);

    // Re-enable RX logic
    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_RXEN) == NRC_SUCCESS, "Enable RX logic failed after reset", out, NRC_FAIL);
    delay_us(100);

out:
    return ret;
}

static nrc_err_t enc28j60_clear_buferr(emac_enc28j60_t *emac) 
{
    uint8_t estat;
    int retries = 1000;
    static int no_resets = 0;

    E(TT_NET, "[%s]\n", __func__);

    no_resets++;

    if(no_resets > MAX_RETRY_COUNT) {
      E(TT_NET, "[%s] no_resets  %d, restart\n", __func__, no_resets);
      nrc_sw_reset();
    }

    // Disable RX
    enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_RXEN);
    // Reset RX FIFO
    enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_RXRST);
    delay_us(10);
    enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_RXRST);
    // Clear RX errors and packet interrupt flags
    enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_RXERIF | EIR_PKTIF);
    enc28j60_do_bitwise_clr(emac, ENC28J60_ESTAT, ESTAT_BUFER);
  
    // Realign RX buffer pointers (odd address is mandatory per errata #14)
    uint16_t rx_start = ENC28J60_BUF_RX_START & 0xFFFE;
    uint16_t rx_end = ENC28J60_BUF_RX_END | 0x0001;

    enc28j60_register_write(emac, ENC28J60_ERXSTL, rx_start & 0xFF);
    enc28j60_register_write(emac, ENC28J60_ERXSTH, rx_start >> 8);
    enc28j60_register_write(emac, ENC28J60_ERXNDL, rx_end & 0xFF);
    enc28j60_register_write(emac, ENC28J60_ERXNDH, rx_end >> 8);
    enc28j60_register_write(emac, ENC28J60_ERXRDPTL, rx_end & 0xFF);
    enc28j60_register_write(emac, ENC28J60_ERXRDPTH, rx_end >> 8);

    // Reinitialize software-side packet pointer state
    emac->next_packet_ptr = rx_start;

    // Clear all packets
    for (int i = 0; i < 256; ++i) 
    {
      uint8_t pk_counter = 0;
      enc28j60_register_read(emac, ENC28J60_EPKTCNT, &pk_counter);
      if(pk_counter <= 0)
        break;
      enc28j60_do_bitwise_set(gemac, ENC28J60_ECON2, ECON2_PKTDEC);
    }

    // Confirm chip clock readiness
    do {
        enc28j60_register_read(emac, ENC28J60_ESTAT, &estat);
        if (estat & ESTAT_CLKRDY)
            break;
        delay_us(10);
    } while (--retries);

    if (!retries) {
        E(TT_NET, "[%s] CLKRDY not set, recovery failed\n", __func__);
        return NRC_FAIL;
    }

    // Re-enable RX logic
    enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_RXEN);
    delay_us(100);
    
    // E(TT_NET, "[%s] RX restored\n", __func__);

    return NRC_SUCCESS;
}

static void enc28j60_reset_tx(emac_enc28j60_t *emac)
{
    static int no_resets = 0;

    E(TT_NET, "[%s]\n", __func__);

    no_resets++;
    
    if(no_resets > 3) {
      E(TT_NET, "[%s] no_resets  %d, restart\n", __func__, no_resets);
      enc28j60_clear_buferr(emac);
      no_resets = 0;
    }
    
    enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_TXRTS);        
    enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_TXRST);    // set transmit logic reset
    delay_us(10);
    enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, ECON1_TXRST);       // clear transmit error flag
    enc28j60_do_bitwise_clr(emac, ENC28J60_EIR,  EIR_TXERIF | EIR_TXIF);  // clear transmit error flags
    enc28j60_do_bitwise_clr(emac, ENC28J60_ESTAT, ESTAT_TXABRT | ESTAT_LATECOL);
    enc28j60_do_bitwise_clr(emac, ENC28J60_EIE, EIE_TXIE);         // clear transmit interrupt enable
    enc28j60_do_bitwise_clr(emac, ENC28J60_EIE, EIE_TXERIE);         // clear transmit interrupt flag
    enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_PKTIE | EIE_INTIE | EIE_TXERIE);
    enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_TXRTS);
    delay_us(100);
    enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXERIF);  // ensure no lingering error
    enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_TXIE | EIE_INTIE);
}

static void enc28j60_clear_single_pkt(emac_enc28j60_t *emac, enc28j60_rx_header_t *header) 
{
    uint8_t pktcnt, estat;

    enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_RXEN);

    enc28j60_register_read(emac, ENC28J60_ESTAT, &estat);
    if (estat & ESTAT_RXBUSY) {
      E(TT_NET, "[%s] ESTAT_RXBUSY %d\n", __func__, estat);
      // If a packet was mid-reception, it will be aborted, setting RXERIF&#8203;:contentReference[oaicite:16]{index=16}
      // We can wait a short time to let the abort flag set
      delay_us(50);
    }

    enc28j60_register_read(emac, ENC28J60_EPKTCNT, &pktcnt);

    E(TT_NET, "[%s] pktcnt %d\n", __func__, pktcnt);

    // Validate the next_packet_ptr from the header
    uint16_t next_packet_addr = (header->next_packet_high << 8) | header->next_packet_low;
    bool valid_ptr = (next_packet_addr >= ENC28J60_BUF_RX_START &&
                      next_packet_addr <= ENC28J60_BUF_RX_END &&
                     (next_packet_addr & 0x0001));

    if (!valid_ptr) {
      // If the pointer is out of range, fallback to full buffer reset for safety
      E(TT_NET, "[%s] Invalid next packet pointer 0x%04X – resetting RX buffer\n", __func__, next_packet_addr);
      enc28j60_clear_buferr(emac);
      return;
    }

    // Set ERXRDPT safely (must be odd address)
    uint16_t erxrdpt = enc28j60_next_ptr_align_odd(next_packet_addr, ENC28J60_BUF_RX_START, ENC28J60_BUF_RX_END);
    enc28j60_register_write(emac, ENC28J60_ERXRDPTL, erxrdpt & 0xFF);
    enc28j60_register_write(emac, ENC28J60_ERXRDPTH, (erxrdpt >> 8) & 0xFF);

    emac->next_packet_ptr = next_packet_addr;

    // Flush one corrupted packet
    if(pktcnt > 0)
      enc28j60_do_bitwise_set(emac, ENC28J60_ECON2, ECON2_PKTDEC);

    enc28j60_register_read(emac, ENC28J60_EPKTCNT, &pktcnt);

    E(TT_NET, "[%s] pktcnt %d after flush\n", __func__, pktcnt);

    // Re-enable RX logic
    enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_RXERIF | EIR_PKTIF);
    enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_RXEN);
    delay_us(50);
}

static inline nrc_err_t
emac_enc28j60_get_tsv(emac_enc28j60_t* emac, enc28j60_tsv_t* tsv)
{
  return enc28j60_read_packet( emac, emac->last_tsv_addr, (uint8_t*)tsv, ENC28J60_TSV_SIZE);
}

#ifdef ENABLE_ETHERNET_INTERRUPT
ATTR_NC __attribute__((optimize("O3"))) static void enc28j60_isr_handler(void *arg)
{
    emac_enc28j60_t *emac = (emac_enc28j60_t *)arg;
    BaseType_t high_task_wakeup = pdFALSE;

    /* notify enc28j60 task */
    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    
    if (high_task_wakeup != pdFALSE) {
        portYIELD_FROM_ISR(high_task_wakeup);
    }
}
#endif /* ENABLE_ETHERNET_INTERRUPT */

/**
 * @brief Main ENC28J60 Task. Mainly used for Rx processing. However, it also
 * handles other interrupts.
 *
 */
ATTR_NC __attribute__((optimize("O3"))) static void
emac_enc28j60_task(void* arg)
{
  emac_enc28j60_t* emac = (emac_enc28j60_t*)arg;
  uint8_t status = 0;
  uint8_t estat = 0;
  uint8_t mask = 0;
  uint8_t* buffer = NULL;
  uint32_t length = 0;
  int int_bit = 0;
  int retries = 0;
  int max_retries = 10;

  while (1) 
  {
  
    loop_start:
    
    MAC_CHECK_NO_RET(emac->run_task == true, "exit task", out);
    
#ifdef ENABLE_ETHERNET_INTERRUPT
    // block until some task notifies me or check the gpio by myself
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0) { // ...and no interrupt asserted
        nrc_gpio_inputb(emac->int_gpio_num, &int_bit);
        if (int_bit == 1) {
          continue;                          // -> just continue to check again
        }
    }
#else
    /* Polling Interrupt GPIO.*/
    vTaskDelay(pdMS_TO_TICKS(1000) / 40);
    nrc_gpio_inputb(emac->int_gpio_num, &int_bit);
    if (int_bit == 1) {
      continue;
    }
#endif

   MAC_CHECK_NO_RET(emac->run_task == true, "exit task", out);
  
    V(TT_NET, "[%s] interrupt asserted bit = %d\n", __func__, int_bit);
    // the host controller should clear the global enable bit for the interrupt pin before servicing the interrupt
    MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIE, EIE_INTIE) == NRC_SUCCESS, "clear EIE_INTIE failed", loop_start);
    MAC_CHECK_NO_RET(enc28j60_do_register_read(emac, true, ENC28J60_EIR, &status) == NRC_SUCCESS, "read EIR failed", loop_end);
    MAC_CHECK_NO_RET(enc28j60_do_register_read(emac, true, ENC28J60_EIE, &mask) == NRC_SUCCESS, "read EIE failed", loop_end);
    status &= mask;

#ifdef ENABLE_ETHERNET_INTERRUPT
    enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, 0xFF); 
    if (emac->interrupt_vector != -1) {
        system_irq_unmask(emac->interrupt_vector);
    }
#endif
    // When source of interrupt is unknown, try to check if there is packet
    // waiting (Errata #6 workaround)
    if (status == 0) {
      uint8_t pk_counter = 0;

      MAC_CHECK_NO_RET(enc28j60_register_read(emac, ENC28J60_EPKTCNT, &pk_counter) == NRC_SUCCESS, "read EPKTCNT failed", loop_end);

      if (pk_counter > 0) {
        status = EIR_PKTIF;
      } else {
        // E(TT_NET, "[%s] goto loop_end...\n", __func__);
        goto loop_end;
      }
    }

    MAC_CHECK_NO_RET(emac->run_task == true, "exit task", out);
    
    // packet received
    if (status & EIR_PKTIF) {
      do {
          /* read while there's packets remaining */
          int retry_alloc = MAX_RETRY_COUNT;
          do {
            length = ETH_MAX_PACKET_SIZE;
            buffer = MEM_MALLOC(length);

            if (!buffer) {
               E(TT_NET, "[%s] buffer allocation failed, waiting to recover retry_alloc %d\n", __func__, retry_alloc);
                delay_us(50);
            }
          } while (!buffer && (--retry_alloc) > 0);

          if (!buffer) {
            E(TT_NET, "[%s] Dropping RX packet due to malloc failure\n", __func__);
            // restart
            nrc_sw_reset();
            continue; // go check next packet
        }
        
        // E(TT_NET, "[%s] emac->parent.receive\n", __func__);
        if (emac->parent.receive(&emac->parent, buffer, &length) == NRC_SUCCESS) {
          /* pass the buffer to stack (e.g. TCP/IP layer) */
          if (length > 0) {
            // E(TT_NET, "[%s] emac->eth->stack_input length %d\n", __func__, length);
            emac->eth->stack_input(emac->eth, buffer, length);
          } else {
            E(TT_NET, "[%s] MEM_FREE #1\n", __func__);
            MEM_FREE(buffer);
          }
        } else {
          E(TT_NET, "[%s] MEM_FREE #2\n", __func__);
          MEM_FREE(buffer);
        }
      } while (emac->packets_remain);
    }

    MAC_CHECK_NO_RET(emac->run_task == true, "exit task", out);
    
    // transmit error
    if (status & EIR_TXERIF) {
      // Errata #12/#13 workaround - reset Tx state machine
      enc28j60_reset_tx(emac);
            
      // Errata #13 workaround (applicable only to B5 and B7 revisions)
      if (emac->revision == ENC28J60_REV_B5 ||
          emac->revision == ENC28J60_REV_B7) {
        __attribute__((aligned(4)))
        enc28j60_tsv_t tx_status; // SPI driver needs the rx buffer 4 byte align
        
        MAC_CHECK_NO_RET(emac_enc28j60_get_tsv(emac, &tx_status) == NRC_SUCCESS, "get Tx Status Vector failed", loop_end);
        
      // Try to retransmit when late collision is indicated
        if (tx_status.late_collision) {
          // Clear Tx Interrupt status Flag (it was set along with the error)
          MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXIF) == NRC_SUCCESS, "clear TXIF failed", loop_end);
          // Enable global interrupt flag and try to retransmit
          MAC_CHECK_NO_RET(enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_INTIE) == NRC_SUCCESS, "set INTIE failed", loop_end);
          MAC_CHECK_NO_RET(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_TXRTS) == NRC_SUCCESS, "set TXRTS failed", loop_end);
          
          delay_us(100);
          
          continue; // no need to handle Tx ready interrupt nor to enable
                    // global interrupt at this point
        }
      }
    }

  MAC_CHECK_NO_RET(emac->run_task == true, "exit task", out);

    // transmit ready
    if (status & EIR_TXIF) {
      MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXIF) == NRC_SUCCESS, "clear TXIF failed", loop_end);
      MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIE, EIE_TXIE) == NRC_SUCCESS, "clear TXIE failed", loop_end);

#ifdef ENABLE_ETHERNET_INTERRUPT
      // V(TT_NET, "[%s] xSemaphoreGive(emac->tx_ready_sem)\n", __func__);
      // xSemaphoreGive(emac->tx_ready_sem);
#endif
    }
  loop_end:
    MAC_CHECK_NO_RET(enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_INTIE) == NRC_SUCCESS, "clear INTIE failed", loop_start);
    MAC_CHECK_NO_RET(emac->run_task == true, "clear EIE_INTIE failed", out);
    // Note: Interrupt flag PKTIF is cleared when PKTDEC is set (in receive function)

  }
out:

  E(TT_NET, "[%s] exiting\n", __func__);

  vTaskDelete(NULL);
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_set_link(esp_eth_mac_t* mac, eth_link_t link)
{
  nrc_err_t ret = NRC_SUCCESS;

  switch (link) {
    case ETH_LINK_UP:
      MAC_CHECK(
        mac->start(mac) == NRC_SUCCESS, "enc28j60 start failed", out, NRC_FAIL);
      break;
    case ETH_LINK_DOWN:
      MAC_CHECK(
        mac->stop(mac) == NRC_SUCCESS, "enc28j60 stop failed", out, NRC_FAIL);
      break;
    default:
      MAC_CHECK(false, "unknown link status", out, NRC_FAIL);
      break;
  }
out:
  return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_set_speed(esp_eth_mac_t* mac, eth_speed_t speed)
{
  nrc_err_t ret = NRC_SUCCESS;

  switch (speed) {
    case ETH_SPEED_10M:
      I(TT_NET, "[%s] working in 10Mbps\n", __func__);
      break;
    default:
      MAC_CHECK(false, "100Mbps unsupported", out, NRC_FAIL);
      break;
  }
out:
  return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_set_duplex(esp_eth_mac_t* mac, eth_duplex_t duplex)
{
  nrc_err_t ret = NRC_SUCCESS;
  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);
  uint8_t mac3 = 0;

  MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MACON3, &mac3) == NRC_SUCCESS, "read MACON3 failed", out, NRC_FAIL);
  switch (duplex) {
    case ETH_DUPLEX_HALF:
      mac3 &= ~MACON3_FULDPX;
      MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MABBIPG, 0x12) == NRC_SUCCESS, "write MABBIPG failed", out, NRC_FAIL);
      enc28j60_register_write(emac, 0x00, (uint8_t) 0x0000); // PHCON1 register
      E(TT_NET, "[%s] working in half duplex\n", __func__);
      break;
    case ETH_DUPLEX_FULL:
      mac3 |= MACON3_FULDPX;
      MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MABBIPG, 0x15) == NRC_SUCCESS, "write MABBIPG failed", out, NRC_FAIL);
      enc28j60_register_write(emac, 0x00, (uint8_t) 0x0100); // PHCON1 register
      E(TT_NET, "[%s] working in full duplex\n", __func__);
      break;
    default:
      MAC_CHECK(false, "unknown duplex", out, NRC_FAIL);
      break;
  }
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MACON3, mac3) == NRC_SUCCESS, "write MACON3 failed", out, NRC_FAIL);
out:
  return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_set_promiscuous(esp_eth_mac_t* mac, bool enable)
{
  nrc_err_t ret = NRC_SUCCESS;
  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);

  if (enable) {
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXFCON, 0x00) == NRC_SUCCESS,
              "write ERXFCON failed",
              out,
              NRC_FAIL);
  }
out:
  return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t  
wait_for_transmit_completion(emac_enc28j60_t *emac) 
{
    nrc_err_t ret = NRC_SUCCESS;
    static int tx_fail_count = 0;
    uint8_t econ1, eir, estat;
    uint32_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(TX_WAIT_TIMEOUT_MS);
    
    do {
        MAC_CHECK(enc28j60_register_read(emac, ENC28J60_ECON1, &econ1) == NRC_SUCCESS, "ECON1 read failed", out, NRC_FAIL);

        if (!(econ1 & ECON1_TXRTS)) {
          tx_fail_count = 0;
          return NRC_SUCCESS; // TXRTS cleared => transmit complete
        }
        delay_us(10);
    } while (xTaskGetTickCount() < deadline);

    tx_fail_count++;
    
    // Timeout expired, TXRTS still set -> transmission stuck
    E(TT_NET, "TX timeout (TXRTS stuck), initiating recovery\n");
    
    MAC_CHECK(enc28j60_register_read(emac, ENC28J60_EIR, &eir) == NRC_SUCCESS, "read EIR failed", out, NRC_FAIL);  // read interrupt flags
    MAC_CHECK(enc28j60_register_read(emac, ENC28J60_ESTAT, &estat) == NRC_SUCCESS, "read EIR failed", out, NRC_FAIL);  // read interrupt flags
    
    if ((eir & EIR_TXERIF || estat & ESTAT_TXABRT || estat & ESTAT_LATECOL) || tx_fail_count > MAX_RETRY_COUNT) {
        enc28j60_reset_tx(emac);
    }
out:
    return ret;  // indicate that a recovery happened (packet was not sent)
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_transmit(esp_eth_mac_t* mac, uint8_t* buf, uint32_t length)
{
    nrc_err_t ret = NRC_SUCCESS;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    uint8_t econ1 = 0;

    MAC_CHECK(wait_for_transmit_completion(emac) == NRC_SUCCESS, "transmission busy", out, NRC_FAIL);

    // Reset TX registers before transmission
    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_TXRST) == NRC_SUCCESS, "set TXRST failed", out, NRC_FAIL);   // set transmit logic reset
    MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_TXRST) == NRC_SUCCESS, "clear TXRST failed", out, NRC_FAIL);   // clear transmit logic reset (TXRTS will clear)
    MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXERIF) == NRC_SUCCESS, "clear TXERIF failed", out, NRC_FAIL);      // clear transmit error flag
        
    /* Set the write pointer to start of transmit buffer area */
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_EWRPTL, ENC28J60_BUF_TX_START & 0xFF) == NRC_SUCCESS, "write EWRPTL failed", out, NRC_FAIL);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_EWRPTH, (ENC28J60_BUF_TX_START & 0xFF00) >> 8) == NRC_SUCCESS, "write EWRPTH failed", out, NRC_FAIL);

    /* Set the end pointer to correspond to the packet size given */
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ETXNDL, (ENC28J60_BUF_TX_START + length) & 0xFF) == NRC_SUCCESS, "write ETXNDL failed", out, NRC_FAIL);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ETXNDH, ((ENC28J60_BUF_TX_START + length) & 0xFF00) >> 8) == NRC_SUCCESS, "write ETXNDH failed", out, NRC_FAIL);

    /* copy data to tx memory */
    uint8_t per_pkt_control = 0; // MACON3 will be used to determine how the packet will be transmitted
    MAC_CHECK(enc28j60_do_memory_write(emac, &per_pkt_control, 1) == NRC_SUCCESS, "write packet control byte failed", out, NRC_FAIL);
    MAC_CHECK(enc28j60_do_memory_write(emac, buf, length) == NRC_SUCCESS, "buffer memory write failed", out, NRC_FAIL);
    emac->last_tsv_addr = ENC28J60_BUF_TX_START + length + 1;

    /* enable Tx Interrupt to indicate next Tx ready state */
    MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXIF) == NRC_SUCCESS, "set EIR_TXIF failed", out, NRC_FAIL);
    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_TXIE) == NRC_SUCCESS, "set EIE_TXIE failed", out, NRC_FAIL);

    /* issue tx polling command and record start time */
    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_TXRTS) == NRC_SUCCESS, "set ECON1.TXRTS failed", out, NRC_FAIL);
    emac->tx_start_ticks = xTaskGetTickCount();
    
out:
    return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_receive(esp_eth_mac_t* mac, uint8_t* buf, uint32_t* length)
{
#if defined(ETHERNET_SPI_DMA) && defined(SPI_DMA_CHAINED)
    for (int i = 0; i < RX_NODE_COUNT; i++) 
    {
        rx_node_t *node = &rx_nodes[i];

        if (node->state == NODE_PACKET_READY && node->aligned_packet_buffer) {
          // send to LwIP
          // MEM_FREE(node->aligned_packet_buffer);
          // node->aligned_packet_buffer = NULL;
          // node->state = NODE_FREE;
        }
    }

    return NULL; // No packets available
#else
    nrc_err_t ret = NRC_SUCCESS;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    static int error_recovery_count = 0;
    uint8_t pk_counter = 0;
    uint16_t rx_len = 0;
    uint32_t next_packet_addr = 0;
    enc28j60_rx_header_t header;

    // read packet header
    MAC_CHECK(enc28j60_read_packet(emac, emac->next_packet_ptr, (uint8_t *)&header, sizeof(header)) == NRC_SUCCESS,
              "read header failed", out, NRC_FAIL);

    V(TT_NET, "[%s] header.next_packet_low = 0x%x\n", __func__, header.next_packet_low);
    V(TT_NET, "[%s] header.next_packet_high = 0x%x\n", __func__, header.next_packet_high);
    V(TT_NET, "[%s] header.length_low = 0x%x\n", __func__, header.length_low);
    V(TT_NET, "[%s] header.length_high = 0x%x\n", __func__, header.length_high);
    V(TT_NET, "[%s] header.status_low = 0x%x\n", __func__, header.status_low);
    V(TT_NET, "[%s] header.status_high = 0x%x\n", __func__, header.status_high);

    // get packets' length, address
    rx_len = header.length_low + (header.length_high << 8);

    // Recovery logic to handle severe and repeated RX buffer corruption
    if (rx_len < MIN_ETH_FRAME_SIZE || rx_len > MAX_ETH_FRAME_SIZE) {
      if(error_recovery_count < MAX_RETRY_COUNT)
        enc28j60_clear_single_pkt(emac, &header);
      else
        enc28j60_clear_buferr(emac);

      if(error_recovery_count < MAX_RETRY_COUNT)
        error_recovery_count++;

      *length = 0;
       return NRC_FAIL;
    }
    else if(error_recovery_count > 0) {
        error_recovery_count--;
    }

    next_packet_addr = header.next_packet_low + (header.next_packet_high << 8);

    // read packet content
    MAC_CHECK(enc28j60_read_packet(emac, enc28j60_rx_packet_start(emac->next_packet_ptr, ENC28J60_RSV_SIZE), buf, rx_len) == NRC_SUCCESS, "read packet content failed", out, NRC_FAIL);

    // free receive buffer space
    uint32_t erxrdpt = enc28j60_next_ptr_align_odd(next_packet_addr, ENC28J60_BUF_RX_START, ENC28J60_BUF_RX_END);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXRDPTL, (erxrdpt & 0xFF)) == NRC_SUCCESS, "write ERXRDPTL failed", out, NRC_FAIL);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXRDPTH, (erxrdpt & 0xFF00) >> 8) == NRC_SUCCESS, "write ERXRDPTH failed", out, NRC_FAIL);
    emac->next_packet_ptr = next_packet_addr;

    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_ECON2, ECON2_PKTDEC) == NRC_SUCCESS, "set ECON2.PKTDEC failed", out, NRC_FAIL);
    MAC_CHECK(enc28j60_register_read(emac, ENC28J60_EPKTCNT, &pk_counter) == NRC_SUCCESS, "read EPKTCNT failed", out, NRC_FAIL);

    *length = rx_len - 4; // substract the CRC length
    emac->packets_remain = (pk_counter > 0);
out:
    return ret;
#endif
}

/**
 * @brief Get chip info
 */
eth_enc28j60_rev_t
emac_enc28j60_get_chip_info(esp_eth_mac_t* mac)
{
  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);

  return emac->revision;
}

#ifdef ETHERNET_DYNAMIC_BUFFERS
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t init_dynamic_buffers(void)
{
  spi_transfer_buf = MEM_MALLOC(SPI_TRANSFER_BUF_LEN);
  if (!spi_transfer_buf) {
    E(TT_NET, "[%s] spi transfer buffer alloc failed\n", __func__);
    return NRC_FAIL;
  }

  return NRC_SUCCESS;
}
#endif

#ifdef ENABLE_ETHERNET_INTERRUPT
static void enc28j60_intr_handler(int vector)
{
    int input;

    if (nrc_gpio_inputb(gemac->int_gpio_num, &input) < 0) {
      E(TT_NET, "[%s] call enc28j60_isr_handler, input %d\n", __func__, input);
      return;
    }

    if (!input) {
        // E(TT_NET, "[%s] call enc28j60_isr_handler\n", __func__);
        gemac->interrupt_vector = vector;
        system_irq_mask(gemac->interrupt_vector);
        enc28j60_isr_handler(gemac);
    } else {
        // E(TT_NET, "[%s] bogus interrupt\n", __func__);
    }
}
#endif

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_init(esp_eth_mac_t* mac)
{
  nrc_err_t ret = NRC_SUCCESS;
  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);
  esp_eth_mediator_t* eth = emac->eth;

  util_trace_set_log_level(TT_NET, 0);
  
  /* gpio used for enc28j60 reset */
  // enc28j60_gpio_config(GPIO_10, false);

  /* gpio used for enc28j60 interrupt */
  enc28j60_gpio_config(emac->int_gpio_num, true);

  nrc_gpio_trigger_config(INT_VECTOR0, TRIGGER_LEVEL, TRIGGER_LOW, true);

#ifdef ETHERNET_DYNAMIC_BUFFERS
  if(init_dynamic_buffers() != NRC_SUCCESS)
    return NRC_FAIL;
#endif

  MAC_CHECK(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL) == NRC_SUCCESS,
            "lowlevel init failed",
            out,
            NRC_FAIL);
  
  MAC_CHECK(enc28j60_reset(emac) == NRC_SUCCESS, "reset enc28j60 failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_configure_registers(emac) == NRC_SUCCESS, "Reconfiguration registers failed", out, NRC_FAIL);
  
#ifdef ENABLE_ETHERNET_INTERRUPT
  nrc_gpio_trigger_config(INT_VECTOR0, TRIGGER_LEVEL, TRIGGER_LOW, true);
  #define ENC28_INT_PRIO   0x05 
  system_irq_prio(INT_VECTOR0, ENC28_INT_PRIO);
  
  if (nrc_gpio_register_interrupt_handler(INT_VECTOR0, emac->int_gpio_num, enc28j60_intr_handler) == NRC_SUCCESS) {
    V(TT_NET, "[%s] interrupt handler installed\n", __func__);
  }
  emac->interrupt_vector = INT_VECTOR0;

#endif

  return NRC_SUCCESS;
out:
  // nrc_gpio_register_interrupt_handler(emac->int_gpio_num, (void *) NULL);
  eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
  return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_deinit(esp_eth_mac_t* mac)
{
  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);
  esp_eth_mediator_t* eth = emac->eth;
  mac->stop(mac);
  eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
  emac->run_task = false;
  
  return NRC_SUCCESS;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_del(esp_eth_mac_t* mac)
{
  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);
  vTaskDelete(emac->rx_task_hdl);
  vSemaphoreDelete(emac->reg_trans_lock);
  vSemaphoreDelete(emac->tx_ready_sem);
  free(emac);
  
#ifdef ETHERNET_DYNAMIC_BUFFERS
  MEM_FREE(spi_transfer_buf);
#endif

#ifdef ETHERNET_SPI_DMA
    if(rx_sem)
       vSemaphoreDelete(rx_sem);
    if(tx_sem)
       vSemaphoreDelete(tx_sem);

      dma_aligned_pbuf_pool_deinit();
#endif /* ETHERNET_SPI_DMA */

  return NRC_SUCCESS;
}

esp_eth_mac_t*
esp_eth_mac_new_enc28j60(spi_device_t *enc28j60_spi,
                         const eth_mac_config_t* mac_config, int gpio_int_pin)
{
  esp_eth_mac_t* ret = NULL;
  emac_enc28j60_t* emac = NULL;

  enc28j60_init_spi(enc28j60_spi);
#ifdef ETHERNET_SPI_DMA
  spi_dma_init(enc28j60_spi);
  nrc_dma_config_p2m(DMA_RX_CHANNEL, &spi_rx_peri, enc28j60_spi_rx_dma_isr, enc28j60_spi_rx_dma_err_isr);
  nrc_dma_config_m2p(DMA_TX_CHANNEL, &spi_tx_peri, enc28j60_spi_tx_dma_isr, enc28j60_spi_tx_dma_err_isr);
  _delay_ms(100);
#endif

  MAC_CHECK(mac_config, "can't set mac config to null", err, NULL);
  emac = calloc(1, sizeof(emac_enc28j60_t));
  gemac = emac;
  
  MAC_CHECK(emac, "calloc emac failed", err, NULL);
  /* enc28j60 driver is interrupt driven */
  emac->last_bank = 0xFF;
  emac->next_packet_ptr = ENC28J60_BUF_RX_START;
  /* bind methods and attributes */
  emac->sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
  emac->int_gpio_num = gpio_int_pin;
  MAC_CHECK(gpio_int_pin >= 0,
            "error interrupt gpio number",
            err,
            NULL);
  emac->spi_hdl = *enc28j60_spi;
  emac->parent.set_mediator = emac_enc28j60_set_mediator;
  emac->parent.init = emac_enc28j60_init;
  emac->parent.deinit = emac_enc28j60_deinit;
  emac->parent.start = emac_enc28j60_start;
  emac->parent.stop = emac_enc28j60_stop;
  emac->parent.del = emac_enc28j60_del;
  emac->parent.write_phy_reg = emac_enc28j60_write_phy_reg;
  emac->parent.read_phy_reg = emac_enc28j60_read_phy_reg;
  emac->parent.set_addr = emac_enc28j60_set_addr;
  emac->parent.get_addr = emac_enc28j60_get_addr;
  emac->parent.set_speed = emac_enc28j60_set_speed;
  emac->parent.set_duplex = emac_enc28j60_set_duplex;
  emac->parent.set_link = emac_enc28j60_set_link;
  emac->parent.set_promiscuous = emac_enc28j60_set_promiscuous;
  emac->parent.transmit = emac_enc28j60_transmit;
  emac->parent.receive = emac_enc28j60_receive;
  emac->mac_config = mac_config;
  emac->run_task = true;
  emac->tx_start_ticks = 0;
  emac->spi_lock = 0;
  
  /* create mutex */
  emac->reg_trans_lock = xSemaphoreCreateMutex();
  MAC_CHECK(emac->reg_trans_lock, "create register transaction lock failed", err, NULL);
  emac->tx_ready_sem = xSemaphoreCreateBinary();
  MAC_CHECK(emac->tx_ready_sem, "create pkt transmit ready semaphore failed", err, NULL);
  xSemaphoreGive(emac->tx_ready_sem); // ensures the first transmit is
                                      // performed without waiting
#ifdef ETHERNET_SPI_DMA
  rx_sem = xSemaphoreCreateBinary();
  MAC_CHECK(rx_sem, "create rx_sem semaphore failed", err, NULL);
  tx_sem = xSemaphoreCreateBinary();
  MAC_CHECK(rx_sem, "create tx_sem semaphore failed", err, NULL);
  dma_aligned_pbuf_pool_init();
#endif /* ETHERNET_SPI_DMA */
  /* create enc28j60 task */
  BaseType_t core_num = 0; // tskNO_AFFINITY;
  if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
    core_num = 0; // cpu_hal_get_core_id();
  }

  BaseType_t xReturned = xTaskCreate(emac_enc28j60_task,
                                     "enc28j60_tsk",
                                     mac_config->rx_task_stack_size,
                                     emac,
                                     NRC_TASK_PRIORITY - 1,
                                     &emac->rx_task_hdl);
  MAC_CHECK(xReturned == pdPASS, "create enc28j60 task failed", err, NULL);

  return &(emac->parent);
err:
  if (emac) {
    if (emac->rx_task_hdl) {
      vTaskDelete(emac->rx_task_hdl);
    }
    if (emac->reg_trans_lock) {
      vSemaphoreDelete(emac->reg_trans_lock);
    }
    if (emac->tx_ready_sem) {
      vSemaphoreDelete(emac->tx_ready_sem);
    }
#ifdef ETHERNET_SPI_DMA
    if(rx_sem)
       vSemaphoreDelete(rx_sem);
    if(tx_sem)
       vSemaphoreDelete(tx_sem);
#endif /* ETHERNET_SPI_DMA */
    MEM_FREE(emac);
  }
  return ret;
}
