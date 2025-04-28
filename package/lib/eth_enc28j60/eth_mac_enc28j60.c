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

#define DUMP_HEX(title, buf, len)                                \
  do {                                                           \
    printf("[DUMP] %s (%d bytes):\n", title, (int)(len));        \
    for (int i = 0; i < (int)(len); i++) {                       \
      printf("%02X ", ((uint8_t *)(buf))[i]);                    \
      if ((i + 1) % 16 == 0) printf("\n");                       \
    }                                                            \
    if ((len) % 16 != 0) printf("\n");                           \
  } while (0)

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

// #define ETHERNET_SPI_DMA_CHAINED 1

#ifdef ETHERNET_SPI_DMA_CHAINED

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

// ISR chain events
typedef enum {
    DMA_STATE_IDLE,
    DMA_STATE_PAYLOAD_IN_PROGRESS,
} dma_transfer_state_t;

static volatile dma_transfer_state_t dma_state = DMA_STATE_IDLE;

static volatile int dma_chain_depth = 0;

#define SPI_IDLE_POLL_CYCLES 100

typedef enum {
    SPI_OWNER_NONE = 0,
    SPI_OWNER_DMA_ISR,
    SPI_OWNER_TASK,
} spi_owner_t;

static volatile spi_owner_t spi_owner = SPI_OWNER_NONE;

typedef enum {
    NODE_FREE = 0,
    NODE_HEADER_READY,
    NODE_PACKET_READY,
} rx_node_state_t;

#define HEADER_SIZE    6
typedef struct {
    rx_node_state_t state;
    uint8_t __attribute__((aligned(4))) header[HEADER_SIZE];  // Static aligned header buffer
    uint8_t *aligned_packet_buffer;                           // Dynamically allocated aligned buffer for DMA
    uint16_t packet_length;
} rx_node_t;

#define RX_NODE_COUNT 8

static rx_node_t rx_nodes[RX_NODE_COUNT];
static rx_node_t *current_dma_node = NULL;

// task read pkt queue
static QueueHandle_t rx_ready_queue;

#endif /*#ifdef ETHERNET_SPI_DMA_CHAINED */
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

#define ENC28J60_DEBUG 1

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
  SemaphoreHandle_t spi_lock;
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

ATTR_NC __attribute__((optimize("O3"))) static inline bool
enc28j60_spi_lock(emac_enc28j60_t* emac)
{
  BaseType_t is_locked = pdFALSE;

#ifdef ETHERNET_SPI_DMA_CHAINED
#ifdef ENC28J60_DEBUG
  switch(spi_owner)
  {
    case SPI_OWNER_TASK:
      nrc_usr_print("[%s] SPI_OWNER_TASK\n",__func__);
      break;
    case SPI_OWNER_DMA_ISR:
      nrc_usr_print("[%s] SPI_OWNER_DMA_ISR\n",__func__);
      break;
    case SPI_OWNER_NONE:
    default:
      nrc_usr_print("[%s] SPI_OWNER_NONE\n",__func__);
      break;
  }
#endif
  while (spi_owner != SPI_OWNER_NONE) 
  {
    E(TT_NET, "waiting to take SPI from ISR\n");
    delay_us(10);  // Wait for SPI until ISR finished
  }
  spi_owner = SPI_OWNER_TASK;
  E(TT_NET, "SPI_OWNER_TASK\n");
#endif

  is_locked = xSemaphoreTake(emac->spi_lock, pdMS_TO_TICKS(ENC28J60_SPI_LOCK_TIMEOUT_MS));
  
  if(!is_locked)
    E(TT_NET, "spi_lock timed out, not locked\n");
  
  return (is_locked == pdTRUE);
}

ATTR_NC __attribute__((optimize("O3"))) static inline bool
enc28j60_spi_unlock(emac_enc28j60_t* emac)
{
#ifdef ETHERNET_SPI_DMA_CHAINED
  if (spi_owner == SPI_OWNER_TASK) 
    spi_owner = SPI_OWNER_NONE;
#endif
  return xSemaphoreGive(emac->spi_lock) == pdTRUE;
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

ATTR_NC __attribute__((optimize("O3"))) static inline void* unalign_aligned_buffer(uint8_t *aligned_buffer)
{
  return ((void**)aligned_buffer)[-1];
}

ATTR_NC __attribute__((optimize("O3"))) static inline void* aligned_malloc(size_t size, size_t alignment)
{
    uintptr_t raw;
    int retry_alloc = MAX_RETRY_COUNT;

    do {
          raw = (uintptr_t)nrc_mem_malloc(size + alignment - 1 + sizeof(void*));

          if (!raw) {
            // E(TT_NET, "[%s] buffer allocation failed, waiting to recover retry_alloc %d\n", __func__, retry_alloc);
            delay_us(100);
          }
    } while (!raw && (--retry_alloc) > 0);

    if (!raw) {
      E(TT_NET, "[%s] buffer allocation failed\n", __func__);
      return NULL;
    }

    uintptr_t aligned = (raw + sizeof(void*) + alignment - 1) & ~(alignment - 1);
    ((void**)aligned)[-1] = (void*)raw;  // store original pointer
    return (void*)aligned;
}

ATTR_NC __attribute__((optimize("O3"))) static inline void aligned_free(void* ptr)
{
    nrc_usr_print("[%s]\n",__func__);

    if (ptr)
        nrc_mem_free(((void**)ptr)[-1]);
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

#ifdef ETHERNET_SPI_DMA_CHAINED

static nrc_err_t enc28j60_register_read(emac_enc28j60_t* emac, uint16_t reg_addr, uint8_t* value);
static nrc_err_t enc28j60_register_write(emac_enc28j60_t* emac, uint16_t reg_addr, uint8_t value);
static nrc_err_t enc28j60_clear_buferr(emac_enc28j60_t *emac);
static nrc_err_t enc28j60_do_bitwise_set(emac_enc28j60_t* emac, uint8_t reg_addr, uint8_t mask);
static int spi_read_buf(spi_device_t* spi, int len, u8* data);
static void enc28j60_spi_rx_dma_err_isr (int channel);
static void enc28j60_spi_rx_dma_isr (int channel);

#define SPI_BSY_BIT    (1 << 4)
static inline bool spi_is_idle(void)
{
    return (RegSSP_SR(SSP0_BASE_ADDR) & SPI_BSY_BIT) == 0;
}

static void defer_to_task(void)
{
    BaseType_t high_task_wakeup = pdFALSE;

    vTaskNotifyGiveFromISR(gemac->rx_task_hdl, &high_task_wakeup);

    if (high_task_wakeup != pdFALSE) {
      portYIELD_FROM_ISR(high_task_wakeup);
    }
}

static void isr_return_spi_to_idle(void)
{
  if(spi_owner == SPI_OWNER_DMA_ISR)
    spi_owner = SPI_OWNER_NONE;
}

static bool isr_grab_spi_from_task(void)
{
  nrc_usr_print("[%s]\n",__func__);

  switch(spi_owner)
  {
    case SPI_OWNER_TASK:
      nrc_usr_print("[%s] SPI_OWNER_TASK\n",__func__);
      break;
    case SPI_OWNER_DMA_ISR:
      nrc_usr_print("[%s] SPI_OWNER_DMA_ISR\n",__func__);
      break;
    case SPI_OWNER_NONE:
    default:
      nrc_usr_print("[%s] SPI_OWNER_NONE\n",__func__);
      break;
  }
  
  if (spi_owner == SPI_OWNER_NONE) {
    spi_owner = SPI_OWNER_DMA_ISR;
    nrc_usr_print("[%s] SPI_OWNER_DMA_ISR\n",__func__);
    return true;
  }

  if (spi_owner == SPI_OWNER_TASK) {
    int poll_count = SPI_IDLE_POLL_CYCLES;
    while (poll_count-- > 0)
    {
      if (spi_is_idle()) {
        spi_owner = SPI_OWNER_DMA_ISR;
        nrc_usr_print("[%s] SPI_OWNER_DMA_ISR\n",__func__);
        return true;
      }
      delay_us(5);
    }
    nrc_usr_print("[%s] timeout, failed to become spi_owner\n",__func__);
    return false;
  }
  
  nrc_usr_print("[%s] end of function, failed to become spi_owner\n",__func__);
  return false;
}

static nrc_err_t dma_init_nodes(void)
{
  nrc_err_t ret = NRC_SUCCESS;

  nrc_usr_print("[%s]\n",__func__);

  for (int i = 0; i < RX_NODE_COUNT; i++)
  {
    rx_nodes[i].aligned_packet_buffer = aligned_malloc(MAX_ETH_FRAME_SIZE, 4);
    if(!rx_nodes[i].aligned_packet_buffer[-1]) {
      ret = NRC_FAIL;
      break;
    }
    rx_nodes[i].packet_length == 0;
    rx_nodes[i].state == NODE_FREE;
  }

  return ret;
}

rx_node_t* dma_find_free_node(void)
{
  for (int i = 0; i < RX_NODE_COUNT; i++)
  {
    if (rx_nodes[i].state == NODE_FREE) {
      rx_nodes[i].state = NODE_HEADER_READY;
      return &rx_nodes[i];
    }
  }

  return NULL;
}

void enc28j60_isr_clr_bit(uint16_t reg, uint8_t val)
{
  // spi_transfer_buf[0] = ENC28J60_BIT_FIELD_CLR | (reg & ADDR_MASK);
  // spi_transfer_buf[1] = val;
  // nrc_spi_start_xfer(&gemac->spi_hdl);
  // nrc_spi_xfer(&gemac->spi_hdl, spi_transfer_buf, spi_transfer_buf, 2);
  // nrc_spi_stop_xfer(&gemac->spi_hdl);

  nrc_spi_writebyte_value(&gemac->spi_hdl, ENC28J60_BIT_FIELD_CLR | (reg & ADDR_MASK), val);
}

void enc28j60_isr_set_bit(uint16_t reg, uint8_t val)
{
  // spi_transfer_buf[0] = ENC28J60_BIT_FIELD_SET | (reg & ADDR_MASK);
  // spi_transfer_buf[1] = val;
  // nrc_spi_start_xfer(&gemac->spi_hdl);
  // nrc_spi_xfer(&gemac->spi_hdl, spi_transfer_buf, spi_transfer_buf, 2);
  // nrc_spi_stop_xfer(&gemac->spi_hdl);
  nrc_spi_writebyte_value(&gemac->spi_hdl, ENC28J60_BIT_FIELD_SET | (reg & ADDR_MASK), val);
}

void enc28j60_isr_set_bank(uint16_t reg)
{
  nrc_err_t ret;
  uint8_t bank = (reg & 0xF00) >> 8;

  nrc_usr_print("[%s] reg 0x%X bank %d gemac->last_bank %d\n",__func__, reg, bank, gemac->last_bank);
  
  if(bank == gemac->last_bank) return;

  enc28j60_isr_clr_bit(ENC28J60_ECON1, 0x03);
  enc28j60_isr_set_bit(ENC28J60_ECON1, bank & 0x03);

  gemac->last_bank = bank;
  
  nrc_usr_print("[%s] bank %d gemac->last_bank %d\n",__func__, bank, gemac->last_bank);
}

void enc28j60_isr_write_reg(uint16_t reg, uint8_t val)
{
  nrc_err_t ret;

  nrc_usr_print("[%s] reg %d val 0x%X\n",__func__, reg, val);
  
  enc28j60_isr_set_bank(reg);

  // spi_transfer_buf[0] = ENC28J60_WRITE_CTRL_REG | (reg & ADDR_MASK);
  // spi_transfer_buf[1] = val;
  // nrc_spi_start_xfer(&gemac->spi_hdl);
  // ret = nrc_spi_xfer(&gemac->spi_hdl, spi_transfer_buf, spi_transfer_buf, 2);
  nrc_spi_writebyte_value(&gemac->spi_hdl, (reg & ADDR_MASK) | ENC28J60_WRITE_CTRL_REG, val);
  // nrc_spi_stop_xfer(&gemac->spi_hdl);
}

uint16_t enc28j60_isr_read_reg(uint16_t reg)
{
  uint16_t is_eth_reg = !(reg & 0xF000);
  uint8_t val[2];
  
  enc28j60_isr_set_bank(reg);

  nrc_usr_print("[%s] is_eth_reg %d reg 0x%X\n",__func__, is_eth_reg, reg);
  
  nrc_spi_writebyte_value(&gemac->spi_hdl, (reg & ADDR_MASK) | ENC28J60_READ_CTRL_REG, 0);
  nrc_spi_readbyte_value(&gemac->spi_hdl, (reg & ADDR_MASK) | ENC28J60_READ_CTRL_REG, &val[0]);
  nrc_usr_print("[%s] read value #1 %d\n",__func__, val[0]);
  // if(is_eth_reg) {
  //   nrc_spi_readbyte_value(&gemac->spi_hdl, (reg & ADDR_MASK) | ENC28J60_READ_CTRL_REG, &val[1]);
  //   nrc_usr_print("[%s] read value #2 %d\n",__func__, val[1]);
  // }

  nrc_usr_print("[%s] returning value %d\n",__func__, val[0]);
 
  return val[0];
}

void enc28j60_isr_read_mem_nodma(uint8_t* buf, int len)
{
  uint8_t cmd = ENC28J60_READ_BUF_MEM;

  // nrc_spi_start_xfer(&gemac->spi_hdl);
  // for (int i = 0; i < len + 1; ++i) 
  // {
  //   uint8_t tx = (i == 0 ? cmd : 0x00);
  //   uint8_t rx = 0x00;
  //   nrc_spi_xfer(&gemac->spi_hdl, &tx, &rx, 1);  // transfer 1 byte
  //   buf[i] = rx;
  // }
  // nrc_spi_stop_xfer(&gemac->spi_hdl);
  nrc_spi_read_values(&gemac->spi_hdl, cmd, buf, len);
}

void enc28j60_isr_read_header(uint8_t* buf)
{
  uint8_t erdptl, erdpth;
  uint16_t rdpt;
  
  erdptl = enc28j60_isr_read_reg(ENC28J60_ERDPTL);
  erdpth = enc28j60_isr_read_reg(ENC28J60_ERDPTH);
  rdpt = ((uint16_t)erdpth << 8) | erdptl;
  nrc_usr_print("[%s]  erdptl 0x%X erdpth 0x%X rdpt 0x%X next_packet_ptr 0x%X\n",__func__, erdptl, erdpth, rdpt, gemac->next_packet_ptr);
  enc28j60_isr_write_reg(ENC28J60_ERDPTL, erdptl);
  enc28j60_isr_write_reg(ENC28J60_ERDPTH, erdpth);

  enc28j60_isr_read_mem_nodma(buf, sizeof(enc28j60_rx_header_t));
  
  DUMP_HEX("header", buf, sizeof(enc28j60_rx_header_t));
}

void spi_dma_isr_read(uint8_t *addr, uint8_t *data, uint32_t size)
{
  int err = 1;

  nrc_usr_print("[%s]\n",__func__);
  
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
  // nrc_dma_stop(DMA_TX_CHANNEL);
  // nrc_dma_stop(DMA_RX_CHANNEL);
  // nrc_ssp_dma(gemac->spi_hdl.controller, SSP_DMA_TX | SSP_DMA_RX, false);
  // spi_dma_stop_xfer(&gemac->spi_hdl);
}

void isr_process_header(void)
{
  enc28j60_rx_header_t* hdr;
  rx_node_t *node = dma_find_free_node();
  uint8_t cmd = ENC28J60_READ_BUF_MEM;
  uint16_t next_packet_addr;

  nrc_usr_print("[%s]\n",__func__);
  
  if(dma_state != DMA_STATE_IDLE){
    nrc_usr_print("[%s] dma_state not idle, returning\n",__func__);
    defer_to_task();
    return;
  }

  if(!node) {
    nrc_usr_print("[%s] dma_find_free_node() failed\n",__func__);
    event_flags |= EVT_RX_DISCARD_PKT;
    dma_chain_depth = 0;
    defer_to_task();
    return;
  }

  if(!isr_grab_spi_from_task()){
    nrc_usr_print("[%s] grab_spi_from_task() failed\n",__func__);
    isr_return_spi_to_idle();
    dma_chain_depth = 0;
    event_flags |= EVT_RX_DISCARD_PKT;
    defer_to_task();
    return;
  }

  if (dma_chain_depth >= DMA_CHAIN_LIMIT) {
    nrc_usr_print("[%s] dma_chain_depth %d failed\n",__func__, dma_chain_depth);
    event_flags |= EVT_RX_DISCARD_PKT;
    isr_return_spi_to_idle();
    dma_chain_depth = 0;
    defer_to_task();
    return;
  }
  
  dma_chain_depth++;

  // read header into spi xfer buffer
  enc28j60_isr_read_header(node->header);
  hdr = (enc28j60_rx_header_t*)node->header;
  DUMP_HEX("raw header", node->header, sizeof(node->header));
  // next_packet_addr = (hdr->next_packet_high << 8) | hdr->next_packet_low;
  node->packet_length = hdr->length_low + (hdr->length_high << 8);
  next_packet_addr = enc28j60_rx_packet_start((hdr->next_packet_high << 8) | hdr->next_packet_low, ENC28J60_RSV_SIZE);
  
  nrc_usr_print("[%s] next_packet_addr 0x%X node->packet_length %d\n",__func__, next_packet_addr, node->packet_length);

  // check validity
  if (node->packet_length < MIN_ETH_FRAME_SIZE || node->packet_length > MAX_ETH_FRAME_SIZE) {
    nrc_usr_print("[%s] invalid pkt, exiting\n", __func__);
    event_flags |= EVT_RX_DISCARD_PKT;
    isr_return_spi_to_idle();
    dma_chain_depth = 0;
    defer_to_task();
    return;
  }

  current_dma_node = node;

  nrc_usr_print("[%s] skipping spi_dma_isr_read()\n",__func__);
  spi_dma_isr_read(&cmd, node->aligned_packet_buffer, node->packet_length + 1);

  dma_state = DMA_STATE_PAYLOAD_IN_PROGRESS;
  spi_owner = SPI_OWNER_NONE;
}

bool isr_packets_pending(void)
{
  uint8_t pktcnt = enc28j60_isr_read_reg(ENC28J60_EPKTCNT);
  nrc_usr_print("[%s] pktcnt %d\n",__func__, pktcnt);
  return (pktcnt > 0);
}

void enc28j60_dma_tx_complete_isr() 
{
  nrc_usr_print("[%s]\n",__func__);
  event_flags = EVT_TX_DONE;
}

// finish packet processing, check for more packets, read header, DMA read packet
void enc28j60_dma_rx_complete_isr() 
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  nrc_usr_print("[%s]\n",__func__);

  if (current_dma_node) {
    // Mark current node ready, decrement packet counter
    current_dma_node->state = NODE_PACKET_READY;
    xQueueSendFromISR(rx_ready_queue, &current_dma_node, &xHigherPriorityTaskWoken);
    enc28j60_isr_set_bit(ENC28J60_ECON2, ECON2_PKTDEC);
    current_dma_node = NULL;
  }

   if (!isr_grab_spi_from_task()) {
        defer_to_task();
        return;
  }
  
  if(isr_packets_pending()) {
    isr_process_header();
  }
  else
      dma_state = DMA_STATE_IDLE;

  defer_to_task();

  event_flags = EVT_RX_PAYLOAD_DONE;
}

#endif /* #ifdef ETHERNET_SPI_DMA_CHAINED */

// Called from ISR (RX/TX DMA completion)
static void dma_done_isr(dma_dir_t dir)
{
    BaseType_t high_task_wakeup = pdFALSE;

#ifdef ETHERNET_SPI_DMA_CHAINED
    E(TT_NET, "[%s] %s\n", __func__);
         
    if (dir == DMA_DIR_RX || dir == DMA_DIR_TX) {
      	nrc_dma_stop(DMA_TX_CHANNEL);
	nrc_dma_stop(DMA_RX_CHANNEL);
	nrc_ssp_dma(gemac->spi_hdl.controller, SSP_DMA_TX | SSP_DMA_RX, false);
	spi_dma_stop_xfer(&gemac->spi_hdl);
    }

    // DMA is now idle
    spi_dma_busy = false;

    E(TT_NET, "[%s] spi_dma_busy == %s dir == %s event_flags 0x%X\n", __func__, (spi_dma_busy == true) ? "true" : "false", (dir == DMA_DIR_TX) ? "TX" : "RX", event_flags);

    if (dir == DMA_DIR_TX) {
        enc28j60_dma_tx_complete_isr();
    } 
    else if (dir == DMA_DIR_RX) {
        enc28j60_dma_rx_complete_isr();
    }
#endif /* ETHERNET_SPI_DMA_CHAINED */
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

        // nrc_usr_print("[%s]\n",__func__);
        // nrc_usr_print("[%s] TX prepare_dma_desc.\n",__func__);
	init_dma_desc(true, (uint32_t) addr, size);
	/* Disable TX interrupt while reading data from SPI */
	nrc_dma_desc_set_inttc(&tx_desc, false);
	//	nrc_usr_print("[%s] RX prepare_dma_desc.\n",__func__);
	init_dma_desc(false, (uint32_t) data, size);

	/* prepare DMA RX channel to start receiving data */
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

        // nrc_usr_print("[%s]\n",__func__);
	//	nrc_usr_print("[%s] TX prepare_dma_desc.\n",__func__);
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

  enc28j60_spi_lock(gemac);
#ifdef ETHERNET_SPI_DMA
  // DMA is only for large transfers
  // if(len > 8) {
    // Use DMA to send RBM command and read len+1 bytes (includes dummy)
    enc28j60_spi_dma_read(&cmd, spi_transfer_buf, len + 1);
    // The first byte in spi_transfer_buf is dummy (from sending cmd), actual data starts at index 1
    memcpy(data, &spi_transfer_buf[1], len);
  // } else {
  //   E(TT_NET, "using slow read for %d bytes instead of DMA\n", len);
  //   nrc_spi_read_values(spi, cmd, spi_transfer_buf, len);
  //   memcpy(data, spi_transfer_buf, len);
  // }
#else
    // Fallback: byte-by-byte SPI transfer (mitigates timing issues if DMA is unavailable)
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
#endif
    enc28j60_spi_unlock(gemac);
    return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static int
spi_write_buf(spi_device_t* spi, int len, const u8* data)
{
  int ret = NRC_SUCCESS;
  
  if (len > SPI_TRANSFER_BUF_LEN - 1 || len <= 0) {
    ret = NRC_FAIL;
  }
    spi_transfer_buf[0] = ENC28J60_WRITE_BUF_MEM;  // 0x7A command for write buffer
    memcpy(&spi_transfer_buf[1], data, len);
    enc28j60_spi_lock(gemac);
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

      if (trans->length / 8 == sizeof(uint8_t)) {
        if (nrc_spi_readbyte_value(&spi_hdl,
                                   (trans->addr & ADDR_MASK) |
                                     ENC28J60_READ_CTRL_REG,
                                   &trans->rx_data[0]) == NRC_SUCCESS) {
          retval = NRC_SUCCESS;
        }
      } else if (trans->length / 8 == 2 * sizeof(uint8_t)) {

        if (nrc_spi_readbyte_value(&spi_hdl,
                                   (trans->addr & ADDR_MASK) |
                                     ENC28J60_READ_CTRL_REG,
                                   &trans->rx_data[0]) == NRC_SUCCESS) {
          retval = NRC_SUCCESS;
        }
        if (nrc_spi_readbyte_value(&spi_hdl,
                                   (trans->addr & ADDR_MASK) |
                                     ENC28J60_READ_CTRL_REG,
                                   &trans->rx_data[1]) == NRC_SUCCESS) {
          retval = NRC_SUCCESS;
        }
      } else if (trans->length / 8 > 2 * sizeof(uint8_t)) {
        if (nrc_spi_read_values(&spi_hdl,
                                (trans->addr & ADDR_MASK) |
                                  ENC28J60_READ_CTRL_REG,
                                trans->rx_data,
                                trans->length / 8) == NRC_SUCCESS) {
          retval = NRC_SUCCESS;
        }
      }
      enc28j60_spi_unlock(gemac);
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

static void enc28j60_reset_tx(emac_enc28j60_t *emac)
{
    static int no_resets = 0;

    E(TT_NET, "[%s]\n", __func__);

    no_resets++;
    
    if(no_resets > 3) {
      E(TT_NET, "[%s] no_resets  %d, restart\n", __func__, no_resets);
      nrc_sw_reset();
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

      // Temporarily reset RX logic
      // enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_RXRST);
      // delay_us(100);
      // enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_RXRST);
      // enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_RXERIF);
      // delay_us(100);

      // Validate the next_packet_ptr from the header
      uint16_t next_packet_addr = (header->next_packet_high << 8) | header->next_packet_low;
      bool valid_ptr = (next_packet_addr >= ENC28J60_BUF_RX_START &&
                        next_packet_addr <= ENC28J60_BUF_RX_END &&
                       (next_packet_addr & 0x0001));

      if (!valid_ptr) {
        // If the pointer is out of range, fallback to full buffer reset for safety
        E(TT_NET, "[%s] Invalid next packet pointer 0x%04X – resetting RX buffer\n", __func__, next_packet_addr);
        enc28j60_clear_buferr(emac);  // perform full RX FIFO reset (see below)
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
#ifndef ETHERNET_SPI_DMA_CHAINED
ATTR_NC __attribute__((optimize("O3"))) static void enc28j60_isr_handler(void *arg)
{
    emac_enc28j60_t *emac = (emac_enc28j60_t *)arg;
    BaseType_t high_task_wakeup = pdFALSE;

    // E(TT_NET, "[%s]\n", __func__);

    /* notify enc28j60 task */
    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    if (high_task_wakeup != pdFALSE) {
        portYIELD_FROM_ISR(high_task_wakeup);
    }
}
#else /* ETHERNET_SPI_DMA_CHAINED */
ATTR_NC __attribute__((optimize("O3"))) static void enc28j60_isr_handler(void *arg)
{
  emac_enc28j60_t *emac = (emac_enc28j60_t *)arg;
  BaseType_t high_task_wakeup = pdFALSE;

  E(TT_NET, "[%s] ETHERNET_SPI_DMA_CHAINED\n", __func__);
    
  // check if there are packets to read
  // read header, check validity, if valid start DMA packet read
  // packet read will finish in DMA done ISR
  if(isr_packets_pending()) {
    dma_chain_depth = 0;
    isr_process_header();
  }

  defer_to_task();
}
#endif /* ETHERNET_SPI_DMA_CHAINED */
#endif /* ENABLE_ETHERNET_INTERRUPT */

#ifdef ETHERNET_SPI_DMA_CHAINED
static void emac_enc28j60_task(void *arg)
{
  emac_enc28j60_t* emac = (emac_enc28j60_t*)arg;
  int int_bit = 0;
  uint8_t flags = 0;
  rx_node_t *node;
  uint8_t pktcnt = 0;

  while (1)
  {
    // E(TT_NET, "[%s] loop top\n", __func__);

    MAC_CHECK_NO_RET(emac->run_task == true, "exit task", out);

    // block until some task notifies me
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0) { // ...and no interrupt asserted
      // nrc_gpio_inputb(emac->int_gpio_num, &int_bit);
      // if (int_bit == 1) {
      //   E(TT_NET, "[%s] no interrupt from gpio\n", __func__);
      delay_us(100);
      continue;                          // -> just continue to check again
      // }
    }

#ifdef ENABLE_ETHERNET_INTERRUPT
    if (emac->interrupt_vector != -1) {
        system_irq_unmask(emac->interrupt_vector);
    }
#endif

    flags = event_flags;
    
    if (flags & EVT_TX_ERR) {
      E(TT_NET, "[%s] EVT_TX_ERR\n", __func__);
      enc28j60_reset_tx(emac);
      flags &= ~EVT_TX_ERR;
    }
    
    if (flags & EVT_RX_ERR) {
      E(TT_NET, "[%s] EVT_RX_ERR\n", __func__);
      enc28j60_clear_buferr(emac);
      flags &= ~EVT_RX_ERR;
    }

    if (flags & EVT_RX_DISCARD_PKT)
    {
      // NOTE: check emac->next_packet_ptr for sanity before passing
      E(TT_NET, "[%s] EVT_RX_DISCARD_PKT\n", __func__);
      // enc28j60_clear_single_pkt(emac, (enc28j60_rx_header_t *)emac->next_packet_ptr);
      flags &= ~EVT_RX_DISCARD_PKT;
    }

    enc28j60_register_read(emac, ENC28J60_EPKTCNT, &pktcnt);
    E(TT_NET, "[%s] pktcnt %d\n", __func__, pktcnt);
    
    while (xQueueReceive(rx_ready_queue, &node, 0) == pdPASS)
    {
      int retry_alloc = MAX_RETRY_COUNT;
      uint8_t *buffer = NULL;

      E(TT_NET, "[%s] process queue\n", __func__);
            
      // log state if not correct
      if (node->state != NODE_PACKET_READY)
        E(TT_NET, "[%s] node->state != NODE_PACKET_READY, state 0x%X\n", __func__, node->state);
        
      // sanity check, recycle node on error
      if (node->packet_length < MIN_ETH_FRAME_SIZE || node->packet_length > MAX_ETH_FRAME_SIZE) {
        E(TT_NET, "[%s] bogus node, node->packet_length %d\n", __func__, node->packet_length);
        if(node->aligned_packet_buffer) {
          aligned_free(node->aligned_packet_buffer);
          node->aligned_packet_buffer = aligned_malloc(MAX_ETH_FRAME_SIZE, 4);
        }
        node->packet_length = 0;
        node->state = NODE_FREE;
        continue;
      }

        // undo aligned offset for ethernet buffer
        buffer = unalign_aligned_buffer(node->aligned_packet_buffer);
        if(!buffer) {
          E(TT_NET, "[%s] system alert, possible heap exhaustion\n", __func__);
          continue;
        }
        memmove(buffer, node->aligned_packet_buffer, node->packet_length);

        E(TT_NET, "[%s] pass node to ethernet handler\n", __func__);
        
        // pass to ethernet handler
        emac->eth->stack_input(emac->eth, buffer, node->packet_length);

        // recycle node
        node->aligned_packet_buffer = aligned_malloc(MAX_ETH_FRAME_SIZE, 4);
        node->packet_length = 0;
        node->state = NODE_FREE;
    }
    
    // E(TT_NET, "[%s] loop bottom\n", __func__);
  }

out:

  E(TT_NET, "[%s] exiting\n", __func__);

  vTaskDelete(NULL);
}
#else /* ! ETHERNET_SPI_DMA_CHAINED */
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
    
    // E(TT_NET, "[%s] status = 0x%x, mask = 0x%x\n", __func__, status, mask);

    // if (status & EIR_RXERIF) {
    //   enc28j60_clear_buferr(emac);  // now this will execute on RX overflow
    // }

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
            buffer = nrc_mem_malloc(length);

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
        
        if (emac->parent.receive(&emac->parent, buffer, &length) == NRC_SUCCESS) {
          /* pass the buffer to stack (e.g. TCP/IP layer) */
          if (length > 0) {
            emac->eth->stack_input(emac->eth, buffer, length);
          } else {
            free(buffer);
          }
        } else {
          free(buffer);
        }
      } while (emac->packets_remain);
    }

    MAC_CHECK_NO_RET(emac->run_task == true, "exit task", out);
    
    // transmit error
    if (status & EIR_TXERIF) {
      // E(TT_NET, "[%s] status & EIR_TXERIF %d\n", __func__, (status & EIR_TXERIF) >> 8);
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
      // enc28j60_clear_buferr(emac);
    }

    // check for RX errors
    // if(status & EIR_RXERIF) {
    //   enc28j60_clear_buferr(emac);
    // }

  MAC_CHECK_NO_RET(emac->run_task == true, "exit task", out);

    // transmit ready
    if (status & EIR_TXIF) {
      // uint32_t tx_done_ticks = xTaskGetTickCount();
      // uint32_t tx_time_ms = (tx_done_ticks - emac->tx_start_ticks) * portTICK_PERIOD_MS;
      // E(TT_NET, "TX completion latency: %u ms\n", tx_time_ms);
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
#endif /* ! ETHERNET_SPI_DMA_CHAINED */

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

#ifdef ETHERNET_SPI_DMA_CHAINED
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_transmit(esp_eth_mac_t* mac, uint8_t* buf, uint32_t length)
{
  E(TT_NET, "[%s] ETHERNET_SPI_DMA_CHAINED\n", __func__);
  event_flags = EVT_TX_START;
  // enc28j60_schedule_dma_tx(buf, length);
  
  return NRC_SUCCESS;
}
#else /* ! ETHERNET_SPI_DMA_CHAINED */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_transmit(esp_eth_mac_t* mac, uint8_t* buf, uint32_t length)
{
    nrc_err_t ret = NRC_SUCCESS;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    uint8_t econ1 = 0;

#ifdef ENABLE_ETHERNET_INTERRUPT
    /* ENC28J60 may be a bottle neck in Eth communication. Hence we need to check if it is ready. */
    // if (xSemaphoreTake(emac->tx_ready_sem, pdMS_TO_TICKS(ENC28J60_TX_READY_TIMEOUT_MS)) == pdFALSE) {
    //     E(TT_NET, "tx_ready_sem expired\n");
    // }
#endif

    // E(TT_NET, "[%s]\n", __func__);

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
#endif /* ! ETHERNET_SPI_DMA_CHAINED */


#ifdef ETHERNET_SPI_DMA_CHAINED
static nrc_err_t emac_enc28j60_receive(esp_eth_mac_t* mac, uint8_t* buf, uint32_t* length) 
{
  E(TT_NET, "[%s]\n", __func__);
  return NRC_FAIL;
}
#else /* ! ETHERNET_SPI_DMA_CHAINED */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_receive(esp_eth_mac_t* mac, uint8_t* buf, uint32_t* length)
{
    nrc_err_t ret = NRC_SUCCESS;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    static int error_recovery_count = 0;
    uint8_t pk_counter = 0;
    uint16_t rx_len = 0;
    uint32_t next_packet_addr = 0;
    enc28j60_rx_header_t header;

    // E(TT_NET, "[%s]\n", __func__);

    // read packet header
    MAC_CHECK(enc28j60_read_packet(emac, emac->next_packet_ptr, (uint8_t *)&header, sizeof(header)) == NRC_SUCCESS,
              "read header failed", out, NRC_FAIL);

    // DUMP_HEX("RSV Header", (uint8_t*)&header, sizeof(header));
    
    V(TT_NET, "[%s] header.next_packet_low = 0x%x\n", __func__, header.next_packet_low);
    V(TT_NET, "[%s] header.next_packet_high = 0x%x\n", __func__, header.next_packet_high);
    V(TT_NET, "[%s] header.length_low = 0x%x\n", __func__, header.length_low);
    V(TT_NET, "[%s] header.length_high = 0x%x\n", __func__, header.length_high);
    V(TT_NET, "[%s] header.status_low = 0x%x\n", __func__, header.status_low);
    V(TT_NET, "[%s] header.status_high = 0x%x\n", __func__, header.status_high);

    // DUMP_HEX("received header", &header, sizeof(header));

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
}
#endif /* ! ETHERNET_SPI_DMA_CHAINED */

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
  int retry_cnt = 0;

  do {
    if (retry_cnt > MAX_RETRY_COUNT) {
      E(TT_NET, "[%s] failed to allocate size %d\n", __func__, SPI_TRANSFER_BUF_LEN);
      return NRC_FAIL;
    }
    spi_transfer_buf = nrc_mem_malloc(SPI_TRANSFER_BUF_LEN);
    if (!spi_transfer_buf) {
      // Memory allocation failed, wait for a short period before retrying
      _delay_ms(1);
    }
    retry_cnt++;
  } while (!spi_transfer_buf);

  return NRC_SUCCESS;
}
#endif

#ifdef ENABLE_ETHERNET_INTERRUPT
static void enc28j60_intr_handler(int vector)
{
    int input;

    if (nrc_gpio_inputb(gemac->int_gpio_num, &input) < 0) {
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
  if (nrc_gpio_register_interrupt_handler(INT_VECTOR0, emac->int_gpio_num, enc28j60_intr_handler) == NRC_SUCCESS) {
    V(TT_NET, "[%s] interrupt handler installed\n", __func__);
  }
  emac->interrupt_vector = INT_VECTOR0;
  system_irq_prio(emac->interrupt_vector,  configMAX_SYSCALL_INTERRUPT_PRIORITY);
  #define ENC28_INT_PRIO   0x08 
  if (emac->interrupt_vector != -1) {
    system_irq_prio(emac->interrupt_vector, ENC28_INT_PRIO);
    // NVIC_ClearPendingIRQ((IRQn_Type)emac->interrupt_vector);
  }
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
  vSemaphoreDelete(emac->spi_lock);
  vSemaphoreDelete(emac->reg_trans_lock);
  vSemaphoreDelete(emac->tx_ready_sem);
  free(emac);
  
#ifdef ETHERNET_DYNAMIC_BUFFERS
  free(spi_transfer_buf);
#endif

#ifdef ETHERNET_SPI_DMA
    if(rx_sem)
       vSemaphoreDelete(rx_sem);
    if(tx_sem)
       vSemaphoreDelete(tx_sem);
#ifdef ETHERNET_SPI_DMA_CHAINED
    // if(rx_ready_queue)
      // xQueueDelete(rx_ready_queue);
#endif /* ETHERNET_SPI_DMA_CHAINED */
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
  
  /* create mutex */
  emac->spi_lock = xSemaphoreCreateMutex();
  MAC_CHECK(emac->spi_lock, "create spi lock failed", err, NULL);
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
#ifdef ETHERNET_SPI_DMA_CHAINED
  rx_ready_queue = xQueueCreate(RX_NODE_COUNT, sizeof(rx_node_t *));
  MAC_CHECK(rx_ready_queue, "create rx_ready_queue queue failed", err, NULL);
  MAC_CHECK(dma_init_nodes() == NRC_SUCCESS,"dma_init_nodes() failed", err, NULL);
#endif /* ETHERNET_SPI_DMA_CHAINED */
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
                                     NRC_TASK_PRIORITY - 2,
                                     &emac->rx_task_hdl);
  MAC_CHECK(xReturned == pdPASS, "create enc28j60 task failed", err, NULL);

  return &(emac->parent);
err:
  if (emac) {
    if (emac->rx_task_hdl) {
      vTaskDelete(emac->rx_task_hdl);
    }
    if (emac->spi_lock) {
      vSemaphoreDelete(emac->spi_lock);
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
#ifdef ETHERNET_SPI_DMA_CHAINED
    // if(rx_ready_queue)
      // xQueueDelete(rx_ready_queue);
#endif /* ETHERNET_SPI_DMA_CHAINED */
#endif /* ETHERNET_SPI_DMA */
    free(emac);
  }
  return ret;
}
