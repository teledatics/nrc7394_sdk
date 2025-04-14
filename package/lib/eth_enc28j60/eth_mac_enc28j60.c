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

// #define ETHERNET_SPI_DMA 1
// #undef ENABLE_ETHERNET_INTERRUPT
// #define ENABLE_ETHERNET_INTERRUPT 1

#define ETHERNET_DYNAMIC_BUFFERS 1
#ifndef ETHERNET_DYNAMIC_BUFFERS
uint8_t spi_transfer_buf[SPI_TRANSFER_BUF_LEN];
#else
uint8_t *spi_transfer_buf;
#endif

#define TX_WAIT_TIMEOUT_MS  10
#define MAX_RETRY_COUNT     10
#define MAX_REG_RETRY_COUNT 1000

#define delay_us(x) vTaskDelay(pdMS_TO_TICKS(x) / 1000)

static const char* TAG = "enc28j60";

#define MAC_CHECK(a, str, goto_tag, ret_value, ...)                            \
  do {                                                                         \
    int retry = MAX_REG_RETRY_COUNT;                                               \
    if (!(a) || !(--retry)) {                                                  \
      E(TT_NET, "%s (%d): " str "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);  \
      ret = ret_value;                                                         \
      goto goto_tag;                                                           \
    }                                                                          \
    delay_us(10);                                                              \
  } while (0)

#define MAC_CHECK_NO_RET(a, str, goto_tag, ...)                                \
  do {                                                                         \
    int retry = MAX_REG_RETRY_COUNT;                                               \
    if (!(a) || !(--retry)) {                                                  \
      E(TT_NET, "%s (%d):" str  "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);  \
      goto goto_tag;                                                           \
    }                                                                          \
    delay_us(10);                                                              \
  } while (0)

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
  spi_device_t spi_hdl;
  SemaphoreHandle_t spi_lock;
  SemaphoreHandle_t reg_trans_lock;
  SemaphoreHandle_t tx_ready_sem;
  TaskHandle_t rx_task_hdl;
  uint32_t sw_reset_timeout_ms;
  uint32_t next_packet_ptr;
  uint32_t last_tsv_addr;
  int int_gpio_num;
  int interrupt_vector;
  uint8_t addr[6];
  uint8_t last_bank;
  bool packets_remain;
  eth_enc28j60_rev_t revision;
} emac_enc28j60_t;

static emac_enc28j60_t* gemac = NULL;

ATTR_NC __attribute__((optimize("O3"))) static inline bool
enc28j60_spi_lock(emac_enc28j60_t* emac)
{
  return xSemaphoreTake(emac->spi_lock,
                        pdMS_TO_TICKS(ENC28J60_SPI_LOCK_TIMEOUT_MS)) == pdTRUE;
}

ATTR_NC __attribute__((optimize("O3"))) static inline bool
enc28j60_spi_unlock(emac_enc28j60_t* emac)
{
  return xSemaphoreGive(emac->spi_lock) == pdTRUE;
}

ATTR_NC __attribute__((optimize("O3"))) static inline bool
enc28j60_reg_trans_lock(emac_enc28j60_t* emac)
{
  return xSemaphoreTake(emac->reg_trans_lock,
                        pdMS_TO_TICKS(ENC28J60_REG_TRANS_LOCK_TIMEOUT_MS)) ==
         pdTRUE;
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

  nrc_spi_start_xfer(spi);
  nrc_spi_xfer(spi, tx_buf, rx_buf, 1);
  memset(tx_buf, 0, sizeof(tx_buf));
  nrc_spi_xfer(spi, tx_buf, rx_buf, slen);
  nrc_spi_stop_xfer(spi);

  return val = rx_buf[slen - 1];
}

ATTR_NC __attribute__((optimize("O3"))) static int
spi_write_op(spi_device_t* spi, u8 op, u8 addr, u8 val)
{
  int ret;

  memset(spi_transfer_buf, 0, SPI_TRANSFER_BUF_LEN);

  spi_transfer_buf[0] = op | (addr & ADDR_MASK);
  spi_transfer_buf[1] = val;

  nrc_spi_start_xfer(spi);
  ret = nrc_spi_xfer(spi, spi_transfer_buf, spi_transfer_buf, 2);
  nrc_spi_stop_xfer(spi);

  return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static int
spi_write(spi_device_t* spi, u8* data, int len)
{
  int ret = NRC_SUCCESS;

  nrc_spi_start_xfer(spi);
  ret = nrc_spi_xfer(spi, data, data, len);
  nrc_spi_stop_xfer(spi);
  
  return ret;
}

/* NOTE: enc28j60 writes on rising edge and reads on falling edge, */
/*       Newracom SDK hides HW so this capability is not exposed    */
/*       code below mitigates this issue                           */
ATTR_NC __attribute__((optimize("O3"))) static int
spi_read_buf(spi_device_t* spi, int len, u8* data)
{
  static __attribute__((aligned(4)))
  u8* rx_buf = NULL;
  u8* tx_buf = NULL;
  int ret = NRC_SUCCESS;
  int idx;
  int xfer_len;
  int remaining;
  int chunk;
  
  memset(spi_transfer_buf, 0, SPI_TRANSFER_BUF_LEN);  
  rx_buf = spi_transfer_buf + 3;
  tx_buf = spi_transfer_buf;

  memset(data, 0, len);

  tx_buf[0] = ENC28J60_READ_BUF_MEM;

  remaining = len + 1;
  chunk = 1;
  idx = 0;
  
  nrc_spi_start_xfer(spi);
  while (remaining > 0) {

    xfer_len = (remaining > chunk) ? chunk : remaining;
    ret = nrc_spi_xfer(spi, tx_buf + idx, rx_buf + idx, xfer_len);
    idx += xfer_len;
    remaining -= xfer_len;
  }
  nrc_spi_stop_xfer(spi);

  if (ret == 0) {
    memcpy(data, &rx_buf[1], len);
  } else {
    E(TT_NET, "%s() failed: ret = %d\n", __func__, ret);
  }

  return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static int
spi_write_buf(spi_device_t* spi, int len, const u8* data)
{
  int ret = NRC_SUCCESS;
  
  if (len > SPI_TRANSFER_BUF_LEN - 1 || len <= 0) {
    ret = NRC_FAIL;
  } else {
    spi_transfer_buf[0] = ENC28J60_WRITE_BUF_MEM;
    memcpy(&spi_transfer_buf[1], data, len);
    ret = spi_write(spi, (u8*)spi_transfer_buf, len + 1);
  }

  return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
spi_device_transmit(spi_device_t spi_hdl,
                            spi_transaction_t* trans)
{
  nrc_err_t retval = NRC_FAIL;

  if (enc28j60_spi_lock(gemac)) {

  switch (trans->cmd) {
    case ENC28J60_SPI_CMD_WCR:

      if (spi_write_op(&spi_hdl,
                       ENC28J60_WRITE_CTRL_REG,
                       trans->addr,
                       trans->tx_data[0]) == NRC_SUCCESS) {
        retval = NRC_SUCCESS;
      }
      break;
    case ENC28J60_SPI_CMD_RCR:

      nrc_spi_writebyte_value(
        &spi_hdl, (trans->addr & ADDR_MASK) | ENC28J60_READ_CTRL_REG, 0);

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
      break;
    case ENC28J60_SPI_CMD_BFS:

      if (spi_write_op(
            &spi_hdl, ENC28J60_BIT_FIELD_SET, trans->addr, trans->tx_data[0]) ==
          NRC_SUCCESS) {
        retval = NRC_SUCCESS;
      }
      break;
    case ENC28J60_SPI_CMD_BFC:

      if (spi_write_op(
            &spi_hdl, ENC28J60_BIT_FIELD_CLR, trans->addr, trans->tx_data[0]) ==
          NRC_SUCCESS) {
        retval = NRC_SUCCESS;
      }
      break;
    case ENC28J60_SPI_CMD_WBM:

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

      if (spi_read_buf(&spi_hdl, trans->length / 8, trans->rx_buffer) ==
          NRC_SUCCESS) {
        retval = NRC_SUCCESS;
      }
      break;
    case ENC28J60_SPI_CMD_SRC:

      if (nrc_spi_writebyte_value(
            &spi_hdl, (trans->addr & ADDR_MASK) | ENC28J60_SOFT_RESET, 0) ==
          NRC_SUCCESS) {
        retval = NRC_SUCCESS;
      }
      break;
  }
  enc28j60_spi_unlock(gemac);
  } else {
    retval = NRC_FAIL;
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
 * @brief SPI operation wrapper for resetting ENC28J60
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_do_reset(emac_enc28j60_t* emac)
{
  nrc_err_t ret = NRC_SUCCESS;

  spi_transaction_t trans = {
    .cmd = ENC28J60_SPI_CMD_SRC, // Soft reset
    .addr = 0x1F,
  };
  
  if (spi_device_transmit(emac->spi_hdl, &trans) != NRC_SUCCESS) {
      // ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
      ret = NRC_FAIL;
  }

  // After reset, wait at least 1ms for the device to be ready
  delay_us(ENC28J60_SYSTEM_RESET_ADDITION_TIME_US);

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

    delay_us(15);

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

  if (enc28j60_reg_trans_lock(emac)) {
    MAC_CHECK(enc28j60_switch_register_bank(emac, (reg_addr & 0xF00) >> 8) ==
                NRC_SUCCESS,
              "switch bank failed",
              out,
              NRC_FAIL);
    MAC_CHECK(enc28j60_do_register_read(
                emac, !(reg_addr & 0xF000), reg_addr & 0xFF, value) == NRC_SUCCESS,
              "read register failed",
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
 * @brief Read ENC28J60 internal memroy
 */
ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
enc28j60_read_packet(emac_enc28j60_t* emac,
                     uint32_t addr,
                     uint8_t* packet,
                     uint32_t len)
{
  nrc_err_t ret = NRC_SUCCESS;

  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERDPTL, addr & 0xFF) ==
              NRC_SUCCESS,
            "write ERDPTL failed",
            out,
            NRC_FAIL);

  MAC_CHECK(enc28j60_register_write(
              emac, ENC28J60_ERDPTH, (addr & 0xFF00) >> 8) == NRC_SUCCESS,
            "write ERDPTH failed",
            out,
            NRC_FAIL);

  MAC_CHECK(enc28j60_do_memory_read(emac, packet, len) == NRC_SUCCESS,
            "read memory failed",
            out,
            NRC_FAIL);

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
  MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status) ==
              NRC_SUCCESS,
            "read MISTAT failed",
            out,
            NRC_FAIL);
  MAC_CHECK(
    !(mii_status & MISTAT_BUSY), "phy is busy", out, NRC_FAIL);

  /* tell the PHY address to write */
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MIREGADR, phy_reg & 0xFF) ==
              NRC_SUCCESS,
            "write MIREGADR failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MIWRL, reg_value & 0xFF) ==
              NRC_SUCCESS,
            "write MIWRL failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(
              emac, ENC28J60_MIWRH, (reg_value & 0xFF00) >> 8) == NRC_SUCCESS,
            "write MIWRH failed",
            out,
            NRC_FAIL);

  /* polling the busy flag */
  uint32_t to = 0;
  do {
    delay_us(15);
    MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status) ==
                NRC_SUCCESS,
              "read MISTAT failed",
              out,
              NRC_FAIL);
    to += 15;
  } while ((mii_status & MISTAT_BUSY) &&
           to < ENC28J60_PHY_OPERATION_TIMEOUT_US);
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
  MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status) ==
              NRC_SUCCESS,
            "read MISTAT failed",
            out,
            NRC_FAIL);
  MAC_CHECK(!(mii_status & MISTAT_BUSY), "phy is busy", out, NRC_FAIL);

  /* tell the PHY address to read */
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MIREGADR, phy_reg & 0xFF) ==
              NRC_SUCCESS,
            "write MIREGADR failed",
            out,
            NRC_FAIL);
  mii_cmd = MICMD_MIIRD;
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MICMD, mii_cmd) == NRC_SUCCESS,
            "write MICMD failed",
            out,
            NRC_FAIL);

  /* polling the busy flag */
  uint32_t to = 0;
  do {
    delay_us(15);
    MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status) ==
                NRC_SUCCESS,
              "read MISTAT failed",
              out,
              NRC_FAIL);
    to += 15;
  } while ((mii_status & MISTAT_BUSY) &&
           to < ENC28J60_PHY_OPERATION_TIMEOUT_US);
  MAC_CHECK(!(mii_status & MISTAT_BUSY), "phy is busy", out, NRC_FAIL);

  mii_cmd &= (~MICMD_MIIRD);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MICMD, mii_cmd) == NRC_SUCCESS,
            "write MICMD failed",
            out,
            NRC_FAIL);

  uint8_t value_l = 0;
  uint8_t value_h = 0;
  MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MIRDL, &value_l) == NRC_SUCCESS,
            "read MIRDL failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MIRDH, &value_h) == NRC_SUCCESS,
            "read MIRDH failed",
            out,
            NRC_FAIL);
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
  MAC_CHECK(enc28j60_register_read(
              emac, ENC28J60_EREVID, (uint8_t*)&emac->revision) == NRC_SUCCESS,
            "read EREVID failed",
            out,
            NRC_FAIL);
  
  MAC_CHECK(emac->revision >= ENC28J60_REV_B1 &&
              emac->revision <= ENC28J60_REV_B7,
            "wrong chip ID",
            out,
            NRC_FAIL);
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
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_EHT0 + i, 0x00) == NRC_SUCCESS,
              "write ENC28J60_EHT%d failed",
              out,
              NRC_FAIL,
              i);
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
  MAC_CHECK(enc28j60_register_write(
              emac, ENC28J60_ERXSTL, ENC28J60_BUF_RX_START & 0xFF) == NRC_SUCCESS,
            "write ERXSTL failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(
              emac, ENC28J60_ERXSTH, (ENC28J60_BUF_RX_START & 0xFF00) >> 8) ==
              NRC_SUCCESS,
            "write ERXSTH failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(
              emac, ENC28J60_ERXNDL, ENC28J60_BUF_RX_END & 0xFF) == NRC_SUCCESS,
            "write ERXNDL failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(
              emac, ENC28J60_ERXNDH, (ENC28J60_BUF_RX_END & 0xFF00) >> 8) ==
              NRC_SUCCESS,
            "write ERXNDH failed",
            out,
            NRC_FAIL);
  uint32_t erxrdpt = enc28j60_next_ptr_align_odd(
    ENC28J60_BUF_RX_START, ENC28J60_BUF_RX_START, ENC28J60_BUF_RX_END);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXRDPTL, erxrdpt & 0xFF) ==
              NRC_SUCCESS,
            "write ERXRDPTL failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(
              emac, ENC28J60_ERXRDPTH, (erxrdpt & 0xFF00) >> 8) == NRC_SUCCESS,
            "write ERXRDPTH failed",
            out,
            NRC_FAIL);

  // set up transmit buffer start + end
  MAC_CHECK(enc28j60_register_write(
              emac, ENC28J60_ETXSTL, ENC28J60_BUF_TX_START & 0xFF) == NRC_SUCCESS,
            "write ETXSTL failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(
              emac, ENC28J60_ETXSTH, (ENC28J60_BUF_TX_START & 0xFF00) >> 8) ==
              NRC_SUCCESS,
            "write ETXSTH failed",
            out,
            NRC_FAIL);

  // set up default filter mode: (unicast OR broadcast OR multicast) AND crc
  // valid
  MAC_CHECK(enc28j60_register_write(emac,
                                    ENC28J60_ERXFCON,
                                    ERXFCON_UCEN | ERXFCON_CRCEN |
                                      ERXFCON_BCEN | ERXFCON_MCEN) == NRC_SUCCESS,
            "write ERXFCON failed",
            out,
            NRC_FAIL);

  // enable MAC receive, enable pause control frame on Tx and Rx path
  MAC_CHECK(enc28j60_register_write(emac,
                                    ENC28J60_MACON1,
                                    MACON1_MARXEN | MACON1_RXPAUS |
                                      MACON1_TXPAUS) == NRC_SUCCESS,
            "write MACON1 failed",
            out,
            NRC_FAIL);
  // enable automatic padding, append CRC, check frame length, half duplex by
  // default (can update at runtime)
  MAC_CHECK(enc28j60_register_write(emac,
                                    ENC28J60_MACON3,
                                    MACON3_PADCFG0 | MACON3_TXCRCEN |
                                      MACON3_FRMLNEN) == NRC_SUCCESS,
            "write MACON3 failed",
            out,
            NRC_FAIL);
  // enable defer transmission (effective only in half duplex)
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MACON4, MACON4_DEFER) ==
              NRC_SUCCESS,
            "write MACON4 failed",
            out,
            NRC_FAIL);
  // set inter-frame gap (back-to-back)
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MABBIPG, 0x12) == NRC_SUCCESS,
            "write MABBIPG failed",
            out,
            NRC_FAIL);
  // set inter-frame gap (non-back-to-back)
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MAIPGL, 0x12) == NRC_SUCCESS,
            "write MAIPGL failed",
            out,
            NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MAIPGH, 0x0C) == NRC_SUCCESS,
            "write MAIPGH failed",
            out,
            NRC_FAIL);

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
  MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, 0xFF) == NRC_SUCCESS,
              "clear EIR failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_PKTIE | EIE_INTIE | EIE_TXERIE) == NRC_SUCCESS,
              "set EIE.[PKTIE|INTIE] failed", out, NRC_FAIL);
  /* enable rx logic */
  MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_RXEN) == NRC_SUCCESS,
              "set ECON1.RXEN failed", out, NRC_FAIL);

  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERDPTL, 0x00) == NRC_SUCCESS,
              "write ERDPTL failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERDPTH, 0x00) == NRC_SUCCESS,
              "write ERDPTH failed", out, NRC_FAIL);
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
  MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_EIE, 0xFF) == NRC_SUCCESS,
            "clear EIE failed",
            out,
            NRC_FAIL);
  /* disable rx */
  MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_RXEN) == NRC_SUCCESS,
            "clear ECON1.RXEN failed",
            out,
            NRC_FAIL);
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

static inline nrc_err_t
emac_enc28j60_get_tsv(emac_enc28j60_t* emac, enc28j60_tsv_t* tsv)
{
  return enc28j60_read_packet(
    emac, emac->last_tsv_addr, (uint8_t*)tsv, ENC28J60_TSV_SIZE);
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
#endif


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
  uint8_t mask = 0;
  uint8_t* buffer = NULL;
  uint32_t length = 0;
  int int_bit = 0;
  int retries = 0;
  int max_retries = 10;

  while (1) {
  loop_start:
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

    V(TT_NET, "[%s] interrupt asserted bit = %d\n", __func__, int_bit);
    // the host controller should clear the global enable bit for the interrupt pin before servicing the interrupt
    MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIE, EIE_INTIE) == NRC_SUCCESS,
                        "clear EIE_INTIE failed", loop_start);        // read interrupt status
    MAC_CHECK_NO_RET(enc28j60_do_register_read(emac, true, ENC28J60_EIR, &status) == NRC_SUCCESS,
                        "read EIR failed", loop_end);
    MAC_CHECK_NO_RET(enc28j60_do_register_read(emac, true, ENC28J60_EIE, &mask) == NRC_SUCCESS,
                        "read EIE failed", loop_end);
    status &= mask;
    V(TT_NET, "[%s] status = 0x%x, mask = 0x%x\n", __func__, status, mask);

#ifdef ENABLE_ETHERNET_INTERRUPT
    if (emac->interrupt_vector != -1) {
        system_irq_unmask(emac->interrupt_vector);
    }
#endif

    // When source of interrupt is unknown, try to check if there is packet
    // waiting (Errata #6 workaround)
    if (status == 0) {
      uint8_t pk_counter = 0;
      MAC_CHECK_NO_RET(
        enc28j60_register_read(emac, ENC28J60_EPKTCNT, &pk_counter) == NRC_SUCCESS,
        "read EPKTCNT failed",
        loop_end);
      if (pk_counter > 0) {
        status = EIR_PKTIF;
      } else {
        V(TT_NET, "[%s] goto loop_end...\n", __func__);
        goto loop_end;
      }
    }

    // packet received
    if (status & EIR_PKTIF) {
      do {
        /* read while there's packets remaining */
        do {
            length = ETH_MAX_PACKET_SIZE;
            buffer = nrc_mem_malloc(length);
            if (!buffer) {
               E(TT_NET, "[%s] buffer allocation failed, waiting 1 ms to recover...\n", __func__);
                _delay_ms(1);
            }
        } while (!buffer);
        if (emac->parent.receive(&emac->parent, buffer, &length) == NRC_SUCCESS) {
          /* pass the buffer to stack (e.g. TCP/IP layer) */
          if (length) {
            emac->eth->stack_input(emac->eth, buffer, length);
          } else {
            nrc_mem_free(buffer);
          }
        } else {
          nrc_mem_free(buffer);
        }
      } while (emac->packets_remain);
    }

    // transmit error
    if (status & EIR_TXERIF) {
      // Errata #12/#13 workaround - reset Tx state machine
      MAC_CHECK_NO_RET(
        enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_TXRST) == NRC_SUCCESS,
        "set TXRST failed",
        loop_end);
      MAC_CHECK_NO_RET(
        enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_TXRST) == NRC_SUCCESS,
        "clear TXRST failed",
        loop_end);

      // Clear Tx Error Interrupt Flag
      MAC_CHECK_NO_RET(
        enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXERIF) == NRC_SUCCESS,
        "clear TXERIF failed",
        loop_end);

      // Errata #13 workaround (applicable only to B5 and B7 revisions)
      if (emac->revision == ENC28J60_REV_B5 ||
          emac->revision == ENC28J60_REV_B7) {
        __attribute__((aligned(4)))
        enc28j60_tsv_t tx_status; // SPI driver needs the rx buffer 4 byte align
        MAC_CHECK_NO_RET(emac_enc28j60_get_tsv(emac, &tx_status) == NRC_SUCCESS,
                         "get Tx Status Vector failed",
                         loop_end);
        // Try to retransmit when late collision is indicated
        if (tx_status.late_collision) {
          // Clear Tx Interrupt status Flag (it was set along with the error)
          MAC_CHECK_NO_RET(
            enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXIF) == NRC_SUCCESS,
            "clear TXIF failed",
            loop_end);
          // Enable global interrupt flag and try to retransmit
          MAC_CHECK_NO_RET(
            enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_INTIE) == NRC_SUCCESS,
            "set INTIE failed",
            loop_end );

          MAC_CHECK_NO_RET(enc28j60_do_bitwise_set(
                             emac, ENC28J60_ECON1, ECON1_TXRTS) == NRC_SUCCESS,
                           "set TXRTS failed",
                           loop_end);
          continue; // no need to handle Tx ready interrupt nor to enable
                    // global interrupt at this point
        }
      }
    }

    // check for RX errors
    if(status & EIR_RXERIF) {

      MAC_CHECK_NO_RET(
        enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_RXRST) == NRC_SUCCESS,
        "set RXRST failed",
        loop_next);

      MAC_CHECK_NO_RET(
        enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_RXRST) == NRC_SUCCESS,
        "clear RXRST failed",
        loop_next);

      // Clear Rx Error Interrupt Flag
      MAC_CHECK_NO_RET(
        enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_RXERIF) == NRC_SUCCESS,
        "clear RXERIF failed",
        loop_next);
    }
loop_next:

    // transmit ready
    if (status & EIR_TXIF) {
      MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXIF) ==
                         NRC_SUCCESS,
                       "clear TXIF failed",
                       loop_end);
      MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIE, EIE_TXIE) ==
                         NRC_SUCCESS,
                       "clear TXIE failed",
                       loop_end);

#ifdef ENABLE_ETHERNET_INTERRUPT
      V(TT_NET, "[%s] xSemaphoreGive(emac->tx_ready_sem)\n", __func__);
      xSemaphoreGive(emac->tx_ready_sem);
#endif
    }
  loop_end:
    // restore global enable interrupt bit
    MAC_CHECK_NO_RET(enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_INTIE) ==
                       NRC_SUCCESS,
                     "clear INTIE failed",
                     loop_start);
    // Note: Interrupt flag PKTIF is cleared when PKTDEC is set (in receive
    // function)

  }

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

  MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MACON3, &mac3) == NRC_SUCCESS,
            "read MACON3 failed",
            out,
            NRC_FAIL);
  switch (duplex) {
    case ETH_DUPLEX_HALF:
      mac3 &= ~MACON3_FULDPX;
      MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MABBIPG, 0x12) == NRC_SUCCESS,
                "write MABBIPG failed",
                out,
                NRC_FAIL);
      I(TT_NET, "[%s] working in half duplex\n", __func__);
      break;
    case ETH_DUPLEX_FULL:
      mac3 |= MACON3_FULDPX;
      MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MABBIPG, 0x15) == NRC_SUCCESS,
                "write MABBIPG failed",
                out,
                NRC_FAIL);
      I(TT_NET, "[%s] working in full duplex\n", __func__);
      break;
    default:
      MAC_CHECK(false, "unknown duplex", out, NRC_FAIL);
      break;
  }
  MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MACON3, mac3) == NRC_SUCCESS,
            "write MACON3 failed",
            out,
            NRC_FAIL);
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
  int ret = NRC_SUCCESS;
  uint8_t econ1 = 0;
  uint32_t timeout = xTaskGetTickCount() + pdMS_TO_TICKS(TX_WAIT_TIMEOUT_MS);

  V(TT_NET, "[%s] checking if last transmit completed.\n", __func__);

  do {
    /* Check if last transmit complete */
    MAC_CHECK(enc28j60_do_register_read(emac, true, ENC28J60_ECON1, &econ1) == NRC_SUCCESS,
              "read ECON1 failed", out, NRC_FAIL);

        if(!(econ1 & ECON1_TXRTS))
          break;

        if (xTaskGetTickCount() >= timeout) {
            E(TT_NET, "timeout expired, transmission stuck\n");
            ret = NRC_FAIL;
            // MAC_CHECK_NO_RET(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_TXRST) == NRC_SUCCESS, "set TXRST failed", out);
            // MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_TXRST) == NRC_SUCCESS, "clear TXRST failed", out);
            // MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXERIF) == NRC_SUCCESS, "clear TXERIF failed", out);
            _delay_ms(10);
            return NRC_FAIL;
        }

        delay_us(100);

    } while (econ1 & ECON1_TXRTS);

out:
    return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_transmit(esp_eth_mac_t* mac, uint8_t* buf, uint32_t length)
{
    nrc_err_t ret = NRC_SUCCESS;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    uint8_t econ1 = 0;

#ifdef ENABLE_ETHERNET_INTERRUPT
    // /* ENC28J60 may be a bottle neck in Eth communication. Hence we need to check if it is ready. */
    // if (xSemaphoreTake(emac->tx_ready_sem, pdMS_TO_TICKS(ENC28J60_TX_READY_TIMEOUT_MS)) == pdFALSE) {
    //     E(TT_NET, "tx_ready_sem expired\n");
    // }
#endif

    MAC_CHECK(wait_for_transmit_completion(emac) == NRC_SUCCESS, "transmission busy", out, NRC_FAIL);

    /* Set the write pointer to start of transmit buffer area */
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_EWRPTL, ENC28J60_BUF_TX_START & 0xFF) == NRC_SUCCESS,
              "write EWRPTL failed", out, NRC_FAIL);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_EWRPTH, (ENC28J60_BUF_TX_START & 0xFF00) >> 8) == NRC_SUCCESS,
              "write EWRPTH failed", out, NRC_FAIL);

    /* Set the end pointer to correspond to the packet size given */
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ETXNDL, (ENC28J60_BUF_TX_START + length) & 0xFF) == NRC_SUCCESS,
              "write ETXNDL failed", out, NRC_FAIL);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ETXNDH, ((ENC28J60_BUF_TX_START + length) & 0xFF00) >> 8) == NRC_SUCCESS,
              "write ETXNDH failed", out, NRC_FAIL);

    /* copy data to tx memory */
    uint8_t per_pkt_control = 0; // MACON3 will be used to determine how the packet will be transmitted
    MAC_CHECK(enc28j60_do_memory_write(emac, &per_pkt_control, 1) == NRC_SUCCESS,
              "write packet control byte failed", out, NRC_FAIL);
    MAC_CHECK(enc28j60_do_memory_write(emac, buf, length) == NRC_SUCCESS,
              "buffer memory write failed", out, NRC_FAIL);
    emac->last_tsv_addr = ENC28J60_BUF_TX_START + length + 1;

    /* enable Tx Interrupt to indicate next Tx ready state */
    MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXIF) == NRC_SUCCESS,
                "set EIR_TXIF failed", out, NRC_FAIL);
    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_TXIE) == NRC_SUCCESS,
                "set EIE_TXIE failed", out, NRC_FAIL);

    /* issue tx polling command */
    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_TXRTS) == NRC_SUCCESS,
              "set ECON1.TXRTS failed", out, NRC_FAIL);
out:
    return ret;
}

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_receive(esp_eth_mac_t* mac, uint8_t* buf, uint32_t* length)
{
    nrc_err_t ret = NRC_SUCCESS;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    uint8_t pk_counter = 0;
    uint16_t rx_len = 0;
    uint32_t next_packet_addr = 0;
    enc28j60_rx_header_t header;

    V(TT_NET, "[%s] calling enc28j60_read_packet.\n", __func__);
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
    
    V(TT_NET, "[%s] rx_len = %d.\n", __func__, rx_len);
    
    if ((rx_len <= 0) || (rx_len > ETH_MAX_PACKET_SIZE)) {
      ret = NRC_FAIL;
      E(TT_NET, "[%s] Error invalid receive length %d\n", __func__, rx_len);
      // MAC_CHECK_NO_RET(enc28j60_do_bitwise_set(emac, ENC28J60_ECON2, ECON2_PKTDEC) == NRC_SUCCESS, "set ECON2.PKTDEC failed", out);
      // MAC_CHECK_NO_RET(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_RXRST) == NRC_SUCCESS, "set RXRST failed", out);
      MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_RXRST) == NRC_SUCCESS, "clear RXRST failed", out);
      // MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_RXERIF) == NRC_SUCCESS, "clear RXERIF failed", out);
      return NRC_FAIL;
    }
    
    next_packet_addr = header.next_packet_low + (header.next_packet_high << 8);

    // read packet content
    MAC_CHECK(enc28j60_read_packet(emac, enc28j60_rx_packet_start(emac->next_packet_ptr, ENC28J60_RSV_SIZE), buf, rx_len) == NRC_SUCCESS,
              "read packet content failed", out, NRC_FAIL);

    // free receive buffer space
    uint32_t erxrdpt = enc28j60_next_ptr_align_odd(next_packet_addr, ENC28J60_BUF_RX_START, ENC28J60_BUF_RX_END);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXRDPTL, (erxrdpt & 0xFF)) == NRC_SUCCESS,
              "write ERXRDPTL failed", out, NRC_FAIL);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXRDPTH, (erxrdpt & 0xFF00) >> 8) == NRC_SUCCESS,
              "write ERXRDPTH failed", out, NRC_FAIL);
    emac->next_packet_ptr = next_packet_addr;

    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_ECON2, ECON2_PKTDEC) == NRC_SUCCESS,
              "set ECON2.PKTDEC failed", out, NRC_FAIL);
    MAC_CHECK(enc28j60_register_read(emac, ENC28J60_EPKTCNT, &pk_counter) == NRC_SUCCESS,
              "read EPKTCNT failed", out, NRC_FAIL);

    *length = rx_len - 4; // substract the CRC length
    emac->packets_remain = pk_counter > 0;
out:
    return ret;
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

    gemac->interrupt_vector = vector;
    if (nrc_gpio_inputb(gemac->int_gpio_num, &input) < 0) {
        return;
    }

    if (!input) {
        V(TT_NET, "[%s] call enc28j60_isr_handler\n", __func__);
        system_irq_mask(gemac->interrupt_vector);
        enc28j60_isr_handler(gemac);
    } else {
        V(TT_NET, "[%s] bogus interrupt\n", __func__);
    }
}
#endif

ATTR_NC __attribute__((optimize("O3"))) static nrc_err_t
emac_enc28j60_init(esp_eth_mac_t* mac)
{
  nrc_err_t ret = NRC_SUCCESS;
  emac_enc28j60_t* emac = __containerof(mac, emac_enc28j60_t, parent);
  esp_eth_mediator_t* eth = emac->eth;

  // util_trace_set_log_level(TT_NET, 0);
  
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
  
  /* reset enc28j60 */
  MAC_CHECK(enc28j60_do_reset(emac) == NRC_SUCCESS, "reset enc28j60 failed", out, NRC_FAIL);
  /* verify chip id */
  MAC_CHECK(enc28j60_verify_id(emac) == NRC_SUCCESS, "verify chip ID failed", out, NRC_FAIL);
  /* default setup of internal registers */
  MAC_CHECK(enc28j60_setup_default(emac) == NRC_SUCCESS, "enc28j60 default setup failed", out, NRC_FAIL);
  /* clear multicast hash table */
  MAC_CHECK(enc28j60_clear_multicast_table(emac) == NRC_SUCCESS, "clear multicast table failed", out, NRC_FAIL);
  /* clear powersave mode */
  MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_ECON2, ECON2_PWRSV) == NRC_SUCCESS, "clr ECON2_PWRSV failed", out, NRC_FAIL);
  MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_ECON2, ECON2_VRPS) == NRC_SUCCESS, "clr ECON2_VRPS failed", out, NRC_FAIL);
  
#ifdef ENABLE_ETHERNET_INTERRUPT
  if (nrc_gpio_register_interrupt_handler(INT_VECTOR0, emac->int_gpio_num, enc28j60_intr_handler) == NRC_SUCCESS) {
    V(TT_NET, "[%s] interrupt handler installed\n", __func__);
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
  /* create mutex */
  emac->spi_lock = xSemaphoreCreateMutex();
  MAC_CHECK(emac->spi_lock, "create spi lock failed", err, NULL);
  emac->reg_trans_lock = xSemaphoreCreateMutex();
  MAC_CHECK(
    emac->reg_trans_lock, "create register transaction lock failed", err, NULL);
  emac->tx_ready_sem = xSemaphoreCreateBinary();
  MAC_CHECK(emac->tx_ready_sem,
            "create pkt transmit ready semaphore failed",
            err,
            NULL);
  xSemaphoreGive(emac->tx_ready_sem); // ensures the first transmit is
                                      // performed without waiting
  /* create enc28j60 task */
  BaseType_t core_num = 0; // tskNO_AFFINITY;
  if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
    core_num = 0; // cpu_hal_get_core_id();
  }

  BaseType_t xReturned = xTaskCreate(emac_enc28j60_task,
                                     "enc28j60_tsk",
                                     mac_config->rx_task_stack_size,
                                     emac,
                                     // mac_config->rx_task_prio,
                                      NRC_TASK_PRIORITY - 1,
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
    free(emac);
  }
  return ret;
}
