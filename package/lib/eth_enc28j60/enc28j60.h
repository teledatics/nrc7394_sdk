// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "nrc_sdk.h"
#include "api_spi.h"
#include "eth_mac.h"
#include "eth_phy.h"

/**
 * @brief SPI Instruction Set
 *
 */
#define ENC28J60_SPI_CMD_RCR (0x00) // Read Control Register
#define ENC28J60_SPI_CMD_RBM (0x01) // Read Buffer Memory
#define ENC28J60_SPI_CMD_WCR (0x02) // Write Control Register
#define ENC28J60_SPI_CMD_WBM (0x03) // Write Buffer Memory
#define ENC28J60_SPI_CMD_BFS (0x04) // Bit Field Set
#define ENC28J60_SPI_CMD_BFC (0x05) // Bit Field Clear
#define ENC28J60_SPI_CMD_SRC (0x07) // Soft Reset

/**
 * @brief Shared Registers in ENC28J60 (accessible on each bank)
 *
 */
#define ENC28J60_EIE (0x1B)   // Ethernet Interrupt Enable
#define ENC28J60_EIR (0x1C)   // Ethernet Interrupt flags
#define ENC28J60_ESTAT (0x1D) // Ethernet Status
#define ENC28J60_ECON2 (0x1E) // Ethernet Control Register2
#define ENC28J60_ECON1 (0x1F) // Ethernet Control Register1

/**
 * @brief Per-bank Registers in ENC28J60
 * @note Address[15:12]: Register Type, 0 -> ETH, 1 -> MII/MAC
 *       Address[11:8] : Bank address
 *       Address[7:0]  : Register Index
 */
// Bank 0 Registers
#define ENC28J60_ERDPTL (0x0000)   // Read Pointer Low Byte ERDPT<7:0>)
#define ENC28J60_ERDPTH (0x0001)   // Read Pointer High Byte (ERDPT<12:8>)
#define ENC28J60_EWRPTL (0x0002)   // Write Pointer Low Byte (EWRPT<7:0>)
#define ENC28J60_EWRPTH (0x0003)   // Write Pointer High Byte (EWRPT<12:8>)
#define ENC28J60_ETXSTL (0x0004)   // TX Start Low Byte (ETXST<7:0>)
#define ENC28J60_ETXSTH (0x0005)   // TX Start High Byte (ETXST<12:8>)
#define ENC28J60_ETXNDL (0x0006)   // TX End Low Byte (ETXND<7:0>)
#define ENC28J60_ETXNDH (0x0007)   // TX End High Byte (ETXND<12:8>)
#define ENC28J60_ERXSTL (0x0008)   // RX Start Low Byte (ERXST<7:0>)
#define ENC28J60_ERXSTH (0x0009)   // RX Start High Byte (ERXST<12:8>)
#define ENC28J60_ERXNDL (0x000A)   // RX End Low Byte (ERXND<7:0>)
#define ENC28J60_ERXNDH (0x000B)   // RX End High Byte (ERXND<12:8>)
#define ENC28J60_ERXRDPTL (0x000C) // RX RD Pointer Low Byte (ERXRDPT<7:0>)
#define ENC28J60_ERXRDPTH (0x000D) // RX RD Pointer High Byte (ERXRDPT<12:8>)
#define ENC28J60_ERXWRPTL (0x000E) // RX WR Pointer Low Byte (ERXWRPT<7:0>)
#define ENC28J60_ERXWRPTH (0x000F) // RX WR Pointer High Byte (ERXWRPT<12:8>)
#define ENC28J60_EDMASTL (0x0010)  // DMA Start Low Byte (EDMAST<7:0>)
#define ENC28J60_EDMASTH (0x0011)  // DMA Start High Byte (EDMAST<12:8>)
#define ENC28J60_EDMANDL (0x0012)  // DMA End Low Byte (EDMAND<7:0>)
#define ENC28J60_EDMANDH (0x0013)  // DMA End High Byte (EDMAND<12:8>)
#define ENC28J60_EDMADSTL (0x0014) // DMA Destination Low Byte (EDMADST<7:0>)
#define ENC28J60_EDMADSTH (0x0015) // DMA Destination High Byte (EDMADST<12:8>)
#define ENC28J60_EDMACSL (0x0016)  // DMA Checksum Low Byte (EDMACS<7:0>)
#define ENC28J60_EDMACSH (0x0017)  // DMA Checksum High Byte (EDMACS<15:8>)

// Bank 1 Registers
#define ENC28J60_EHT0 (0x0100)   // Hash Table Byte 0 (EHT<7:0>)
#define ENC28J60_EHT1 (0x0101)   // Hash Table Byte 1 (EHT<15:8>)
#define ENC28J60_EHT2 (0x0102)   // Hash Table Byte 2 (EHT<23:16>)
#define ENC28J60_EHT3 (0x0103)   // Hash Table Byte 3 (EHT<31:24>)
#define ENC28J60_EHT4 (0x0104)   // Hash Table Byte 4 (EHT<39:32>)
#define ENC28J60_EHT5 (0x0105)   // Hash Table Byte 5 (EHT<47:40>)
#define ENC28J60_EHT6 (0x0106)   // Hash Table Byte 6 (EHT<55:48>)
#define ENC28J60_EHT7 (0x0107)   // Hash Table Byte 7 (EHT<63:56>)
#define ENC28J60_EPMM0 (0x0108)  // Pattern Match Mask Byte 0 (EPMM<7:0>)
#define ENC28J60_EPMM1 (0x0109)  // Pattern Match Mask Byte 1 (EPMM<15:8>)
#define ENC28J60_EPMM2 (0x010A)  // Pattern Match Mask Byte 2 (EPMM<23:16>)
#define ENC28J60_EPMM3 (0x010B)  // Pattern Match Mask Byte 3 (EPMM<31:24>)
#define ENC28J60_EPMM4 (0x010C)  // Pattern Match Mask Byte 4 (EPMM<39:32>)
#define ENC28J60_EPMM5 (0x010D)  // Pattern Match Mask Byte 5 (EPMM<47:40>)
#define ENC28J60_EPMM6 (0x010E)  // Pattern Match Mask Byte 6 (EPMM<55:48>)
#define ENC28J60_EPMM7 (0x010F)  // Pattern Match Mask Byte 7 (EPMM<63:56>)
#define ENC28J60_EPMCSL (0x0110) // Pattern Match Checksum Low Byte (EPMCS<7:0>)
#define ENC28J60_EPMCSH                                                        \
  (0x0111) // Pattern Match Checksum High Byte (EPMCS<15:0>)
#define ENC28J60_EPMOL (0x0114)   // Pattern Match Offset Low Byte (EPMO<7:0>)
#define ENC28J60_EPMOH (0x0115)   // Pattern Match Offset High Byte (EPMO<12:8>)
#define ENC28J60_ERXFCON (0x0118) // Receive Fileter Control
#define ENC28J60_EPKTCNT (0x0119) // Ethernet Packet Count

// Bank 2 Register
#define ENC28J60_MACON1 (0x1200)  // MAC Control Register 1
#define ENC28J60_MACON2 (0x1201)  // MAC Control Register 2
#define ENC28J60_MACON3 (0x1202)  // MAC Control Register 3
#define ENC28J60_MACON4 (0x1203)  // MAC Control Register 4
#define ENC28J60_MABBIPG (0x1204) // Back-to-Back Inter-Packet Gap (BBIPG<6:0>)
#define ENC28J60_MAIPGL                                                        \
  (0x1206) // Non-Back-to-Back Inter-Packet Gap Low Byte (MAIPGL<6:0>)
#define ENC28J60_MAIPGH                                                        \
  (0x1207) // Non-Back-to-Back Inter-Packet Gap High Byte (MAIPGH<6:0>)
#define ENC28J60_MACLCON1 (0x1208) // Retransmission Maximum (RETMAX<3:0>)
#define ENC28J60_MACLCON2 (0x1209) // Collision Window (COLWIN<5:0>)
#define ENC28J60_MAMXFLL (0x120A) // Maximum Frame Length Low Byte (MAMXFL<7:0>)
#define ENC28J60_MAMXFLH                                                       \
  (0x120B)                      // Maximum Frame Length High Byte (MAMXFL<15:8>)
#define ENC28J60_MICMD (0x1212) // MII Command Register
#define ENC28J60_MIREGADR (0x1214) // MII Register Address (MIREGADR<4:0>)
#define ENC28J60_MIWRL (0x1216)    // MII Write Data Low Byte (MIWR<7:0>)
#define ENC28J60_MIWRH (0x1217)    // MII Write Data High Byte (MIWR<15:8>)
#define ENC28J60_MIRDL (0x1218)    // MII Read Data Low Byte (MIRD<7:0>)
#define ENC28J60_MIRDH (0x1219)    // MII Read Data High Byte(MIRD<15:8>)

// Bank 3 Registers
#define ENC28J60_MAADR5 (0x1300) // MAC Address Byte 5 (MAADR<15:8>)
#define ENC28J60_MAADR6 (0x1301) // MAC Address Byte 6 (MAADR<7:0>)
#define ENC28J60_MAADR3                                                        \
  (0x1302) // MAC Address Byte 3 (MAADR<31:24>), OUI Byte 3
#define ENC28J60_MAADR4 (0x1303) // MAC Address Byte 4 (MAADR<23:16>)
#define ENC28J60_MAADR1                                                        \
  (0x1304) // MAC Address Byte 1 (MAADR<47:40>), OUI Byte 1
#define ENC28J60_MAADR2                                                        \
  (0x1305) // MAC Address Byte 2 (MAADR<39:32>), OUI Byte 2
#define ENC28J60_EBSTSD (0x0306)  // Built-in Self-Test Fill Seed (EBSTSD<7:0>)
#define ENC28J60_EBSTCON (0x0307) // Built-in Self-Test Control
#define ENC28J60_EBSTCSL                                                       \
  (0x0308) // Built-in Self-Test Checksum Low Byte (EBSTCS<7:0>)
#define ENC28J60_EBSTCSH                                                       \
  (0x0309) // Built-in Self-Test Checksum High Byte (EBSTCS<15:8>)
#define ENC28J60_MISTAT (0x130A)  // MII Status Register
#define ENC28J60_EREVID (0x0312)  // Ethernet Revision ID (EREVID<4:0>)
#define ENC28J60_ECOCON (0x0315)  // Clock Output Control Register
#define ENC28J60_EFLOCON (0x0317) // Ethernet Flow Control
#define ENC28J60_EPAUSL (0x0318)  // Pause Timer Value Low Byte (EPAUS<7:0>)
#define ENC28J60_EPAUSH (0x0319)  // Pause Timer Value High Byte (EPAUS<15:8>)

/**
 * @brief status and flag of ENC28J60 specific registers
 *
 */
// EIE bit definitions
#define EIE_INTIE (1 << 7)  // Global INT Interrupt Enable
#define EIE_PKTIE (1 << 6)  // Receive Packet Pending Interrupt Enable
#define EIE_DMAIE (1 << 5)  // DMA Interrupt Enable
#define EIE_LINKIE (1 << 4) // Link Status Change Interrupt Enable
#define EIE_TXIE (1 << 3)   // Transmit Enable
#define EIE_TXERIE (1 << 1) // Transmit Error Interrupt Enable
#define EIE_RXERIE (1 << 0) // Receive Error Interrupt Enable

// EIR bit definitions
#define EIR_PKTIF (1 << 6)  // Receive Packet Pending Interrupt Flag
#define EIR_DMAIF (1 << 5)  // DMA Interrupt Flag
#define EIR_LINKIF (1 << 4) // Link Change Interrupt Flag
#define EIR_TXIF (1 << 3)   // Transmit Interrupt Flag
#define EIR_TXERIF (1 << 1) // Transmit Error Interrupt Flag
#define EIR_RXERIF (1 << 0) // Receive Error Interrupt Flag

// ESTAT bit definitions
#define ESTAT_INT (1 << 7)     // INT Interrupt Flag
#define ESTAT_BUFER (1 << 6)   // Buffer Error Status
#define ESTAT_LATECOL (1 << 4) // Late Collision Error
#define ESTAT_RXBUSY (1 << 2)  // Receive Busy
#define ESTAT_TXABRT (1 << 1)  // Transmit Abort Error
#define ESTAT_CLKRDY (1 << 0)  // Clock Ready

// ECON2 bit definitions
#define ECON2_AUTOINC (1 << 7) // Automatic Buffer Pointer Increment Enable
#define ECON2_PKTDEC (1 << 6)  // Packet Decrement
#define ECON2_PWRSV (1 << 5)   // Power Save Enable
#define ECON2_VRPS (1 << 3)    // Voltage Regulator Power Save Enable

// ECON1 bit definitions
#define ECON1_TXRST (1 << 7)  // Transmit Logic Reset
#define ECON1_RXRST (1 << 6)  // Receive Logic Reset
#define ECON1_DMAST (1 << 5)  // DMA Start and Busy Status
#define ECON1_CSUMEN (1 << 4) // DMA Checksum Enable
#define ECON1_TXRTS (1 << 3)  // Transmit Request to Send
#define ECON1_RXEN (1 << 2)   // Receive Enable
#define ECON1_BSEL1 (1 << 1)  // Bank Select1
#define ECON1_BSEL0 (1 << 0)  // Bank Select0

// ERXFCON bit definitions
#define ERXFCON_UCEN (1 << 7)  // Unicast Filter Enable
#define ERXFCON_ANDOR (1 << 6) // AND/OR Filter Select
#define ERXFCON_CRCEN (1 << 5) // Post-Filter CRC Check Enable
#define ERXFCON_PMEN (1 << 4)  // Pattern Match Filter Enable
#define ERXFCON_MPEN (1 << 3)  // Magic Packet Filter Enable
#define ERXFCON_HTEN (1 << 2)  // Hash Table Filter Enable
#define ERXFCON_MCEN (1 << 1)  // Multicast Filter Enable
#define ERXFCON_BCEN (1 << 0)  // Broadcast Filter Enable

// MACON1 bit definitions
#define MACON1_TXPAUS (1 << 3)  // Pause Control Frame Transmission Enable
#define MACON1_RXPAUS (1 << 2)  // Pause Control Frame Reception Enable
#define MACON1_PASSALL (1 << 1) // Pass All Received Frames Enable
#define MACON1_MARXEN (1 << 0)  // MAC Receive Enable

// MACON3 bit definitions
#define MACON3_PADCFG2 (1 << 7) // Automatic Pad and CRC Configuration bit 2
#define MACON3_PADCFG1 (1 << 6) // Automatic Pad and CRC Configuration bit 1
#define MACON3_PADCFG0 (1 << 5) // Automatic Pad and CRC Configuration bit 0
#define MACON3_TXCRCEN (1 << 4) // Transmit CRC Enable
#define MACON3_PHDRLEN (1 << 3) // Proprietary Header Enable
#define MACON3_HFRMLEN (1 << 2) // Huge Frame Enable
#define MACON3_FRMLNEN (1 << 1) // Frame Length Checking Enable
#define MACON3_FULDPX (1 << 0)  // MAC Full-Duplex Enable

// MACON4 bit definitions
#define MACON4_DEFER (1 << 6)  // Defer Transmission Enable
#define MACON4_BPEN (1 << 5)   // No Backoff During Backpressure Enable
#define MACON4_NOBKFF (1 << 4) // No Backoff Enable

// MICMD bit definitions
#define MICMD_MIISCAN (1 << 1) // MII Scan Enable
#define MICMD_MIIRD (1 << 0)   // MII Read Enable

// EBSTCON bit definitions
#define EBSTCON_PSV2 (1 << 7)   // Pattern Shift Value 2
#define EBSTCON_PSV1 (1 << 6)   // Pattern Shift Value 1
#define EBSTCON_PSV0 (1 << 5)   // Pattern Shift Value 0
#define EBSTCON_PSEL (1 << 4)   // Port Select
#define EBSTCON_TMSEL1 (1 << 3) // Test Mode Select 1
#define EBSTCON_TMSEL0 (1 << 2) // Test Mode Select 0
#define EBSTCON_TME (1 << 1)    // Test Mode Enable
#define EBSTCON_BISTST (1 << 0) // Built-in Self-Test Start/Busy

// MISTAT bit definitions
#define MISTAT_NVALID (1 << 2) // MII Management Read Data Not Valid
#define MISTAT_SCAN (1 << 1)   // MII Management Scan Operation in Progress
#define MISTAT_BUSY (1 << 0)   // MII Management Busy

// EFLOCON bit definitions
#define EFLOCON_FULDPXS (1 << 2) // Full-Duplex Shadown
#define EFLOCON_FCEN1 (1 << 1)   // Flow Control Enable 1
#define EFLOCON_FCEN0 (1 << 0)   // Flow Control Enable 0

#define CS_HOLD_TIME_MIN_NS 210

/* buffer boundaries applied to internal 8K ram
 * entire available packet buffer space is allocated.
 * Give TX buffer space for one full ethernet frame (~1500 bytes)
 * receive buffer gets the rest */
#define TXSTART_INIT 0x1A00
#define TXEND_INIT 0x1FFF

/* Put RX buffer at 0 as suggested by the Errata datasheet */
#define RXSTART_INIT 0x0000
#define RXEND_INIT 0x19FF

/* maximum ethernet frame length */
#define MAX_FRAMELEN 1518

#define SPI_TRANSFER_BUF_LEN (4 + MAX_FRAMELEN)

/*
 * ENC28J60 Control Registers
 * Control register definitions are a combination of address,
 * bank number, and Ethernet/MAC/PHY indicator bits.
 * - Register address	(bits 0-4)
 * - Bank number	(bits 5-6)
 * - MAC/MII indicator	(bit 7)
 */
#define ADDR_MASK 0x1F
#define BANK_MASK 0x60
#define SPRD_MASK 0x80

/* SPI operation codes */
#define ENC28J60_READ_CTRL_REG 0x00
#define ENC28J60_READ_BUF_MEM 0x3A
#define ENC28J60_WRITE_CTRL_REG 0x40
#define ENC28J60_WRITE_BUF_MEM 0x7A
#define ENC28J60_BIT_FIELD_SET 0x80
#define ENC28J60_BIT_FIELD_CLR 0xA0
#define ENC28J60_SOFT_RESET 0xFF

/* ENC28J60 Receive Status Vector */
#define RSV_RXLONGEVDROPEV 0
#define RSV_CARRIEREV 2
#define RSV_CRCERROR 4
#define RSV_LENCHECKERR 5
#define RSV_LENOUTOFRANGE 6
#define RSV_RXOK 7
#define RSV_RXMULTICAST 8
#define RSV_RXBROADCAST 9
#define RSV_DRIBBLENIBBLE 10
#define RSV_RXCONTROLFRAME 11
#define RSV_RXPAUSEFRAME 12
#define RSV_RXUNKNOWNOPCODE 13
#define RSV_RXTYPEVLAN 14

#define RSV_SIZE 6
#define RSV_BITMASK(x) (1 << (x))
#define RSV_GETBIT(x, y) (((x)&RSV_BITMASK(y)) ? 1 : 0)

#define SPI_OPLEN 1

typedef spi_device_t spi_device_handle_t;

/**
* This structure describes one SPI transaction. The descriptor should not be
* modified until the transaction finishes.
*/
struct spi_transaction_t
{
uint32_t flags; ///< Bitwise OR of SPI_TRANS_* flags
uint16_t
cmd; /**< Command data, of which the length is set in the ``command_bits``
* of spi_device_interface_config_t.
*
*  <b>NOTE: this field, used to be "command" in ESP-IDF 2.1 and
* before, is re-written to be used in a new way in ESP-IDF 3.0.</b>
*
*  Example: write 0x0123 and command_bits=12 to send command 0x12,
* 0x3_ (in previous version, you may have to write 0x3_12).
*/
uint64_t
addr;          /**< Address data, of which the length is set in the
* ``address_bits`` of spi_device_interface_config_t.
*
*  <b>NOTE: this field, used to be "address" in ESP-IDF 2.1 and
* before, is re-written to be used in a new way in ESP-IDF3.0.</b>
*
*  Example: write 0x123400 and address_bits=24 to send address of
* 0x12, 0x34, 0x00 (in previous version, you may have to write
* 0x12340000).
*/
size_t length;   ///< Total data length, in bits
size_t rxlength; ///< Total data length received, should be not greater than
 ///< ``length`` in full-duplex mode (0 defaults this to the
 ///< value of ``length``).
void*
user; ///< User-defined variable. Can be used to store eg transaction ID.
union
{
const void*
tx_buffer; ///< Pointer to transmit buffer, or NULL for no MOSI phase
uint8_t tx_data[4]; ///< If SPI_TRANS_USE_TXDATA is set, data set here is
///< sent directly from this variable.
};
union
{
void* rx_buffer; ///< Pointer to receive buffer, or NULL for no MISO
 ///< phase. Written by 4 bytes-unit if DMA is used.
uint8_t rx_data[4]; ///< If SPI_TRANS_USE_RXDATA is set, data is received
///< directly to this variable
};
};

typedef struct spi_transaction_t spi_transaction_t;

#define SPI_TRANS_MODE_DIO (1 << 0) ///< Transmit/receive data in 2-bit mode
#define SPI_TRANS_MODE_QIO (1 << 1) ///< Transmit/receive data in 4-bit mode
#define SPI_TRANS_USE_RXDATA                                                   \
(1 << 2) ///< Receive into rx_data member of spi_transaction_t instead into
 ///< memory at rx_buffer.
#define SPI_TRANS_USE_TXDATA                                                   \
(1 << 3) ///< Transmit tx_data member of spi_transaction_t instead of data at
 ///< tx_buffer. Do not set tx_buffer when using this.
#define SPI_TRANS_MODE_DIOQIO_ADDR                                             \
(1 << 4) ///< Also transmit address in mode selected by
 ///< SPI_MODE_DIO/SPI_MODE_QIO
#define SPI_TRANS_VARIABLE_CMD                                                 \
(1 << 5) ///< Use the ``command_bits`` in ``spi_transaction_ext_t`` rather
 ///< than default value in ``spi_device_interface_config_t``.
#define SPI_TRANS_VARIABLE_ADDR                                                \
(1 << 6) ///< Use the ``address_bits`` in ``spi_transaction_ext_t`` rather
 ///< than default value in ``spi_device_interface_config_t``.
#define SPI_TRANS_VARIABLE_DUMMY                                               \
(1 << 7) ///< Use the ``dummy_bits`` in ``spi_transaction_ext_t`` rather than
 ///< default value in ``spi_device_interface_config_t``.
#define SPI_TRANS_CS_KEEP_ACTIVE                                               \
(1 << 8) ///< Keep CS active after data transfer
#define SPI_TRANS_MULTILINE_CMD   (1 << 9) ///< The data lines used at command phase is the same as data phase
 ///< (otherwise, only one data line is used at command phase)
#define SPI_TRANS_MODE_OCT        (1 << 10) ///< Transmit/receive data in 8-bit mode
#define SPI_TRANS_MULTILINE_ADDR SPI_TRANS_MODE_DIOQIO_ADDR ///< The data lines used at address phase is the
///< same as data phase (otherwise, only one data
///< line is used at address phase)

/**
 * @brief ENC28J60 specific configuration
 *
 */
// typedef struct
// {
// spi_device_handle_t spi_hdl; /*!< Handle of SPI device driver */
// int int_gpio_num;            /*!< Interrupt GPIO number */
// } eth_enc28j60_config_t;

/**
 * @brief ENC28J60 Supported Revisions
 *
 */
typedef enum
{
  ENC28J60_REV_B1 = 0b00000010,
  ENC28J60_REV_B4 = 0b00000100,
  ENC28J60_REV_B5 = 0b00000101,
  ENC28J60_REV_B7 = 0b00000110
} eth_enc28j60_rev_t;

/**
 * @brief Default ENC28J60 specific configuration
 *
 */
/* #define ETH_ENC28J60_DEFAULT_CONFIG(spi_device)                                \
 {                                                                            \
   .spi_hdl = spi_device, .int_gpio_num = 4,                                  \
 }
*/

/**
 * @brief Compute amount of SPI bit-cycles the CS should stay active after the
 * transmission to meet ENC28J60 CS Hold Time specification.
 *
 * @param clock_speed_mhz SPI Clock frequency in MHz (valid range is <1, 20>)
 * @return uint8_t
 */
static inline uint8_t enc28j60_cal_spi_cs_hold_time(int clock_speed_mhz)
{
  if (clock_speed_mhz <= 0 || clock_speed_mhz > 20) {
    return 0;
  }
  
  int temp = clock_speed_mhz * CS_HOLD_TIME_MIN_NS;
  uint8_t cs_posttrans = temp / 1000;
  
  if (temp % 1000) {
    cs_posttrans += 1;
  }

  return cs_posttrans;
}

/**
 * @brief Create ENC28J60 Ethernet MAC instance
 *
* @param[in] eth_spi: SPI configuration.
* @param[in] mac_config: Ethernet MAC configuration.
* @param[in] gpio_int_pin: interrupt pin to be used.
 *
 * @return
 *      - instance: create MAC instance successfully
 *      - NULL: create MAC instance failed because some error occurred
 */
esp_eth_mac_t* esp_eth_mac_new_enc28j60(spi_device_t *eth_spi, const eth_mac_config_t* mac_config, int gpio_int_pin);

/**
 * @brief Create a PHY instance of ENC28J60
 *
 * @param[in] config: configuration of PHY
 *
 * @return
 *      - instance: create PHY instance successfully
 *      - NULL: create PHY instance failed because some error occurred
 */
esp_eth_phy_t* esp_eth_phy_new_enc28j60(const eth_phy_config_t* config);

/**
 * @brief Get ENC28J60 silicon revision ID
 *
 * @param mac ENC28J60 MAC Handle
 * @return eth_enc28j60_rev_t
 *           - returns silicon revision ID read during initialization
 */
eth_enc28j60_rev_t emac_enc28j60_get_chip_info(esp_eth_mac_t* mac);

#ifdef __cplusplus
}
#endif
