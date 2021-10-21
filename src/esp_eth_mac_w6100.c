// Copyright 2020 Espressif Systems (Shanghai) PTE LTD
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

// https://wizwiki.net/wiki/lib/exe/fetch.php/products:w6100:w6100_ds_v104e.pdf
// http://www.hschip.com/Private/Files/20190304152520622%E2%88%AEW6100_an_interrupt_v100e.pdf

#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include "driver/gpio.h"



#include "driver/spi_master.h"




#include "esp_attr.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "esp_system.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
//#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "hal/cpu_hal.h"
#include "w6100.h"
#include "sdkconfig.h"
#include "IO_mcp23017.h"
#include "IOT_data.h"

static const char *TAG = "w6100-mac";
#define MAC_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do {                                                                          \
        if (!(a)) {                                                               \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define W5500_SPI_LOCK_TIMEOUT_MS (50)
#define W6100_TX_MEM_SIZE (0x4000)
#define W6100_RX_MEM_SIZE (0x4000)

typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    spi_device_handle_t spi_hdl;
    SemaphoreHandle_t spi_lock;
    TaskHandle_t rx_task_hdl;
    uint32_t sw_reset_timeout_ms;
    int int_gpio_num;
    uint8_t addr[6];
    bool packets_remain;
} emac_w6100_t;

static inline bool w6100_lock(emac_w6100_t *emac)
{
   //  printf("Mac w5500 Lock\n");

    return xSemaphoreTake(emac->spi_lock, pdMS_TO_TICKS(W5500_SPI_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static inline bool w6100_unlock(emac_w6100_t *emac)
{
     //  printf("Mac w5500 Unlock\n");

    return xSemaphoreGive(emac->spi_lock) == pdTRUE;
}









static esp_err_t w6100_write(emac_w6100_t *emac, uint32_t address, const void *value, uint32_t len)
{
   // printf("Mac w5500 Write  ");

    esp_err_t ret = ESP_OK;

    spi_transaction_t trans = {
        .cmd = (address >> W5500_ADDR_OFFSET),
        .addr = ((address & 0xFFFF) | (W5500_ACCESS_MODE_WRITE << W5500_RWB_OFFSET) | W5500_SPI_OP_MODE_VDM),
        .length = 8 * len,
        .tx_buffer = value
    };
    if (w6100_lock(emac)) {
        if (spi_device_polling_transmit(emac->spi_hdl, &trans) != ESP_OK) {
            ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
            ret = ESP_FAIL;
        }
        w6100_unlock(emac);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    return ret;
}

static esp_err_t w6100_read(emac_w6100_t *emac, uint32_t address, void *value, uint32_t len)
{
    esp_err_t ret = ESP_OK;
  // printf("Mac w5500 Read  ");

    spi_transaction_t trans = {
// .flags ny 27-05-2021
        .flags = len <= 4 ? SPI_TRANS_USE_RXDATA : 0, // use direct reads for registers to prevent overwrites by 4-byte boundary writes
        .cmd = (address >> W5500_ADDR_OFFSET),
        .addr = ((address & 0xFFFF) | (W5500_ACCESS_MODE_READ << W5500_RWB_OFFSET) | W5500_SPI_OP_MODE_VDM),
        .length = 8 * len,
        .rx_buffer = value
    };
    if (w6100_lock(emac)) {
        if (spi_device_polling_transmit(emac->spi_hdl, &trans) != ESP_OK) {
            ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
            ret = ESP_FAIL;
        }
        w6100_unlock(emac);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    if ((trans.flags&SPI_TRANS_USE_RXDATA) && len <= 4) {
        memcpy(value, trans.rx_data, len);  // copy register values to output
    }
    return ret;
}













static esp_err_t w6100_send_command(emac_w6100_t *emac, uint8_t command, uint32_t timeout_ms)
{
  //   
   //    printf("Mac w5500 Send Command %04x \n",command);

    esp_err_t ret = ESP_OK;
    MAC_CHECK(w6100_write(emac, W6100_REG_SOCK_CR(0), &command, sizeof(command)) == ESP_OK, "write SCR failed", err, ESP_FAIL);
    // after W5500 accepts the command, the command register will be cleared automatically
    uint32_t to = 0;
    for (to = 0; to < timeout_ms / 10; to++) {
        MAC_CHECK(w6100_read(emac, W6100_REG_SOCK_CR(0), &command, sizeof(command)) == ESP_OK, "read SCR failed", err, ESP_FAIL);
        if (!command) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    MAC_CHECK(to < timeout_ms / 10, "send command timeout", err, ESP_ERR_TIMEOUT);

err:
    return ret;
}

static esp_err_t w6100_get_tx_free_size(emac_w6100_t *emac, uint16_t *size)
{
 //  printf("Mac w5500 get_tx_free_size\n");

    esp_err_t ret = ESP_OK;
    uint16_t free0, free1 = 0;
    // read TX_FSR register more than once, until we get the same value
    // this is a trick because we might be interrupted between reading the high/low part of the TX_FSR register (16 bits in length)
    do {
        MAC_CHECK(w6100_read(emac, W6100_REG_SOCK_TX_FSR(0), &free0, sizeof(free0)) == ESP_OK, "read TX FSR failed", err, ESP_FAIL);
        MAC_CHECK(w6100_read(emac, W6100_REG_SOCK_TX_FSR(0), &free1, sizeof(free1)) == ESP_OK, "read TX FSR failed", err, ESP_FAIL);
    } while (free0 != free1);

    *size = __builtin_bswap16(free0);
 //  printf("Mac w5500 get_tx_free_size %i\n",*size);

err:
    return ret;
}

static esp_err_t w6100_get_rx_received_size(emac_w6100_t *emac, uint16_t *size)
{
//   printf("Mac w5500 get_rx_received_size\n");

    esp_err_t ret = ESP_OK;
    uint16_t received0, received1 = 0;


    volatile uint16_t timeout = 0;
    do {
        MAC_CHECK(w6100_read(emac, W6100_REG_SOCK_RX_RSR(0), &received0, sizeof(received0)) == ESP_OK, "read RX RSR failed", err, ESP_FAIL);
        MAC_CHECK(w6100_read(emac, W6100_REG_SOCK_RX_RSR(0), &received1, sizeof(received1)) == ESP_OK, "read RX RSR failed", err, ESP_FAIL);
    } while (received0 != received1 && (timeout++ < 5000));

    if (timeout >= 5000 && (received0 != received1)) {
        ESP_LOGE(TAG, "Timeout exceeded: %d, received0 = %d, received1 = %d\n", timeout, received0, received1); 
        ret = ESP_FAIL;
        goto err;
    }

    *size = __builtin_bswap16(received0);


 //  printf("Mac w5500 get_rx_received_size %i\n",*size);
err:
    return ret;
}



static esp_err_t w6100_write_buffer(emac_w6100_t *emac, const void *buffer, uint32_t len, uint16_t offset)
{
 //  printf("Mac w5500 write_buffer   ");

    esp_err_t ret = ESP_OK;
    uint32_t remain = len;
    const uint8_t *buf = buffer;
    offset %= W6100_TX_MEM_SIZE;
    if (offset + len > W6100_TX_MEM_SIZE) {
        remain = (offset + len) % W6100_TX_MEM_SIZE;
        len = W6100_TX_MEM_SIZE - offset;
        MAC_CHECK(w6100_write(emac, W6100_MEM_SOCK_TX(0, offset), buf, len) == ESP_OK, "write TX buffer failed", err, ESP_FAIL);
        offset += len;
        buf += len;
    }
    MAC_CHECK(w6100_write(emac, W6100_MEM_SOCK_TX(0, offset), buf, remain) == ESP_OK, "write TX buffer failed", err, ESP_FAIL);

err:
    return ret;
}

static esp_err_t w6100_read_buffer(emac_w6100_t *emac, void *buffer, uint32_t len, uint16_t offset)
{
  // printf("Mac w5500 read_buffer\n");

    esp_err_t ret = ESP_OK;
    uint32_t remain = len;
    uint8_t *buf = buffer;
    offset %= W6100_RX_MEM_SIZE;
    if (offset + len > W6100_RX_MEM_SIZE) {
        remain = (offset + len) % W6100_RX_MEM_SIZE;
        len = W6100_RX_MEM_SIZE - offset;
        MAC_CHECK(w6100_read(emac, W5500_MEM_SOCK_RX(0, offset), buf, len) == ESP_OK, "read RX buffer failed", err, ESP_FAIL);
        offset += len;
        buf += len;
    }
    MAC_CHECK(w6100_read(emac, W5500_MEM_SOCK_RX(0, offset), buf, remain) == ESP_OK, "read RX buffer failed", err, ESP_FAIL);

err:
    return ret;
}

static esp_err_t w6100_set_mac_addr(emac_w6100_t *emac)
{
 //  printf("Mac w5500 set_mac_addr\n");

    esp_err_t ret = ESP_OK;
    MAC_CHECK(w6100_write(emac, W6100_REG_MAC, emac->addr, 6) == ESP_OK, "write MAC address register failed", err, ESP_FAIL);

err:
    return ret;
}




static esp_err_t w6100_reset(emac_w6100_t *emac)
{

    esp_err_t ret = ESP_OK;
    uint8_t mr = W6100_CHPLCKR_UNLOCK; // Set RST bit (auto clear)
    MAC_CHECK(w6100_write(emac, W6100_CHPLCKR, &mr, sizeof(mr)) == ESP_OK, "write W6100_CHPLCKR failed", err, ESP_FAIL);
    int count = 0;
    do{												// Wait Unlock Complete
        if(++count > 20) {							// Check retry count
            return ESP_FAIL;								// Over Limit retry count
        }
        w6100_read(emac, SYSR_W6100, &mr, sizeof(mr));
    } while ((   mr & W6100_SYSR_CHPL_LOCK) ^ W6100_SYSR_CHPL_ULOCK);	// Exit Wait Unlock Complete

    mr = W6100_MR_RST; // Set RST bit (auto clear)
    MAC_CHECK(w6100_write(emac, W6100_SYCR0, &mr, sizeof(mr)) == ESP_OK, "write MR failed", err, ESP_FAIL);

    mr = W6100_CHPLCKR_LOCK; // Set RST bit (auto clear)
    MAC_CHECK(w6100_write(emac, W6100_CHPLCKR, &mr, sizeof(mr)) == ESP_OK, "write W6100_CHPLCKR failed", err, ESP_FAIL);
    count = 0;
    do{												// Wait Unlock Complete
        if(++count > 20) {							// Check retry count
            return ESP_FAIL;								// Over Limit retry count
        }
        w6100_read(emac, SYSR_W6100, &mr, sizeof(mr));
    } while ((   mr & W6100_SYSR_CHPL_LOCK)!= W6100_SYSR_CHPL_LOCK);	// Exit Wait Unlock Complete
 //   printf("Mac w5500 software resat\n");

err:
    return ret;

}

static esp_err_t w6100_verify_id(emac_w6100_t *emac)
{
//  printf("Mac w5500 verify  %04x\n",W6100_REG_VERSIONR);

    esp_err_t ret = ESP_OK;
    uint8_t version = 0;
    MAC_CHECK(w6100_read(emac, W6100_REG_VERSIONR, &version, sizeof(version)) == ESP_OK,  "read VERSIONR failed", err, ESP_FAIL);           // 39
    // W5500 doesn't have chip ID, we just print the version number instead
    ESP_LOGI(TAG, "version=%x", version);

    if ( version==97) 
        EthernetChip=true;
    else
         ret=ESP_FAIL;

err:
    return ret;
}

static esp_err_t w6100_setup_default(emac_w6100_t *emac)
{
//   printf("Mac w5500 setup default\n");

    esp_err_t ret = ESP_OK;
    uint8_t reg_value = 16;

    // Only SOCK0 can be used as MAC RAW mode, so we give the whole buffer (16KB TX and 16KB RX) to SOCK0
    MAC_CHECK(w6100_write(emac, W6100_REG_SOCK_RXBUF_SIZE(0), &reg_value, sizeof(reg_value)) == ESP_OK, "set rx buffer size failed", err, ESP_FAIL);
    MAC_CHECK(w6100_write(emac, W6100_REG_SOCK_TXBUF_SIZE(0), &reg_value, sizeof(reg_value)) == ESP_OK, "set tx buffer size failed", err, ESP_FAIL);
    reg_value = 0;
    for (int i = 1; i < 8; i++) {
        MAC_CHECK(w6100_write(emac, W6100_REG_SOCK_RXBUF_SIZE(i), &reg_value, sizeof(reg_value)) == ESP_OK, "set rx buffer size failed", err, ESP_FAIL);
        MAC_CHECK(w6100_write(emac, W6100_REG_SOCK_TXBUF_SIZE(i), &reg_value, sizeof(reg_value)) == ESP_OK, "set tx buffer size failed", err, ESP_FAIL);
    }
// Følgende kan aktiveres eller forblive deaktiverede.....

     /* Enable ping block, disable PPPoE, WOL */
    //  reg_value = W6100_MR_PB;                            
    //  MAC_CHECK(w6100_write(emac, W6100_REG_4MR, &reg_value, sizeof(reg_value)) == ESP_OK, "write MR failed", err, ESP_FAIL);

    // reg_value=4;    // enable ARPv4 for PINGv4 Reply
    // MAC_CHECK(w6100_write(emac, W6100_REG_4MR, &reg_value, sizeof(reg_value)) == ESP_OK, "write MR failed", err, ESP_FAIL);

    // reg_value=11;   // block PINGv6 Reply Block, block TCP6 RST Packet Block, block UDP6 Port Unreachable Packet Block
    // MAC_CHECK(w6100_write(emac, W6100_REG_6MR, &reg_value, sizeof(reg_value)) == ESP_OK, "write MR failed", err, ESP_FAIL);

    //  reg_value = 2+16+32; //2 : Block - ANB & M6B bit is ignored ---- 16 IPv6 Multicast Block  ------- 32 IPv6 ALLNODE Block
    //  MAC_CHECK(w6100_write(emac, W6100_REG_NETMR, &reg_value, sizeof(reg_value)) == ESP_OK, "write W6100_REG_NETMR failed", err, ESP_FAIL);




    /* Disable interrupt for all sockets by default */
    reg_value = 0;
    MAC_CHECK(w6100_write(emac, W6100_REG_SIMR, &reg_value, sizeof(reg_value)) == ESP_OK, "write SIMR failed", err, ESP_FAIL);

    /* Enable MAC RAW mode for SOCK0, enable MAC filter, no blocking broadcast and multicast */
    reg_value = W6100_SMR_MAC_RAW ;//    W5500_SMR_MAC_FILTER skal ikke med på 6100  -- så virker den ikke;
    MAC_CHECK(w6100_write(emac, W6100_REG_SOCK_MR(0), &reg_value, sizeof(reg_value)) == ESP_OK, "write SMR failed", err, ESP_FAIL);

    /* Enable receive and send event for SOCK0 */
    reg_value = W6100_SIR_RECV | W6100_SIR_SEND;
    MAC_CHECK(w6100_write(emac, W6100_REG_SOCK_IMR(0), &reg_value, sizeof(reg_value)) == ESP_OK, "write SOCK0 IMR failed", err, ESP_FAIL);


    // reg_value = 1;      // forse ARP - FARP
    // MAC_CHECK(w6100_write(emac, W6100_REG_SOCK_MR2(0), &reg_value, sizeof(reg_value)) == ESP_OK, "write SOCK0 IMR failed", err, ESP_FAIL);


err:
    return ret;
}

static esp_err_t emac_w6100_start(esp_eth_mac_t *mac)
{
 //  printf("Mac w6100 start\n");

    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);


    uint8_t reg_value = 0;
    /* open SOCK0 */
    MAC_CHECK(w6100_send_command(emac, W6100_SCR_OPEN, 100) == ESP_OK, "issue OPEN command failed", err, ESP_FAIL);
    /* enable interrupt for SOCK0 */

    reg_value = 1<<7;
    /* Enable INTn */
    MAC_CHECK(w6100_send_command(emac, W6100_SYCR1, 100) == ESP_OK, "issue OPEN command failed", err, ESP_FAIL);

    reg_value = W6100_SIMR_SOCK0;       // enable Socket 0 Interrupt
    MAC_CHECK(w6100_write(emac, W6100_REG_SIMR, &reg_value, sizeof(reg_value)) == ESP_OK, "write SIMR failed", err, ESP_FAIL);

err:
    return ret;
}

static esp_err_t emac_w6100_stop(esp_eth_mac_t *mac)
{
  // printf("Mac w5500 stop\n");

    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    uint8_t reg_value = 0;
    /* disable interrupt */
    MAC_CHECK(w6100_write(emac, W6100_REG_SIMR, &reg_value, sizeof(reg_value)) == ESP_OK, "write SIMR failed", err, ESP_FAIL);
    /* close SOCK0 */
    MAC_CHECK(w6100_send_command(emac, W6100_SCR_CLOSE, 100) == ESP_OK, "issue CLOSE command failed", err, ESP_FAIL);

err:
    return ret;
}

IRAM_ATTR static void w6100_isr_handler(void *arg)
{
    emac_w6100_t *emac = (emac_w6100_t *)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    /* notify w5500 task */

    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    if (high_task_wakeup != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

static void emac_w6100_task(void *arg)
{
    emac_w6100_t *emac = (emac_w6100_t *)arg;
    uint8_t status = 0;
    uint8_t *buffer = NULL;
    uint32_t length = 0;
    while (1) {
        // block indefinitely until some task notifies me
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* read interrupt status */


        
        w6100_read(emac, W6100_REG_SOCK_IR(0), &status, sizeof(status));
        /* packet received */
        if (status & W6100_SIR_RECV) {
          //  printf("Der kom et recv. interrupt\n");
  
            status = W6100_SIR_RECV;            // clear interrupt status
            w6100_write(emac, W6100_REG_SOCK_IRCLR(0), &status, sizeof(status));
            do {
                length = ETH_MAX_PACKET_SIZE;
                buffer = heap_caps_malloc(length, MALLOC_CAP_DMA);
                if (!buffer) {
                    ESP_LOGE(TAG, "no mem for receive buffer");
                    break;
                } else if (emac->parent.receive(&emac->parent, buffer, &length) == ESP_OK) {
                    /* pass the buffer to stack (e.g. TCP/IP layer) */
                    if (length) {
                        emac->eth->stack_input(emac->eth, buffer, length);
                    } else {
                        free(buffer);
                    }
                } else {
                    free(buffer);
                }
      //          printf("packet remain %i ",emac->packets_remain);
            } while (emac->packets_remain);
        }
    }
    vTaskDelete(NULL);
}

static esp_err_t emac_w6100_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *eth)
{
//printf("Mac w5500 set mediator\n");

    esp_err_t ret = ESP_OK;
    MAC_CHECK(eth, "can't set mac's mediator to null", err, ESP_ERR_INVALID_ARG);
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    emac->eth = eth;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_w6100_write_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);


    uint8_t mr = W6100_PHYLCKR_UNLOCK; // Unlock
    MAC_CHECK(w6100_write(emac, W6100_PHYLCKR, &mr, sizeof(mr)) == ESP_OK, "write W6100_PHYLCKR failed", err, ESP_FAIL);

    int count = 0;
    do{												// Wait Unlock Complete
        if(++count > 20) {							// Check retry count
            return ESP_FAIL;								// Over Limit retry count
        }
        w6100_read(emac, SYSR_W6100, &mr, sizeof(mr));

    } while ((   mr & W6100_SYSR_PHYL_LOCK) ^ W6100_SYSR_PHYL_ULOCK);	// Exit Wait Unlock Complete

    MAC_CHECK(phy_reg == W6100_REG_PHYCR0 || phy_reg == W6100_REG_PHYCR1, "wrong PHY register", err, ESP_FAIL);
    MAC_CHECK(w6100_write(emac, phy_reg, &reg_value, sizeof(uint8_t)) == ESP_OK, "write PHY register failed", err, ESP_FAIL);
//    printf("Mac w5500 write phy reg %02x \n",(uint8_t) reg_value);

    mr = W6100_PHYLCKR_LOCK; // lock
    MAC_CHECK(w6100_write(emac, W6100_PHYLCKR, &mr, sizeof(mr)) == ESP_OK, "write W6100_PHYLCKR failed", err, ESP_FAIL);

    count = 0;
    do{												// Wait Unlock Complete
        if(++count > 20) {							// Check retry count
            return ESP_FAIL;								// Over Limit retry count
        }
        w6100_read(emac, SYSR_W6100, &mr, sizeof(mr));
    } while ((   mr & W6100_SYSR_PHYL_LOCK)!= W6100_SYSR_PHYL_LOCK);	// Exit Wait Unlock Complete
 //   printf("Mac PHY locked & written\n");
err:
    return ret;
}



static esp_err_t emac_w6100_read_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value)
{

    esp_err_t ret = ESP_OK;

    MAC_CHECK(reg_value, "can't set reg_value to null", err, ESP_ERR_INVALID_ARG);
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    // PHY register and MAC registers are mixed together in W5500
    // The only PHY register is PHYCFGR
    MAC_CHECK(phy_reg == W6100_REG_PHYSR, "wrong PHY register", err, ESP_FAIL);
    MAC_CHECK(w6100_read(emac, W6100_REG_PHYSR, reg_value, sizeof(uint8_t)) == ESP_OK, "read PHY register failed", err, ESP_FAIL);
   // printf("Mac w5500 read phy reg %02x \n", (uint8_t)* reg_value);
    if (((uint8_t)* reg_value & 0x80) == 0x80) NoCable=true;
err:
    return ret;
}

static esp_err_t emac_w6100_set_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
 //  printf("Mac w5500 set addr\n");

    esp_err_t ret = ESP_OK;
    MAC_CHECK(addr, "invalid argument", err, ESP_ERR_INVALID_ARG);
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    memcpy(emac->addr, addr, 6);
    MAC_CHECK(w6100_set_mac_addr(emac) == ESP_OK, "set mac address failed", err, ESP_FAIL);

err:
    return ret;
}

static esp_err_t emac_w6100_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
  // printf("Mac w5500 get addr\n");

    esp_err_t ret = ESP_OK;
    MAC_CHECK(addr, "invalid argument", err, ESP_ERR_INVALID_ARG);
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    memcpy(addr, emac->addr, 6);

err:
    return ret;
}

static esp_err_t emac_w6100_set_link(esp_eth_mac_t *mac, eth_link_t link)
{
  // printf("Mac w5500 set link\n");

    esp_err_t ret = ESP_OK;
    switch (link) {
    case ETH_LINK_UP:
        ESP_LOGD(TAG, "link is up");
        MAC_CHECK(mac->start(mac) == ESP_OK, "w5500 start failed", err, ESP_FAIL);
        break;
    case ETH_LINK_DOWN:
        ESP_LOGD(TAG, "link is down");
        MAC_CHECK(mac->stop(mac) == ESP_OK, "w5500 stop failed", err, ESP_FAIL);
        break;
    default:
        MAC_CHECK(false, "unknown link status", err, ESP_ERR_INVALID_ARG);
        break;
    }

err:
    return ret;
}

static esp_err_t emac_w6100_set_speed(esp_eth_mac_t *mac, eth_speed_t speed)
{
//   printf("Mac w5500 set speed\n");

    esp_err_t ret = ESP_OK;
    switch (speed) {
    case ETH_SPEED_10M:
        ESP_LOGD(TAG, "working in 10Mbps");
        break;
    case ETH_SPEED_100M:
        ESP_LOGD(TAG, "working in 100Mbps");
        break;
    default:
        MAC_CHECK(false, "unknown speed", err, ESP_ERR_INVALID_ARG);
        break;
    }

err:
    return ret;
}

static esp_err_t emac_w6100_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex)
{
 // printf("Mac w5500 set duplex\n");

    esp_err_t ret = ESP_OK;
    switch (duplex) {
    case ETH_DUPLEX_HALF:
        ESP_LOGD(TAG, "working in half duplex");
        break;
    case ETH_DUPLEX_FULL:
        ESP_LOGD(TAG, "working in full duplex");
        break;
    default:
        MAC_CHECK(false, "unknown duplex", err, ESP_ERR_INVALID_ARG);
        break;
    }

err:
    return ret;
}

static esp_err_t emac_w6100_set_promiscuous(esp_eth_mac_t *mac, bool enable)
{
  // printf("Mac w5500 set promiscuous\n");

    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    uint8_t smr = 0;
    MAC_CHECK(w6100_read(emac, W6100_REG_SOCK_MR(0), &smr, sizeof(smr)) == ESP_OK, "read SMR failed", err, ESP_FAIL);
    if (enable) {
        smr &= ~W6100_SMR_MAC_FILTER;
    } else {
        smr |= W6100_SMR_MAC_FILTER;
    }
    MAC_CHECK(w6100_write(emac, W6100_REG_SOCK_MR(0), &smr, sizeof(smr)) == ESP_OK, "write SMR failed", err, ESP_FAIL);

err:
    return ret;
}

static esp_err_t emac_w6100_enable_flow_ctrl(esp_eth_mac_t *mac, bool enable)
{
  //  printf("Mac w5500 enable flow ctrl\n");
    /* w5500 doesn't support flow control function, so accept any value */
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t emac_w6100_set_peer_pause_ability(esp_eth_mac_t *mac, uint32_t ability)
{
    // printf("Mac w5500 set peer pause ability\n");
    /* w5500 doesn't suppport PAUSE function, so accept any value */
    return ESP_ERR_NOT_SUPPORTED;
}



static inline bool is_w5500_sane_for_rxtx(esp_eth_mac_t *emac)
{
    uint8_t phycfg;
    /* phy is ok for rx and tx operations if bits RST and LNK are set (no link down, no reset) */
    // if (w6100_read(emac, W6100_REG_PHYCFGR, &phycfg, 1) == ESP_OK && (phycfg & 0x8001)) {
    //     return true;
    // }

   if (w6100_read(emac, W6100_REG_PHYSR, &phycfg, sizeof(uint8_t)) == ESP_OK && (phycfg & 0x01)){
  //     printf("Phy status %i\n",phycfg);
       return true;
   }

   return false;
}



static esp_err_t emac_w6100_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
 //   printf("T");
//     for (int i=1;i<length;i++){
//         printf("%02x ",buf[i]);
//     }
//     printf("\n");
//     printf("\n");
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    uint16_t offset = 0;

    // check if there're free memory to store this packet
    uint16_t free_size = 0;
    MAC_CHECK(w6100_get_tx_free_size(emac, &free_size) == ESP_OK, "get free size failed", err, ESP_FAIL);
    MAC_CHECK(length <= free_size, "free size (%d) < send length (%d)", err, ESP_FAIL, free_size, length);
    // get current write pointer
    MAC_CHECK(w6100_read(emac, W6100_REG_SOCK_TX_WR(0), &offset, sizeof(offset)) == ESP_OK, "read TX WR failed", err, ESP_FAIL);
    offset = __builtin_bswap16(offset);
    // copy data to tx memory
    MAC_CHECK(w6100_write_buffer(emac, buf, length, offset) == ESP_OK, "write frame failed", err, ESP_FAIL);
    // update write pointer
    offset += length;
    offset = __builtin_bswap16(offset);
    MAC_CHECK(w6100_write(emac, W6100_REG_SOCK_TX_WR(0), &offset, sizeof(offset)) == ESP_OK, "write TX WR failed", err, ESP_FAIL);
    // issue SEND command
    MAC_CHECK(w6100_send_command(emac, W6100_SCR_SEND, 100) == ESP_OK, "issue SEND command failed", err, ESP_FAIL);
    // pooling the TX done event



    // uint8_t status = 0;
    // volatile int32_t count = 0;
    // do {
    //     MAC_CHECK(w6100_read(emac, W6100_REG_SOCK_IR(0), &status, sizeof(status)) == ESP_OK, "read SOCK0 IR failed", err, ESP_FAIL);
    //     }
    //     while (!(status & W6100_SIR_SEND) && (count++ < 5000));         //Sn_IR(SENDOK) Interrupt   This is issued when SEND command is completed.
        
                
    //     if (count >= 5000 && !(status & W6100_SIR_SEND))
    //     {
    //          ESP_LOGE(TAG, "Failed cnt %d, status %d\r\n", count, status);       // HuyTV

    //         ret = ESP_FAIL;
    //         goto err;
    //     }

    // ESP_LOGE(TAG, "count %d\r\n", count);
    // clear the event bit

  // pooling the TX done event
    int retry = 0;
    uint8_t status = 0;
    while (!(status & W6100_SIR_SEND)) {
     
        MAC_CHECK(w6100_read(emac, W6100_REG_SOCK_IR(0), &status, sizeof(status)) == ESP_OK, "read SOCK0 IR failed", err, ESP_FAIL);
     
//        ESP_GOTO_ON_ERROR(w5500_read(emac, W5500_REG_SOCK_IR(0), &status, sizeof(status)), err, TAG, "read SOCK0 IR failed");

        if ((retry++ > 3 && !is_w5500_sane_for_rxtx(emac)) || retry > 10) {
            return ESP_FAIL;
        }
    }






    status  = W6100_SIR_SEND;
    MAC_CHECK(w6100_write(emac, W6100_REG_SOCK_IRCLR(0), &status, sizeof(status)) == ESP_OK, "write SOCK0 IR failed", err, ESP_FAIL);
    // er et RO RO RO RO RO
err:
    return ret;
}





static esp_err_t emac_w6100_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{

    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);

    uint16_t offset = 0;
    uint16_t rx_len = 0;
    uint16_t remain_bytes = 0;
    emac->packets_remain = false;

    w6100_get_rx_received_size(emac, &remain_bytes);
  //  printf("R");

    if (remain_bytes) {
        // get current read pointer
        MAC_CHECK(w6100_read(emac, W6100_REG_SOCK_RX_RD(0), &offset, sizeof(offset)) == ESP_OK, "read RX RD failed", err, ESP_FAIL);
        offset = __builtin_bswap16(offset);
        // read head first
        MAC_CHECK(w6100_read_buffer(emac, &rx_len, sizeof(rx_len), offset) == ESP_OK, "read frame header failed", err, ESP_FAIL);
        rx_len = __builtin_bswap16(rx_len) - 2; // data size includes 2 bytes of header
        offset += 2;
        // read the payload
        MAC_CHECK(w6100_read_buffer(emac, buf, rx_len, offset) == ESP_OK, "read payload failed, len=%d, offset=%d", err, ESP_FAIL, rx_len, offset);
        offset += rx_len;
        // update read pointer
        offset = __builtin_bswap16(offset);
        MAC_CHECK(w6100_write(emac, W6100_REG_SOCK_RX_RD(0), &offset, sizeof(offset)) == ESP_OK, "write RX RD failed", err, ESP_FAIL);
        /* issue RECV command */
        MAC_CHECK(w6100_send_command(emac, W6100_SCR_RECV, 100) == ESP_OK, "issue RECV command failed", err, ESP_FAIL);

        // check if there're more data need to process
        remain_bytes -= rx_len + 2;
        emac->packets_remain = remain_bytes > 0;
 

    }

    *length = rx_len;

    // printf(" To buf recieve %i \n",(int)*length);
    // for (int i=1;i<(int)*length;i++){
    //     printf("%02x ",buf[i]);
    // }
    // printf("\n");
err:
    return ret;
}

static esp_err_t emac_w6100_init(esp_eth_mac_t *mac)
{
    uint8_t PhyStatus=0;

  //  printf("Mac w5500 init\n");

    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    //esp_rom_gpio_pad_select_gpio(emac->int_gpio_num);// Mads
    gpio_set_direction(emac->int_gpio_num, GPIO_MODE_INPUT);
    gpio_set_pull_mode(emac->int_gpio_num, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(emac->int_gpio_num, GPIO_INTR_NEGEDGE); // active low
    gpio_intr_enable(emac->int_gpio_num);
    gpio_isr_handler_add(emac->int_gpio_num, w6100_isr_handler, emac);
    MAC_CHECK(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL) == ESP_OK, "lowlevel init failed", err, ESP_FAIL);
    /* reset w5500 */
    MAC_CHECK(w6100_reset(emac) == ESP_OK, "reset w5500 failed", err, ESP_FAIL);
    /* verify chip id */
    MAC_CHECK(w6100_verify_id(emac) == ESP_OK, "vefiry chip ID failed", err, ESP_FAIL);
    /* default setup of internal registers */
    MAC_CHECK(w6100_setup_default(emac) == ESP_OK, "w5500 default setup failed", err, ESP_FAIL);




    return ESP_OK;


err:
    gpio_isr_handler_remove(emac->int_gpio_num);
    gpio_reset_pin(emac->int_gpio_num);
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ret;
}

static esp_err_t emac_w6100_deinit(esp_eth_mac_t *mac)
{
 //   printf("MAC Deinit\n");
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    mac->stop(mac);
    gpio_isr_handler_remove(emac->int_gpio_num);
    gpio_reset_pin(emac->int_gpio_num);
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ESP_OK;
}

static esp_err_t emac_w6100_del(esp_eth_mac_t *mac)
{
  // printf("Mac w5500 del\n");

    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    vTaskDelete(emac->rx_task_hdl);
    vSemaphoreDelete(emac->spi_lock);
    free(emac);
    return ESP_OK;
}

esp_eth_mac_t *esp_eth_mac_new_w6100(const eth_w5500_config_t *w5500_config, const eth_mac_config_t *mac_config)
{
   //    printf("Mac w6100 _new_w6100\n");

    esp_eth_mac_t *ret = NULL;
    emac_w6100_t *emac = NULL;
    MAC_CHECK(w5500_config && mac_config, "invalid argument", err, NULL);
    emac = calloc(1, sizeof(emac_w6100_t));
    MAC_CHECK(emac, "no mem for MAC instance", err, NULL);
    /* w5500 driver is interrupt driven */
    MAC_CHECK(w5500_config->int_gpio_num >= 0, "invalid interrupt gpio number", err, NULL);
    /* bind methods and attributes */
    emac->sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
    emac->int_gpio_num = w5500_config->int_gpio_num;
    emac->spi_hdl = w5500_config->spi_hdl;
    emac->parent.set_mediator = emac_w6100_set_mediator;
    emac->parent.init = emac_w6100_init;
    emac->parent.deinit = emac_w6100_deinit;
    emac->parent.start = emac_w6100_start;
    emac->parent.stop = emac_w6100_stop;
    emac->parent.del = emac_w6100_del;







    emac->parent.write_phy_reg = emac_w6100_write_phy_reg;
    emac->parent.read_phy_reg = emac_w6100_read_phy_reg;
    emac->parent.set_addr = emac_w6100_set_addr;
    emac->parent.get_addr = emac_w6100_get_addr;
    emac->parent.set_speed = emac_w6100_set_speed;
    emac->parent.set_duplex = emac_w6100_set_duplex;
    emac->parent.set_link = emac_w6100_set_link;
    emac->parent.set_promiscuous = emac_w6100_set_promiscuous;
    emac->parent.set_peer_pause_ability = emac_w6100_set_peer_pause_ability;//  Mads
    emac->parent.enable_flow_ctrl = emac_w6100_enable_flow_ctrl;       //   Mads
    emac->parent.transmit = emac_w6100_transmit;
    emac->parent.receive = emac_w6100_receive;
    /* create mutex */
    emac->spi_lock = xSemaphoreCreateMutex();
    MAC_CHECK(emac->spi_lock, "create lock failed", err, NULL);
    /* create w5500 task */
    BaseType_t core_num = tskNO_AFFINITY;
    if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
        core_num = cpu_hal_get_core_id();
  //      printf("W6100 core hall %i\n",core_num);

    } else {
        core_num=0;
    }
//printf("W6100 core %i\n",core_num);
    BaseType_t xReturned = xTaskCreatePinnedToCore(emac_w6100_task, "w5500_tsk", mac_config->rx_task_stack_size, emac,     mac_config->rx_task_prio, &emac->rx_task_hdl, core_num);
    MAC_CHECK(xReturned == pdPASS, "create w5500 task failed", err, NULL);
    return &(emac->parent);

err:
    if (emac) {
        if (emac->rx_task_hdl) {
            vTaskDelete(emac->rx_task_hdl);
        }
        if (emac->spi_lock) {
            vSemaphoreDelete(emac->spi_lock);
        }
        free(emac);
    }
    return ret;
}
