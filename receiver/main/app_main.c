#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#define TAG "FULL_DUPLEX_SLAVE"

#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 15
#define GPIO_CS   14
#define GPIO_HANDSHAKE 2

#define BUFFER_SIZE 1600

static RingbufHandle_t uplink_ringbuf; // لحزم الـ Uplink (من الأجهزة لـ SPI)

WORD_ALIGNED_ATTR static uint8_t slave_tx_buf[BUFFER_SIZE];
WORD_ALIGNED_ATTR static uint8_t slave_rx_buf[BUFFER_SIZE];

// 1. الالتقاط المباشر لبيانات الأجهزة المتصلة بالـ AP
static void wifi_ap_promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    uint16_t len = pkt->rx_ctrl.sig_len;

    if (len > 0 && len <= BUFFER_SIZE) {
        xRingbufferSend(uplink_ringbuf, pkt->payload, len, pdMS_TO_TICKS(5));
    }
}

// 2. مهمة تبادل الـ SPI للطرف المستجيب
static void spi_slave_duplex_task(void *pvParameters) {
    size_t item_size = 0;
    spi_slave_transaction_t t;

    while (1) {
        memset(slave_tx_buf, 0, BUFFER_SIZE);
        memset(slave_rx_buf, 0, BUFFER_SIZE);

        // سحب حزمة Uplink لإرسالها نحو Master
        uint8_t *item = (uint8_t *)xRingbufferReceive(uplink_ringbuf, &item_size, pdMS_TO_TICKS(2));
        if (item != NULL) {
            memcpy(slave_tx_buf, item, item_size);
            vRingbufferReturnItem(uplink_ringbuf, (void *)item);
        }

        memset(&t, 0, sizeof(t));
        t.length = BUFFER_SIZE * 8;
        t.tx_buffer = slave_tx_buf;
        t.rx_buffer = slave_rx_buf;

        // رفع إشارة الجاهزية
        gpio_set_level(GPIO_HANDSHAKE, 1);

        // التنفيذ والتزامن مع Master عبر الـ DMA
        esp_err_t ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);

        // إنزال الإشارة
        gpio_set_level(GPIO_HANDSHAKE, 0);

        if (ret == ESP_OK) {
            // حقن حزمة الـ Downlink القادمة من Master نحو الأجهزة
            uint16_t rx_len = t.trans_len / 8;
            if (rx_len > 0 && rx_len <= BUFFER_SIZE) {
                esp_wifi_80211_tx(WIFI_IF_AP, slave_rx_buf, rx_len, false);
            }
        }
    }
}

static void init_spi_slave(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&io_conf);

    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_MISO,
        .mosi_io_num = GPIO_MOSI,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BUFFER_SIZE,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 7,
    };

    ESP_ERROR_CHECK(spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
}

static void init_wifi_ap(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_L2_BRIDGE",
            .ssid_len = strlen("ESP32_L2_BRIDGE"),
            .channel = 1,
            .max_connection = 10,
            .authmode = WIFI_AUTH_OPEN,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(&wifi_ap_promiscuous_cb));
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Slave Full-Duplex Node...");
    uplink_ringbuf = xRingbufferCreate(32 * 1024, RINGBUF_TYPE_NOSPLIT);

    init_wifi_ap();
    init_spi_slave();

    xTaskCreate(spi_slave_duplex_task, "spi_slave_duplex_task", 4096, NULL, 5, NULL);
}
