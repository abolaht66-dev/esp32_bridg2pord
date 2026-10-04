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
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define TAG "L2_SPI_MASTER"

// ==========================================
// 1. إعدادات شبكة الراوتر الرئيسي (عدلها لشبكتك)
// ==========================================
#define ROUTER_SSID     "YOUR_ROUTER_SSID"     // اكتب اسم شبكة الراوتر هنا
#define ROUTER_PASSWORD "YOUR_ROUTER_PASSWORD" // اكتب كلمة سر الراوتر هنا

// ==========================================
// 2. تعيين دبابيس توصيل الـ SPI والإشارات
// ==========================================
#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 15
#define GPIO_CS   14
#define GPIO_HANDSHAKE 2  // دبوس استقبال إشارة الجاهزية من الـ Slave

#define BUFFER_SIZE 1600

static spi_device_handle_t spi_handle;
static RingbufHandle_t tx_ringbuf; // ذاكرة موقتة للحزم القادمة من الهواء المتجهة إلى SPI

WORD_ALIGNED_ATTR static uint8_t master_tx_buf[BUFFER_SIZE];
WORD_ALIGNED_ATTR static uint8_t master_rx_buf[BUFFER_SIZE];

// --- دالة استدعاء التقاط الحزم الخام (Promiscuous RX Callback) ---
static void wifi_promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    uint16_t len = pkt->rx_ctrl.sig_len;

    if (len > 0 && len <= BUFFER_SIZE) {
        // إدخال الحزمة في الـ RingBuffer لنقلها عبر الـ SPI
        xRingbufferSend(tx_ringbuf, pkt->payload, len, pdMS_TO_TICKS(5));
    }
}

// --- مهمة إرسال واستقبال البيانات التزامنية المزدوجة عبر الـ SPI ---
static void spi_master_duplex_task(void *pvParameters) {
    size_t item_size = 0;

    while (1) {
        // 1. الانتظار حتى تكون البوردة الثانية (Slave) جاهزة ومرفوعة الإشارة على Handshake
        while (gpio_get_level(GPIO_HANDSHAKE) == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // تفريغ أوفست الذاكرة الموقتة
        memset(master_tx_buf, 0, BUFFER_SIZE);
        memset(master_rx_buf, 0, BUFFER_SIZE);

        // 2. سحب حزمة Downlink متجهة للبوردة الثانية إن وجدت
        uint8_t *item = (uint8_t *)xRingbufferReceive(tx_ringbuf, &item_size, pdMS_TO_TICKS(2));
        if (item != NULL) {
            memcpy(master_tx_buf, item, item_size);
            vRingbufferReturnItem(tx_ringbuf, (void *)item);
        } else {
            item_size = 0;
        }

        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = BUFFER_SIZE * 8; // الطول بالبت
        t.tx_buffer = master_tx_buf;
        t.rx_buffer = master_rx_buf;

        // 3. تنفيذ تبادل البيانات التزامني (Full-Duplex SPI DMA)
        esp_err_t ret = spi_device_transmit(spi_handle, &t);

        if (ret == ESP_OK) {
            // 4. إذا أرجعت البوردة الثانية حزمة Uplink قادمة من الأجهزة، نحقنها مباشرة للراوتر
            uint16_t rx_len = (master_rx_buf[12] << 8) | master_rx_buf[13]; // قراءة الطول التقريبي من الهيدر
            if (rx_len > 0 && rx_len <= BUFFER_SIZE) {
                esp_wifi_80211_tx(WIFI_IF_STA, master_rx_buf, rx_len, false);
            }
        }
    }
}

// --- تهيئة الـ SPI Master ---
static void init_spi_master(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
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

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // سرعة 10 ميجاهرتز
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 7,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));
}

// --- تهيئة الـ Wi-Fi والاتصال بالراوتر وتفعيل الالتقاط الشفاف ---
static void init_wifi_sta(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ROUTER_SSID,
            .password = ROUTER_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // الاتصال بالراوتر للإنهاء المباشر للتشفير
    ESP_LOGI(TAG, "Connecting to router: %s...", ROUTER_SSID);
    ESP_ERROR_CHECK(esp_wifi_connect());

    // تفعيل وضع الالتقاط الشفاف
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous_rx_cb));
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Master Full-Duplex Node...");

    // 1. إنشاء الـ RingBuffer لحفظ الحزم
    tx_ringbuf = xRingbufferCreate(32 * 1024, RINGBUF_TYPE_NOSPLIT);
    if (tx_ringbuf == NULL) {
        ESP_LOGE(TAG, "Failed to create RingBuffer!");
        return;
    }

    // 2. تهيئة الـ SPI والـ Wi-Fi
    init_spi_master();
    init_wifi_sta();

    // 3. إطلاق مهمة النقل المزدوج عبر الـ SPI
    xTaskCreate(spi_master_duplex_task, "spi_master_duplex_task", 4096, NULL, 5, NULL);
}
