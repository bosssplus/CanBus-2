#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// پیکربندی سخت‌افزاری و نرم‌افزاری پروژه CarHack-ESP32 Pro v5.1
// ============================================================

// --- پین‌های CAN Bus (ESP32 TWAI Controller) ---
#define CAN_TX_PIN          GPIO_NUM_5
#define CAN_RX_PIN          GPIO_NUM_4
#define CAN_SPEED_DEFAULT   500000
#define CAN_SPEED_ALTERNATE 250000

// --- پین‌های RF (CC1101) ---
#define RF_CS_PIN           GPIO_NUM_18
#define RF_MOSI_PIN         GPIO_NUM_23
#define RF_MISO_PIN         GPIO_NUM_19
#define RF_CLK_PIN          GPIO_NUM_5
#define RF_FREQ_433         433.92f     // فرکانس ایران/اروپا
#define RF_FREQ_315         315.0f      // فرکانس آمریکا/ژاپن

// --- SD Card ---
#define SD_CS_PIN           GPIO_NUM_13
#define SD_MOSI_PIN         GPIO_NUM_23
#define SD_MISO_PIN         GPIO_NUM_19
#define SD_CLK_PIN          GPIO_NUM_18

// --- پین‌های LED/Button ---
#define LED_BUILTIN         2
#define BTN_BOOT            0

// --- Web Dashboard ---
#define WEBSOCKET_PORT      81
#define HTTP_PORT           80
#define WEB_USERNAME        "admin"
#define WEB_PASSWORD        "c4rh4ck"

// --- BLE ---
#define BLE_DEVICE_NAME     "CarHack-Pro"
#define BLE_SERVICE_UUID    "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_TX_UUID         "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_RX_UUID         "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// --- ویژگی‌های فعال/غیرفعال ---
#define ENABLE_WIFI         true
#define ENABLE_BLE          true
#define ENABLE_SD_LOGGING   true
#define ENABLE_AUTO_LEARN   true
#define ENABLE_ISO_TP       true
#define ENABLE_DBC_PARSE    true
#define ENABLE_RF           true
#define ENABLE_OBD2         true

// --- محدودیت‌های ایمنی ---
#define MAX_SEND_RATE_MS    10
#define MAX_DIAG_SESSION_S  30
#define SAFETY_LOCKOUT      true

// --- WiFi Credentials ---
#define WIFI_SSID           "CarHack-AP"
#define WIFI_PASS           "12345678"

// --- ساختار capability flags ---
typedef enum {
    CAP_NONE          = 0,
    CAP_CAN_READ      = 1 << 0,
    CAP_CAN_WRITE     = 1 << 1,
    CAP_OBD2          = 1 << 2,
    CAP_UDS_DIAG      = 1 << 3,
    CAP_UDS_SECURITY  = 1 << 4,
    CAP_UDS_CODING    = 1 << 5,
    CAP_RF_433        = 1 << 6,
    CAP_RF_ROLLJAM    = 1 << 7,
    CAP_SNIFFER       = 1 << 8,
    CAP_AUTOLEARN     = 1 << 9,
    CAP_ISO_TP        = 1 << 10,
    CAP_LOCKOUT       = 1 << 15
} CapabilityFlag;

// --- CAN IDهای استاندارد ---
#define CAN_ID_OBD2         0x7DF
#define CAN_ID_OBD2_REPLY   0x7E8
#define CAN_ID_UDS_PHYS     0x7E0
#define CAN_ID_UDS_FUNC     0x7DF
#define CAN_ID_UDS_REPLY    0x7E8
#define CAN_ID_GLOBAL       0x000

// --- خطاها ---
typedef enum {
    ERR_OK           = 0,
    ERR_TIMEOUT      = -1,
    ERR_BUS_OFF      = -2,
    ERR_NO_REPLY     = -3,
    ERR_SECURITY     = -4,
    ERR_NOT_SUPPORT  = -5,
    ERR_PARAM        = -6
} CanError;

#endif
