// ============================================================
// CarHack-ESP32 Pro v5.1 — Complete
// Professional CAN Bus + OBD2 + UDS + RF + BLE + SD
// ============================================================
//
// اتصال سخت‌افزاری:
//   ESP32 GPIO5  → SN65HVD230 TXD (CAN TX)
//   ESP32 GPIO4  → SN65HVD230 RXD (CAN RX)
//   SN65HVD230 CANH → OBD2 Pin 6
//   SN65HVD230 CANL → OBD2 Pin 14
//   CC1101 → SPI (18=CS, 23=MOSI, 19=MISO, 5=CLK)
//   SD Card → SPI (13=CS, 23=MOSI, 19=MISO, 18=CLK)
//
// ============================================================

#include "config.h"
#include "can_manager.h"
#include "vehicle_db.h"
#include "uds_diag.h"
#include "obd2_diag.h"
#include "sd_logger.h"
#include "ble_remote.h"
#include "rf_rolljam.h"
#include "web_dashboard.h"

// ============================================================
// اشیاء سراسری
// ============================================================
CANManager canManager;
UDSManager udsManager;
OBD2Manager obd2Manager;
SDLogger sdLogger;
BLERemote bleRemote;
RFManager rfManager;
WebDashboard dashboard;

// ============================================================
// متغیرهای وضعیت
// ============================================================
bool system_ready = false;
unsigned long last_tester_present_ms = 0;
unsigned long last_obd2_poll_ms = 0;
unsigned long last_status_ms = 0;
unsigned long last_sd_flush_ms = 0;

// ============================================================
// Callbackها
// ============================================================
void onLog(const char* msg) {
    Serial.println(msg);
}

void onFrame(uint32_t id, bool ext, uint8_t* data, uint8_t len) {
    Serial.printf("[CAN] 0x%03lX [%d] ", id, len);
    for (int i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
    Serial.println();
    
    // ذخیره روی SD
    if (sdLogger.isLogging()) {
        sdLogger.logFrame(id, ext, data, len);
    }
}

// ============================================================
// Callback BLE — دستورات دریافتی از موبایل
// ============================================================
void onBLECommand(const char* cmd) {
    Serial.printf("[BLE] Command: %s\n", cmd);
    
    // پردازش دستورات ساده
    if (strcmp(cmd, "ping") == 0) {
        bleRemote.send("{\"pong\":true}");
    }
    else if (strcmp(cmd, "status") == 0) {
        String s = "{\"can_running\":" + String(canManager.isRunning() ? "true" : "false") + "}";
        bleRemote.send(s);
    }
    else if (strncmp(cmd, "can ", 4) == 0) {
        // فرمت: can 7E0 02 10 03
        char buf[64];
        strncpy(buf, cmd + 4, sizeof(buf) - 1);
        uint32_t can_id = strtol(strtok(buf, " "), NULL, 16);
        uint8_t data[8];
        uint8_t len = 0;
        char* token = strtok(NULL, " ");
        while (token && len < 8) {
            data[len++] = strtol(token, NULL, 16);
            token = strtok(NULL, " ");
        }
        if (canManager.sendFrame(can_id, data, len)) {
            bleRemote.send("{\"sent\":true}");
        } else {
            bleRemote.send("{\"sent\":false}");
        }
    }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n\n============================================");
    Serial.println("  CarHack-ESP32 Pro v5.1 - FINAL");
    Serial.println("  Complete CAN + OBD2 + UDS + RF + BLE + SD");
    Serial.println("============================================");
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // --- 1. CAN Bus ---
    Serial.print("[*] CAN Bus... ");
    canManager.setCallbacks(onFrame, onLog);
    if (canManager.begin(500000)) {
        Serial.println("OK (500 kbps)");
    } else {
        Serial.println("FAILED - trying auto detect...");
        canManager.autoDetectSpeed();
    }
    
    // --- 2. UDS ---
    Serial.print("[*] UDS... ");
    udsManager.setCAN(&canManager);
    Serial.println("OK");
    
    // --- 3. OBD2 ---
    Serial.print("[*] OBD2... ");
    obd2Manager.setCAN(&canManager);
    if (obd2Manager.begin()) {
        obd2Manager.readSupportedPIDs();
        obd2Manager.readVIN();
        Serial.printf("OK (VIN: %s)\n", obd2Manager.getVIN());
    } else {
        Serial.println("Waiting for CAN...");
    }
    
    // --- 4. SD Card ---
    Serial.print("[*] SD Card... ");
    if (sdLogger.begin()) {
        sdLogger.startSession();
        Serial.println("OK - Logging started");
    } else {
        Serial.println("Not found (optional)");
    }
    
    // --- 5. BLE ---
    Serial.print("[*] BLE... ");
    bleRemote.setOnCommand(onBLECommand);
    if (bleRemote.begin()) {
        Serial.printf("OK (%s)\n", BLE_DEVICE_NAME);
    } else {
        Serial.println("FAILED");
    }
    
    // --- 6. RF (CC1101) ---
    Serial.print("[*] RF CC1101... ");
    if (rfManager.begin(RF_FREQ_433)) {
        Serial.println("OK");
    } else {
        Serial.println("Not found (optional)");
    }
    
    // --- 7. Auto Detect Vehicle ---
    Serial.print("[*] Vehicle... ");
    const VehicleProfile* vehicle = canManager.autoDetectVehicle();
    if (vehicle) {
        Serial.printf("%s %s\n", vehicle->make, vehicle->model);
    } else {
        Serial.println("Generic mode");
    }
    
    // --- 8. Web Dashboard ---
    Serial.print("[*] Web Dashboard... ");
    dashboard.setCAN(&canManager);
    dashboard.setUDS(&udsManager);
    dashboard.setOBD2(&obd2Manager);
    dashboard.setSDLogger(&sdLogger);
    dashboard.setBLERemote(&bleRemote);
    dashboard.setRFManager(&rfManager);
    if (dashboard.begin()) {
        Serial.printf("OK - http://%s\n", WiFi.softAPIP().toString().c_str());
    } else {
        Serial.println("FAILED");
    }
    
    system_ready = true;
    digitalWrite(LED_BUILTIN, HIGH);
    
    Serial.println("============================================");
    Serial.println("  SYSTEM READY");
    Serial.printf("  WiFi AP: %s / %s\n", WIFI_SSID, WIFI_PASS);
    Serial.printf("  BLE: %s\n", BLE_DEVICE_NAME);
    Serial.printf("  Web: http://%s\n", WiFi.softAPIP().toString().c_str());
    Serial.println("============================================");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    if (!system_ready) return;
    
    // --- 1. CAN Frame Polling ---
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    while (canManager.receiveFrame(&id, data, &len)) {
        // پردازش خودکار در can_manager
    }
    
    // --- 2. Tester Present (UDS) ---
    if (udsManager.getCurrentSession() != 0x01 && 
        millis() - last_tester_present_ms > 2000) {
        udsManager.sendTesterPresent();
        last_tester_present_ms = millis();
    }
    
    // --- 3. OBD2 Polling (هر ۱ ثانیه) ---
    if (ENABLE_OBD2 && canManager.isRunning() &&
        millis() - last_obd2_poll_ms > 1000) {
        obd2Manager.pollAll();
        last_obd2_poll_ms = millis();
    }
    
    // --- 4. Web Dashboard ---
    dashboard.loop();
    
    // --- 5. LED Heartbeat ---
    static unsigned long last_blink = 0;
    if (millis() - last_blink > 1000) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        last_blink = millis();
    }
    
    // --- 6. Status هر ۵ ثانیه ---
    if (millis() - last_status_ms > 5000) {
        if (canManager.isRunning()) {
            Serial.printf("[STATUS] CAN: TX=%lu RX=%lu | OBD2: %d vals | SD: %lu frames | BLE: %s\n",
                canManager.getTxCount(), canManager.getRxCount(),
                obd2Manager.getValues() ? 1 : 0,
                sdLogger.getFramesLogged(),
                bleRemote.isConnected() ? "CON" : "DIS");
        }
        last_status_ms = millis();
    }
    
    delay(1);
}
