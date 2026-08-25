#ifndef BLE_REMOTE_H
#define BLE_REMOTE_H

#include "config.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <functional>

// ============================================================
// BLE Remote Control Module
// کنترل خودرو از طریق بلوتوث Low Energy
// ============================================================

// --- Callbackها برای دستورات دریافتی از BLE ---
typedef std::function<void(const char* cmd)> BLECmdCallback;

class BLERemote {
private:
    bool initialized;
    bool connected;
    BLEServer* pServer;
    BLEService* pService;
    BLECharacteristic* pTxChar;  // ارسال به موبایل (Notify)
    BLECharacteristic* pRxChar;  // دریافت از موبایل (Write)
    BLECmdCallback on_command;
    uint32_t connected_clients;
    uint32_t last_activity_ms;
    char device_name[32];
    
    // ============================================================
    // Callback کلاس برای دریافت داده از BLE Client
    // ============================================================
    class MyCallbacks : public BLECharacteristicCallbacks {
    private:
        BLERemote* parent;
    public:
        MyCallbacks(BLERemote* p) : parent(p) {}
        void onWrite(BLECharacteristic* pCharacteristic) {
            std::string value = pCharacteristic->getValue();
            if (value.length() > 0 && parent) {
                parent->last_activity_ms = millis();
                char buf[256];
                strncpy(buf, value.c_str(), sizeof(buf) - 1);
                buf[sizeof(buf) - 1] = '\0';
                
                // حذف whitespace
                for (int i = strlen(buf) - 1; i >= 0; i--) {
                    if (buf[i] == '\n' || buf[i] == '\r' || buf[i] == ' ') buf[i] = '\0';
                    else break;
                }
                
                // Callback
                if (parent->on_command) {
                    parent->on_command(buf);
                }
            }
        }
    };
    
    // ============================================================
    // Callback برای اتصال/قطع BLE
    // ============================================================
    class MyServerCallbacks : public BLEServerCallbacks {
    private:
        BLERemote* parent;
    public:
        MyServerCallbacks(BLERemote* p) : parent(p) {}
        void onConnect(BLEServer* pServer) {
            parent->connected = true;
            parent->connected_clients++;
            parent->last_activity_ms = millis();
            Serial.printf("[BLE] Client connected (total: %lu)\n", parent->connected_clients);
            parent->send("{\"event\":\"connected\",\"device\":\"" + String(parent->device_name) + "\"}");
        }
        void onDisconnect(BLEServer* pServer) {
            parent->connected = false;
            Serial.println("[BLE] Client disconnected");
            pServer->startAdvertising();
        }
    };
    
public:
    BLERemote() : initialized(false), connected(false), pServer(nullptr),
                  pService(nullptr), pTxChar(nullptr), pRxChar(nullptr),
                  on_command(nullptr), connected_clients(0), last_activity_ms(0) {
        strcpy(device_name, BLE_DEVICE_NAME);
    }
    
    void setOnCommand(BLECmdCallback cb) { on_command = cb; }
    
    // ============================================================
    // مقداردهی اولیه BLE Server
    // ============================================================
    bool begin(const char* name = nullptr) {
        if (name) strncpy(device_name, name, sizeof(device_name) - 1);
        
        BLEDevice::init(device_name);
        BLEDevice::setPower(ESP_PWR_LVL_P7); // حداکثر قدرت
        
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(new MyServerCallbacks(this));
        
        pService = pServer->createService(BLE_SERVICE_UUID);
        
        // TX Characteristic (ارسال به موبایل)
        pTxChar = pService->createCharacteristic(
            BLE_TX_UUID,
            BLECharacteristic::PROPERTY_NOTIFY
        );
        pTxChar->addDescriptor(new BLE2902());
        
        // RX Characteristic (دریافت از موبایل)
        pRxChar = pService->createCharacteristic(
            BLE_RX_UUID,
            BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
        );
        pRxChar->setCallbacks(new MyCallbacks(this));
        
        pService->start();
        
        // Advertising
        BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
        pAdvertising->setScanResponse(true);
        pAdvertising->setMinPreferred(0x06);
        pAdvertising->setMinPreferred(0x12);
        BLEDevice::startAdvertising();
        
        initialized = true;
        last_activity_ms = millis();
        
        Serial.printf("[BLE] Advertising as: %s\n", device_name);
        return true;
    }
    
    // ============================================================
    // ارسال پیام به موبایل
    // ============================================================
    bool send(const String& data) {
        if (!initialized || !connected || !pTxChar) return false;
        pTxChar->setValue(data.c_str());
        pTxChar->notify();
        return true;
    }
    
    bool sendJSON(const String& key, const String& value) {
        String json = "{\"" + key + "\":\"" + value + "\"}";
        return send(json);
    }
    
    // ============================================================
    // ارسال CAN Frame از طریق BLE
    // ============================================================
    bool sendCANFrame(uint32_t id, uint8_t* data, uint8_t len) {
        if (!connected) return false;
        String json = "{\"can_id\":\"0x" + String(id, HEX) + "\",\"data\":\"";
        for (int i = 0; i < len; i++) {
            if (i > 0) json += " ";
            json += String(data[i], HEX);
        }
        json += "\"}";
        return send(json);
    }
    
    // ============================================================
    // ارسال OBD2 Value
    // ============================================================
    bool sendOBD2(const char* name, float value, const char* unit) {
        String json = "{\"obd2\":\"" + String(name) + "\",\"value\":" + String(value, 1) + ",\"unit\":\"" + String(unit) + "\"}";
        return send(json);
    }
    
    // ============================================================
    // وضعیت
    // ============================================================
    bool isConnected() { return connected; }
    bool isInitialized() { return initialized; }
    uint32_t getConnectedClients() { return connected_clients; }
    uint32_t getIdleTime() { return (millis() - last_activity_ms) / 1000; }
    
    String toJSON() {
        String json = "{\n";
        json += "\"initialized\": " + String(initialized ? "true" : "false") + ",\n";
        json += "\"connected\": " + String(connected ? "true" : "false") + ",\n";
        json += "\"clients\": " + String(connected_clients) + ",\n";
        json += "\"idle\": " + String(getIdleTime()) + ",\n";
        json += "\"device\": \"" + String(device_name) + "\"\n";
        json += "}";
        return json;
    }
};

#endif
