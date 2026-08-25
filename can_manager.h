#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include "config.h"
#include "iso_tp.h"
#include "can_learner.h"
#include "vehicle_db.h"
#include "driver/twai.h"
#include <functional>

// ============================================================
// CAN Manager Pro — مدیریت کامل CAN Bus
// ============================================================

typedef struct {
    uint32_t id;
    bool extended;
    uint8_t data[8];
    uint8_t len;
    uint32_t timestamp_ms;
} CANFrameStruct;

// --- Callback types ---
typedef std::function<void(uint32_t id, bool ext, uint8_t* data, uint8_t len)> CANFrameCallback;
typedef std::function<void(const char* msg)> CANLogCallback;

class CANManager {
private:
    bool initialized;
    bool is_running;
    uint32_t current_speed;
    int selected_vehicle_index;
    const VehicleProfile* active_profile;
    
    CANLearner* learner;
    ISOTPTransport* iso_tp;
    
    // --- Callbacks ---
    CANFrameCallback on_frame;
    CANLogCallback on_log;
    
    // --- آمار ---
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
    uint32_t last_tx_ms;
    uint32_t last_rx_ms;
    
    // --- بافر چرخشی  ---
    static const int RING_BUF_SIZE = 256;
    CANFrameStruct ring_buffer[RING_BUF_SIZE];
    int ring_head;
    int ring_tail;
    
public:
    CANManager() : initialized(false), is_running(false), current_speed(500000),
                   selected_vehicle_index(-1), active_profile(nullptr),
                   learner(nullptr), iso_tp(nullptr),
                   tx_count(0), rx_count(0), error_count(0),
                   last_tx_ms(0), last_rx_ms(0),
                   ring_head(0), ring_tail(0) {}
    
    ~CANManager() {
        end();
        if (learner) delete learner;
        if (iso_tp) delete iso_tp;
    }
    
    void setCallbacks(CANFrameCallback frame_cb, CANLogCallback log_cb) {
        on_frame = frame_cb;
        on_log = log_cb;
    }
    
    // ============================================================
    // مقداردهی اولیه CAN Bus
    // ============================================================
    bool begin(uint32_t speed = 500000, int vehicle_index = -1) {
        if (initialized) end();
        
        current_speed = speed;
        selected_vehicle_index = vehicle_index;
        
        if (vehicle_index >= 0 && vehicle_index < VEHICLE_COUNT) {
            active_profile = &vehicle_profiles[vehicle_index];
            current_speed = active_profile->can_speed;
            log(("Selected vehicle: " + String(active_profile->make) + " " + 
                 String(active_profile->model)).c_str());
        }
        
        // --- راه‌اندازی TWAI ---
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, 
            TWAI_MODE_NORMAL);
        
        twai_timing_config_t t_config;
        if (current_speed == 500000) {
            t_config = TWAI_TIMING_CONFIG_500KBITS();
        } else if (current_speed == 250000) {
            t_config = TWAI_TIMING_CONFIG_250KBITS();
        } else if (current_speed == 125000) {
            t_config = TWAI_TIMING_CONFIG_125KBITS();
        } else {
            t_config = TWAI_TIMING_CONFIG_500KBITS();
            current_speed = 500000;
        }
        
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        
        esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
        if (err != ESP_OK) {
            log("TWAI install failed!");
            return false;
        }
        
        err = twai_start();
        if (err != ESP_OK) {
            log("TWAI start failed!");
            twai_driver_uninstall();
            return false;
        }
        
        initialized = true;
        is_running = true;
        
        // --- ISO-TP ---
        if (ENABLE_ISO_TP) {
            iso_tp = new ISOTPTransport();
            uint32_t target = CAN_ID_UDS_PHYS;
            uint32_t source = CAN_ID_UDS_REPLY;
            
            if (active_profile) {
                target = active_profile->can_ids[activity->can_id_count - 2].id;
                source = active_profile->can_ids[activity->can_id_count - 1].id;
            }
            
            iso_tp->begin(target, source, current_speed);
            iso_tp->setCallbacks(
                [this](uint32_t id, uint8_t* data, uint8_t len) -> bool {
                    return this->sendFrame(id, data, len);
                },
                [this](uint32_t* id, uint8_t* data, uint8_t* len) -> bool {
                    return this->receiveFrame(id, data, len);
                }
            );
        }
        
        // --- Auto Learner ---
        if (ENABLE_AUTO_LEARN) {
            learner = new CANLearner();
            learner->begin();
        }
        
        log("CAN Bus initialized at " + String(current_speed / 1000) + " kbps");
        return true;
    }
    
    void end() {
        if (initialized) {
            if (learner) {
                learner->stop();
            }
            twai_stop();
            twai_driver_uninstall();
            initialized = false;
            is_running = false;
            log("CAN Bus stopped");
        }
    }
    
    // ============================================================
    // ارسال فریم CAN
    // ============================================================
    bool sendFrame(uint32_t id, uint8_t* data, uint8_t len, bool extended = false) {
        if (!initialized || !is_running) return false;
        
        // محدودیت نرخ ارسال
        if (millis() - last_tx_ms < MAX_SEND_RATE_MS) {
            delay(MAX_SEND_RATE_MS - (millis() - last_tx_ms));
        }
        
        twai_message_t msg;
        msg.identifier = id;
        msg.extd = extended ? 1 : 0;
        msg.data_length_code = (len > 8) ? 8 : len;
        memcpy(msg.data, data, msg.data_length_code);
        
        esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
        if (err == ESP_OK) {
            tx_count++;
            last_tx_ms = millis();
            return true;
        } else {
            error_count++;
            if (err == ESP_ERR_TIMEOUT) {
                log("CAN TX timeout");
            } else if (err == ESP_FAIL) {
                log("CAN TX failed - bus may be off");
                // بازیابی اتوماتیک
                twai_initiate_recovery();
            }
            return false;
        }
    }
    
    // ============================================================
    // دریافت یک فریم CAN (غیرمسدودکننده)
    // ============================================================
    bool receiveFrame(uint32_t* id, uint8_t* data, uint8_t* len) {
        if (!initialized || !is_running) return false;
        
        twai_message_t msg;
        esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(0));
        
        if (err == ESP_OK) {
            *id = msg.identifier;
            *len = msg.data_length_code;
            memcpy(data, msg.data, *len);
            rx_count++;
            last_rx_ms = millis();
            
            // --- ذخیره در بافر چرخشی ---
            int next = (ring_head + 1) % RING_BUF_SIZE;
            if (next != ring_tail) {
                ring_buffer[ring_head].id = msg.identifier;
                ring_buffer[ring_head].extended = msg.extd;
                memcpy(ring_buffer[ring_head].data, msg.data, msg.data_length_code);
                ring_buffer[ring_head].len = msg.data_length_code;
                ring_buffer[ring_head].timestamp_ms = millis();
                ring_head = next;
            }
            
            // --- خوراک به Auto Learner ---
            if (learner && learner->isLearning()) {
                learner->feedFrame(msg.identifier, msg.extd, msg.data, msg.data_length_code);
            }
            
            // --- Callback ---
            if (on_frame) {
                on_frame(msg.identifier, msg.extd, msg.data, msg.data_length_code);
            }
            
            return true;
        }
        
        return false;
    }
    
    // ============================================================
    // فرستادن پیام ISO-TP
    // ============================================================
    int sendISOTP(const uint8_t* data, uint16_t len) {
        if (!iso_tp) return ERR_NOT_SUPPORT;
        return iso_tp->send(data, len);
    }
    
    // ============================================================
    // Sniffer هوشمند (ذخیره در بافر)
    // ============================================================
    int sniff(int duration_ms, bool block = false) {
        if (!initialized || !is_running) return 0;
        
        int before = ring_head;
        uint32_t start = millis();
        
        if (block) {
            while (millis() - start < (uint32_t)duration_ms) {
                uint32_t id;
                uint8_t data[8];
                uint8_t len;
                while (receiveFrame(&id, data, &len)) {
                    // فریم‌ها در receiveFrame ذخیره می‌شوند
                }
                delay(1);
            }
        } else {
            // غیرمسدودکننده
            delay(1);
        }
        
        // تعداد فریم‌های دریافتی
        int count = (ring_head - before + RING_BUF_SIZE) % RING_BUF_SIZE;
        return count;
    }
    
    // ============================================================
    // دریافت بافر Sniffer
    // ============================================================
    int getSnifferBuffer(CANFrameStruct* out, int max_count) {
        int count = 0;
        int idx = ring_tail;
        while (idx != ring_head && count < max_count) {
            out[count] = ring_buffer[idx];
            count++;
            idx = (idx + 1) % RING_BUF_SIZE;
        }
        ring_tail = idx; // پاک کردن بافر خوانده شده
        return count;
    }
    
    // ============================================================
    // Auto VIN Read از طریق UDS
    // ============================================================
    bool readVIN(char* vin_buf, int buf_size) {
        if (!initialized) return false;
        
        uint8_t uds_req[] = { 0x22, 0xF1, 0x90 };  // UDS ReadDataByIdentifier - VIN
        uint8_t response[256];
        uint16_t resp_len = 0;
        
        // ISO-TP send
        if (iso_tp) {
            int ret = iso_tp->send(uds_req, sizeof(uds_req));
            if (ret != ERR_OK) return false;
            
            delay(100);
            
            ret = iso_tp->receive(response, &resp_len);
            if (ret != ERR_OK) return false;
        } else {
            // بدون ISO-TP (Single Frame)
            if (!sendFrame(CAN_ID_UDS_PHYS, uds_req, sizeof(uds_req))) {
                return false;
            }
            
            delay(100);
            
            uint32_t id;
            uint8_t len;
            if (!receiveFrame(&id, response, &len)) {
                return false;
            }
            resp_len = len;
        }
        
        // استخراج VIN از پاسخ
        if (resp_len >= 22) {
            int vi = 0;
            for (int i = 3; i < resp_len && vi < buf_size - 1; i++) {
                if (response[i] >= 0x20 && response[i] <= 0x7E) {
                    vin_buf[vi++] = response[i];
                }
            }
            vin_buf[vi] = '\0';
            return (vi > 0);
        }
        
        return false;
    }
    
    // ============================================================
    // Auto-detect vehicle از طریق VIN
    // ============================================================
    const VehicleProfile* autoDetectVehicle() {
        char vin[18];
        if (readVIN(vin, sizeof(vin))) {
            const VehicleProfile* prof = VehicleDB::findByVIN(vin);
            if (prof) {
                active_profile = prof;
                log(("Auto-detected: " + String(prof->make) + " " + String(prof->model)).c_str());
                return prof;
            } else {
                log("VIN read but not in database: " + String(vin));
            }
        } else {
            log("Could not read VIN - trying CAN speed detection...");
        }
        return nullptr;
    }
    
    // ============================================================
    // Auto Speed Detection
    // ============================================================
    bool autoDetectSpeed() {
        uint32_t speeds[] = { 500000, 250000, 125000, 100000 };
        
        for (int s = 0; s < 4; s++) {
            log("Trying speed " + String(speeds[s] / 1000) + " kbps...");
            
            end();
            delay(100);
            
            if (begin(speeds[s], selected_vehicle_index)) {
                delay(200);
                
                uint32_t id;
                uint8_t data[8];
                uint8_t len;
                int frames = 0;
                
                // تلاش برای دریافت چند فریم
                uint32_t start = millis();
                while (millis() - start < 500) {
                    if (receiveFrame(&id, data, &len)) {
                        frames++;
                    }
                    delay(1);
                }
                
                if (frames > 0) {
                    log("Speed " + String(speeds[s] / 1000) + " kbps works! (" + 
                        String(frames) + " frames)");
                    return true;
                }
            }
        }
        
        log("Auto speed detection failed");
        return false;
    }
    
    // ============================================================
    // Auto-learn all CAN signals
    // ============================================================
    void startLearning(int duration_sec = 10) {
        if (!learner) {
            learner = new CANLearner();
        }
        learner->begin();
        log("CAN Learning started for " + String(duration_sec) + " seconds");
    }
    
    void stopLearning() {
        if (learner) {
            learner->stop();
            log("CAN Learning completed. Found " + String(learner->getSignalCount()) + " signals");
        }
    }
    
    CANLearner* getLearner() { return learner; }
    
    // ============================================================
    // JSON Status
    // ============================================================
    String toJSON() {
        String json = "{\n";
        json += "\"status\": \"" + String(is_running ? "running" : "stopped") + "\",\n";
        json += "\"speed\": " + String(current_speed / 1000) + ",\n";
        json += "\"tx\": " + String(tx_count) + ",\n";
        json += "\"rx\": " + String(rx_count) + ",\n";
        json += "\"errors\": " + String(error_count) + ",\n";
        json += "\"sniffer_buf\": " + String((ring_head - ring_tail + RING_BUF_SIZE) % RING_BUF_SIZE) + ",\n";
        json += "\"learning\": " + String(learner ? (learner->isLearning() ? "true" : "false") : "false") + ",\n";
        json += "\"vehicle\": \"" + String(active_profile ? active_profile->model : "none") + "\"\n";
        json += "}";
        return json;
    }
    
    bool isInitialized() { return initialized; }
    bool isRunning() { return is_running; }
    uint32_t getSpeed() { return current_speed; }
    uint32_t getTxCount() { return tx_count; }
    uint32_t getRxCount() { return rx_count; }
    
    const VehicleProfile* getActiveProfile() { return active_profile; }
    void setActiveProfile(const VehicleProfile* prof) { active_profile = prof; }
    
private:
    void log(const char* msg) {
        if (on_log) {
            on_log(msg);
        }
    }
};

#endif
