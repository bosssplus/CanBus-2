#ifndef UDS_DIAG_H
#define UDS_DIAG_H

#include "config.h"
#include "can_manager.h"

// ============================================================
// UDS Diagnostic Services (ISO 14229)
// ============================================================

// --- SID (Service Identifiers) ---
#define UDS_SID_DIAG_SESSION      0x10
#define UDS_SID_ECU_RESET         0x11
#define UDS_SID_SECURITY_ACCESS   0x27
#define UDS_SID_COMM_CONTROL      0x28
#define UDS_SID_TESTER_PRESENT    0x3E
#define UDS_SID_READ_DATA         0x22
#define UDS_SID_READ_SCALING      0x23
#define UDS_SID_READ_MEMORY       0x23
#define UDS_SID_WRITE_DATA        0x2E
#define UDS_SID_ROUTINE_CONTROL   0x31
#define UDS_SID_REQUEST_DOWNLOAD  0x34
#define UDS_SID_REQUEST_UPLOAD    0x35
#define UDS_SID_TRANSFER_DATA     0x36
#define UDS_SID_REQUEST_XFER_EXIT 0x37
#define UDS_SID_WRITE_MEMORY      0x3D
#define UDS_SID_AUTH              0x29
#define UDS_SID_IO_CONTROL        0x2F
#define UDS_SID_RESPONSE          0x40  // + SID for response

// --- Negative Response Codes ---
#define UDS_NRC_GENERAL_REJECT        0x10
#define UDS_NRC_SERVICE_NOT_SUPPORTED 0x11
#define UDS_NRC_SUBFUNC_NOT_SUPPORTED 0x12
#define UDS_NRC_INVALID_MESSAGE_LEN   0x13
#define UDS_NRC_RESPONSE_TOO_LONG     0x14
#define UDS_NRC_BUSY_REPEAT_REQUEST   0x21
#define UDS_NRC_CONDITIONS_NOT_CORRECT 0x22
#define UDS_NRC_REQUEST_SEQUENCE_ERROR 0x24
#define UDS_NRC_NO_RESPONSE           0x25
#define UDS_NRC_SECURITY_ACCESS_DENIED 0x33
#define UDS_NRC_INVALID_KEY           0x35
#define UDS_NRC_EXCEEDED_NUM_ATTEMPTS 0x36
#define UDS_NRC_REQUIRED_TIME_DELAY   0x37
#define UDS_NRC_UPLOAD_DOWNLOAD_NOT_ACCEPTED 0x70

// --- Diagnostic Sessions ---
#define UDS_SESSION_DEFAULT          0x01
#define UDS_SESSION_PROGRAMMING      0x02
#define UDS_SESSION_EXTENDED         0x03
#define UDS_SESSION_SAFETY_SYSTEM    0x04

// --- Security Access Levels ---
#define UDS_SECURITY_LEVEL_1         0x01
#define UDS_SECURITY_LEVEL_2         0x03
#define UDS_SECURITY_LEVEL_3         0x05
#define UDS_SECURITY_LEVEL_4         0x07

// ============================================================
// کلاس مدیریت UDS
// ============================================================
class UDSManager {
private:
    CANManager* can;
    uint8_t current_session;
    uint8_t current_security_level;
    uint8_t security_attempts;
    bool extended_session_active;
    bool security_unlocked;
    uint32_t session_start_ms;
    
    // --- الگوریتم Seed-Key برای PSA ---
    // از پروژه ludwig-v/psa-seedkey-algorithm
    static uint16_t psaSeedToKey(uint16_t seed) {
        uint8_t b0 = (seed >> 8) & 0xFF;
        uint8_t b1 = seed & 0xFF;
        
        // الگوریتم معکوس PSA
        uint8_t r0 = b0 ^ 0xA5;
        uint8_t r1 = b1 ^ 0x5A;
        
        r0 = (r0 << 3) | (r0 >> 5);
        r1 = (r1 >> 3) | (r1 << 5);
        
        uint8_t k0 = r0 ^ 0x55;
        uint8_t k1 = r1 ^ 0xAA;
        
        return (k0 << 8) | k1;
    }
    
public:
    UDSManager() : can(nullptr), current_session(0x01),
                   current_security_level(0),
                   security_attempts(0),
                   extended_session_active(false),
                   security_unlocked(false),
                   session_start_ms(0) {}
    
    void setCAN(CANManager* can_mgr) { can = can_mgr; }
    
    bool isSecurityUnlocked() { return security_unlocked; }
    uint8_t getCurrentSession() { return current_session; }
    
    // ============================================================
    // تغییر Diagnostic Session
    // ============================================================
    bool changeSession(uint8_t session) {
        if (!can) return false;
        
        uint8_t req[] = { UDS_SID_DIAG_SESSION, session };
        
        int ret;
        if (ENABLE_ISO_TP) {
            ret = can->sendISOTP(req, sizeof(req));
        } else {
            if (!can->sendFrame(CAN_ID_UDS_PHYS, req, sizeof(req))) return false;
            ret = ERR_OK;
        }
        
        if (ret != ERR_OK) return false;
        
        delay(50);
        
        uint8_t response[256];
        uint16_t resp_len = 0;
        
        if (ENABLE_ISO_TP) {
            // دریافت از ISO-TP
            // (ساده‌سازی شده)
        } else {
            uint32_t id;
            uint8_t len;
            if (!can->receiveFrame(&id, response, &len)) return false;
            resp_len = len;
        }
        
        if (resp_len >= 2 && response[0] == (UDS_SID_DIAG_SESSION | 0x40)) {
            current_session = session;
            extended_session_active = (session != UDS_SESSION_DEFAULT);
            session_start_ms = millis();
            return true;
        }
        
        return false;
    }
    
    // ============================================================
    // Tester Present (نگه داشتن جلسه)
    // ============================================================
    bool sendTesterPresent() {
        if (!can || !extended_session_active) return false;
        
        uint8_t req[] = { UDS_SID_TESTER_PRESENT, 0x00 };
        
        if (ENABLE_ISO_TP) {
            return (can->sendISOTP(req, sizeof(req)) == ERR_OK);
        } else {
            return can->sendFrame(CAN_ID_UDS_PHYS, req, sizeof(req));
        }
    }
    
    // ============================================================
    // Security Access - Seed-Key (الگوریتم PSA)
    // ============================================================
    bool securityAccessPSA() {
        if (!can) return false;
        
        // مرحله ۱: Extended Diagnostic Session
        if (!changeSession(UDS_SESSION_EXTENDED)) {
            return false;
        }
        
        // مرحله ۲: درخواست Seed
        uint8_t seed_req[] = { UDS_SID_SECURITY_ACCESS, UDS_SECURITY_LEVEL_1 };
        
        if (!can->sendFrame(CAN_ID_UDS_PHYS, seed_req, sizeof(seed_req))) {
            return false;
        }
        
        delay(50);
        
        uint32_t id;
        uint8_t resp[8];
        uint8_t len;
        
        if (!can->receiveFrame(&id, resp, &len)) {
            return false;
        }
        
        if (len < 4 || resp[0] != (UDS_SID_SECURITY_ACCESS | 0x40)) {
            return false;
        }
        
        // استخراج Seed (4 بایت)
        uint16_t seed = (resp[2] << 8) | resp[3];
        
        // محاسبه Key
        uint16_t key = psaSeedToKey(seed);
        
        // مرحله ۳: ارسال Key
        uint8_t key_req[] = { 
            UDS_SID_SECURITY_ACCESS, 
            UDS_SECURITY_LEVEL_1, 
            (uint8_t)(key >> 8), 
            (uint8_t)(key & 0xFF) 
        };
        
        if (!can->sendFrame(CAN_ID_UDS_PHYS, key_req, sizeof(key_req))) {
            security_attempts++;
            return false;
        }
        
        delay(50);
        
        if (!can->receiveFrame(&id, resp, &len)) {
            security_attempts++;
            return false;
        }
        
        if (resp[0] == (UDS_SID_SECURITY_ACCESS | 0x40)) {
            security_unlocked = true;
            current_security_level = UDS_SECURITY_LEVEL_1;
            security_attempts = 0;
            return true;
        } else if (resp[0] == 0x7F && resp[2] == UDS_NRC_INVALID_KEY) {
            security_attempts++;
        } else if (resp[0] == 0x7F && resp[2] == UDS_NRC_EXCEEDED_NUM_ATTEMPTS) {
            security_attempts = 99;
        }
        
        return false;
    }
    
    // ============================================================
    // Read Data By Identifier
    // ============================================================
    bool readData(uint16_t did, uint8_t* out_data, uint8_t* out_len) {
        if (!can) return false;
        
        uint8_t req[] = { UDS_SID_READ_DATA, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF) };
        
        if (!can->sendFrame(CAN_ID_UDS_PHYS, req, sizeof(req))) {
            return false;
        }
        
        delay(50);
        
        uint32_t id;
        uint8_t resp[8];
        uint8_t len;
        
        if (!can->receiveFrame(&id, resp, &len)) {
            return false;
        }
        
        if (resp[0] == (UDS_SID_READ_DATA | 0x40)) {
            uint8_t data_len = len - 3; // منهای SID + DID(2)
            if (data_len > 0 && data_len <= *out_len) {
                memcpy(out_data, resp + 3, data_len);
                *out_len = data_len;
                return true;
            }
        }
        
        return false;
    }
    
    // ============================================================
    // Write Data By Identifier
    // ============================================================
    bool writeData(uint16_t did, uint8_t* data, uint8_t len) {
        if (!can || !security_unlocked) return false;
        
        uint8_t req[8] = { UDS_SID_WRITE_DATA, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF) };
        
        if (len > 5) len = 5;
        memcpy(req + 3, data, len);
        
        if (!can->sendFrame(CAN_ID_UDS_PHYS, req, len + 3)) {
            return false;
        }
        
        delay(50);
        
        // دریافت تأیید
        uint32_t id;
        uint8_t resp[8];
        uint8_t resp_len;
        
        if (!can->receiveFrame(&id, resp, &resp_len)) {
            return false;
        }
        
        return (resp[0] == (UDS_SID_WRITE_DATA | 0x40));
    }
    
    // ============================================================
    // Routine Control
    // ============================================================
    bool startRoutine(uint16_t routine_id, uint8_t* data, uint8_t len) {
        if (!can || !security_unlocked) return false;
        
        uint8_t req[8] = { UDS_SID_ROUTINE_CONTROL, 0x01, 
                           (uint8_t)(routine_id >> 8), (uint8_t)(routine_id & 0xFF) };
        
        if (len > 4) len = 4;
        memcpy(req + 4, data, len);
        
        return can->sendFrame(CAN_ID_UDS_PHYS, req, len + 4);
    }
    
    // ============================================================
    // ECU Reset
    // ============================================================
    bool resetECU(uint8_t reset_type = 0x01) {
        if (!can) return false;
        
        uint8_t req[] = { UDS_SID_ECU_RESET, reset_type };
        return can->sendFrame(CAN_ID_UDS_PHYS, req, sizeof(req));
    }
    
    // ============================================================
    // I/O Control
    // ============================================================
    bool ioControl(uint16_t did, uint8_t control_mode, uint8_t* data, uint8_t len) {
        if (!can || !security_unlocked) return false;
        
        uint8_t req[8] = { UDS_SID_IO_CONTROL, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF), control_mode };
        
        if (len > 4) len = 4;
        memcpy(req + 4, data, len);
        
        return can->sendFrame(CAN_ID_UDS_PHYS, req, len + 4);
    }
    
    // ============================================================
    // دریافت VIN کامل
    // ============================================================
    bool readVIN(char* vin, int buf_size) {
        uint8_t data[20];
        uint8_t data_len = sizeof(data);
        
        if (readData(0xF190, data, &data_len)) {
            int idx = 0;
            for (int i = 0; i < data_len && idx < buf_size - 1; i++) {
                if (data[i] >= 0x20 && data[i] <= 0x7E) {
                    vin[idx++] = data[i];
                }
            }
            vin[idx] = '\0';
            return (idx == 17);  // VIN باید ۱۷ کاراکتر باشد
        }
        return false;
    }
    
    // ============================================================
    // JSON Status
    // ============================================================
    String toJSON() {
        String json = "{\n";
        json += "\"session\": \"0x" + String(current_session, HEX) + "\",\n";
        json += "\"extended\": " + String(extended_session_active ? "true" : "false") + ",\n";
        json += "\"security_level\": " + String(current_security_level) + ",\n";
        json += "\"unlocked\": " + String(security_unlocked ? "true" : "false") + ",\n";
        json += "\"attempts\": " + String(security_attempts) + "\n";
        json += "}";
        return json;
    }
};

#endif
