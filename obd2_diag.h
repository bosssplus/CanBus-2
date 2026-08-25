#ifndef OBD2_DIAG_H
#define OBD2_DIAG_H

#include "config.h"
#include "can_manager.h"
#include <vector>

// ============================================================
// OBD2 Diagnostic Module — ISO 15031 / SAE J1979
// ============================================================

// --- PIDهای استاندارد OBD2 Mode 1 ---
#define OBD2_MODE_CURRENT_DATA     0x01
#define OBD2_MODE_FREEZE_FRAME     0x02
#define OBD2_MODE_DTC              0x03
#define OBD2_MODE_CLEAR_DTC        0x04
#define OBD2_MODE_O2_MONITOR       0x05
#define OBD2_MODE_ONBOARD_MONITOR  0x06
#define OBD2_MODE_DTC_DETAIL       0x07
#define OBD2_MODE_CONTROL          0x08
#define OBD2_MODE_VEHICLE_INFO     0x09

// --- PIDs ---
#define PID_ENGINE_RPM         0x0C
#define PID_VEHICLE_SPEED      0x0D
#define PID_COOLANT_TEMP       0x05
#define PID_THROTTLE_POS       0x11
#define PID_ENGINE_LOAD        0x04
#define PID_FUEL_LEVEL         0x2F
#define PID_INTAKE_TEMP        0x0F
#define PID_MAF_SENSOR         0x10
#define PID_FUEL_PRESSURE      0x0A
#define PID_TIMING_ADVANCE     0x0E
#define PID_O2_VOLTAGE         0x14
#define PID_DISTANCE           0x31
#define PID_VIN                0x02
#define PID_CALIBRATION_ID     0x04
#define PID_CVN                0x06

// --- DTC Severity ---
#define DTC_SEVERITY_NONE      0
#define DTC_SEVERITY_CHECK     1   // Check Engine
#define DTC_SEVERITY_WARN      2   // Warning
#define DTC_SEVERITY_CRITICAL  3  // Critical

// --- یک مقدار OBD2 ---
typedef struct {
    uint8_t  pid;
    char     name[24];
    float    value;
    char     unit[8];
    bool     valid;
} OBD2Value;

// --- یک کد خطا (DTC) ---
typedef struct {
    char     code[8];      // P0101, P0300, ...
    char     description[64];
    uint8_t  severity;     // 0-3
    bool     stored;       // true=stored, false=pending
} DTCode;

class OBD2Manager {
private:
    CANManager* can;
    bool initialized;
    uint32_t last_poll_ms;
    uint16_t supported_pids[4]; // بیت‌مپ PIDهای پشتیبانی‌شده
    
    // --- OBD2 Values cache ---
    OBD2Value values[20];
    int value_count;
    bool vin_read_done;
    char vin[18];
    
    // --- DTC list ---
    std::vector<DTCode> dtc_list;
    
public:
    OBD2Manager() : can(nullptr), initialized(false), last_poll_ms(0),
                    value_count(0), vin_read_done(false) {
        memset(vin, 0, sizeof(vin));
        memset(supported_pids, 0, sizeof(supported_pids));
    }
    
    void setCAN(CANManager* c) { can = c; }
    
    bool begin() {
        if (!can || !can->isRunning()) return false;
        initialized = true;
        last_poll_ms = millis();
        return true;
    }
    
    // ============================================================
    // ارسال درخواست OBD2 و دریافت پاسخ
    // ============================================================
    bool sendRequest(uint8_t mode, uint8_t pid, uint8_t* resp, uint8_t* resp_len) {
        if (!can || !can->isRunning()) return false;
        
        uint8_t req[] = { mode, pid };
        
        if (!can->sendFrame(CAN_ID_OBD2, req, 2)) return false;
        delay(50);
        
        uint32_t id;
        uint8_t len;
        // تلاش ۳ بار برای دریافت پاسخ
        for (int attempt = 0; attempt < 3; attempt++) {
            if (can->receiveFrame(&id, resp, &len)) {
                if (id == CAN_ID_OBD2_REPLY && len >= 2) {
                    *resp_len = len;
                    return true;
                }
            }
            delay(20);
        }
        return false;
    }
    
    // ============================================================
    // دریافت Supported PIDs
    // ============================================================
    bool readSupportedPIDs() {
        for (int i = 0; i < 4; i++) {
            uint8_t pid_to_query = 0x01 + (i * 0x20);
            uint8_t resp[8], len;
            
            if (sendRequest(OBD2_MODE_CURRENT_DATA, pid_to_query, resp, &len)) {
                if (len >= 6) {
                    supported_pids[i] = (resp[2] << 8) | resp[3];
                }
            }
            delay(50);
        }
        return true;
    }
    
    // ============================================================
    // بررسی پشتیبانی یک PID
    // ============================================================
    bool isPIDSupported(uint8_t pid) {
        if (pid == 0) return false;
        int index = (pid - 1) / 32;
        int bit = (pid - 1) % 32;
        if (index >= 4) return false;
        return (supported_pids[index] >> bit) & 1;
    }
    
    // ============================================================
    // دریافت مقدار یک PID
    // ============================================================
    float readPID(uint8_t pid, bool force = false) {
        uint8_t resp[8], len;
        
        if (!sendRequest(OBD2_MODE_CURRENT_DATA, pid, resp, &len)) {
            return -1.0;
        }
        
        if (len < 3) return -1.0;
        
        // --- تبدیل داده به مقدار قابل خواندن ---
        switch (pid) {
            case PID_ENGINE_RPM: {
                if (len < 4) return -1.0;
                uint16_t raw = (resp[2] << 8) | resp[3];
                return raw / 4.0;
            }
            case PID_VEHICLE_SPEED: {
                return resp[2]; // km/h
            }
            case PID_COOLANT_TEMP: {
                return resp[2] - 40.0; // درجه سانتیگراد
            }
            case PID_THROTTLE_POS: {
                return (resp[2] * 100.0) / 255.0; // درصد
            }
            case PID_ENGINE_LOAD: {
                return (resp[2] * 100.0) / 255.0; // درصد
            }
            case PID_FUEL_LEVEL: {
                return (resp[2] * 100.0) / 255.0; // درصد
            }
            case PID_INTAKE_TEMP: {
                return resp[2] - 40.0;
            }
            case PID_FUEL_PRESSURE: {
                return resp[2] * 3.0; // kPa
            }
            case PID_MAF_SENSOR: {
                if (len < 4) return -1.0;
                uint16_t raw = (resp[2] << 8) | resp[3];
                return raw / 100.0; // g/s
            }
            case PID_TIMING_ADVANCE: {
                return (resp[2] / 2.0) - 64.0; // درجه
            }
            case PID_DISTANCE: {
                if (len < 4) return -1.0;
                uint32_t raw = (resp[2] << 16) | (resp[3] << 8) | resp[4];
                return raw; // km
            }
            default:
                return resp[2];
        }
    }
    
    // ============================================================
    // خواندن VIN
    // ============================================================
    bool readVIN() {
        // Mode 9, PID 2
        uint8_t resp[8], len;
        
        if (!sendRequest(OBD2_MODE_VEHICLE_INFO, PID_VIN, resp, &len)) {
            return false;
        }
        
        // VIN معمولاً در پاسخ‌های متوالی می‌آید
        int vin_idx = 0;
        for (int seq = 0; seq < 4; seq++) {
            if (sendRequest(OBD2_MODE_VEHICLE_INFO, PID_VIN, resp, &len)) {
                if (len >= 4) {
                    for (int i = 3; i < len && vin_idx < 17; i++) {
                        if (resp[i] >= 0x20 && resp[i] <= 0x7E) {
                            vin[vin_idx++] = resp[i];
                        }
                    }
                }
            }
            delay(30);
        }
        vin[vin_idx] = '\0';
        vin_read_done = (vin_idx == 17);
        return vin_read_done;
    }
    
    // ============================================================
    // خواندن DTCها (Diagnostic Trouble Codes)
    // ============================================================
    int readDTCs() {
        dtc_list.clear();
        
        uint8_t resp[8], len;
        if (!sendRequest(OBD2_MODE_DTC, 0x00, resp, &len)) {
            return 0;
        }
        
        if (len < 4) return 0;
        
        // 2 بایت اول تعداد DTCها
        uint16_t dtc_count = (resp[0] << 8) | resp[1];
        
        // بقیه بایت‌ها کدهای خطا
        int idx = 2;
        while (idx + 1 < len && dtc_list.size() < dtc_count) {
            uint8_t hi = resp[idx];
            uint8_t lo = resp[idx + 1];
            
            char prefix;
            switch ((hi >> 6) & 0x03) {
                case 0: prefix = 'P'; break; // Powertrain
                case 1: prefix = 'C'; break; // Chassis
                case 2: prefix = 'B'; break; // Body
                case 3: prefix = 'U'; break; // Network
            }
            
            DTCode dtc;
            snprintf(dtc.code, sizeof(dtc.code), "%c%02d%02X%02X",
                     prefix, (hi >> 4) & 0x03, ((hi & 0x0F) << 4) | (lo >> 4), lo & 0x0F);
            
            // Severity
            uint8_t severity_bits = (hi >> 4) & 0x03;
            if (severity_bits >= 3) dtc.severity = DTC_SEVERITY_CRITICAL;
            else if (severity_bits >= 2) dtc.severity = DTC_SEVERITY_WARN;
            else if (severity_bits >= 1) dtc.severity = DTC_SEVERITY_CHECK;
            else dtc.severity = DTC_SEVERITY_NONE;
            
            dtc.stored = true;
            strcpy(dtc.description, dtcLookup(dtc.code));
            
            dtc_list.push_back(dtc);
            idx += 2;
        }
        
        return dtc_list.size();
    }
    
    // ============================================================
    // پاک کردن DTCها
    // ============================================================
    bool clearDTCs() {
        uint8_t req[] = { OBD2_MODE_CLEAR_DTC, 0x00 };
        if (!can->sendFrame(CAN_ID_OBD2, req, 2)) return false;
        delay(100);
        dtc_list.clear();
        return true;
    }
    
    // ============================================================
    // به‌روزرسانی خودکار همه مقادیر
    // ============================================================
    void pollAll() {
        if (!initialized || !can || !can->isRunning()) return;
        
        uint8_t pids_to_read[] = {
            PID_ENGINE_RPM, PID_VEHICLE_SPEED, PID_COOLANT_TEMP,
            PID_THROTTLE_POS, PID_ENGINE_LOAD, PID_FUEL_LEVEL,
            PID_INTAKE_TEMP, PID_FUEL_PRESSURE, PID_MAF_SENSOR
        };
        int num_pids = sizeof(pids_to_read) / sizeof(pids_to_read[0]);
        
        value_count = 0;
        for (int i = 0; i < num_pids; i++) {
            float val = readPID(pids_to_read[i]);
            if (val >= 0) {
                values[value_count].pid = pids_to_read[i];
                values[value_count].value = val;
                values[value_count].valid = true;
                
                switch (pids_to_read[i]) {
                    case PID_ENGINE_RPM:
                        strcpy(values[value_count].name, "Engine RPM");
                        strcpy(values[value_count].unit, "rpm");
                        break;
                    case PID_VEHICLE_SPEED:
                        strcpy(values[value_count].name, "Speed");
                        strcpy(values[value_count].unit, "km/h");
                        break;
                    case PID_COOLANT_TEMP:
                        strcpy(values[value_count].name, "Coolant Temp");
                        strcpy(values[value_count].unit, "°C");
                        break;
                    case PID_THROTTLE_POS:
                        strcpy(values[value_count].name, "Throttle");
                        strcpy(values[value_count].unit, "%");
                        break;
                    case PID_ENGINE_LOAD:
                        strcpy(values[value_count].name, "Engine Load");
                        strcpy(values[value_count].unit, "%");
                        break;
                    case PID_FUEL_LEVEL:
                        strcpy(values[value_count].name, "Fuel Level");
                        strcpy(values[value_count].unit, "%");
                        break;
                    case PID_INTAKE_TEMP:
                        strcpy(values[value_count].name, "Intake Temp");
                        strcpy(values[value_count].unit, "°C");
                        break;
                    case PID_FUEL_PRESSURE:
                        strcpy(values[value_count].name, "Fuel Pressure");
                        strcpy(values[value_count].unit, "kPa");
                        break;
                    case PID_MAF_SENSOR:
                        strcpy(values[value_count].name, "MAF");
                        strcpy(values[value_count].unit, "g/s");
                        break;
                }
                value_count++;
            }
            delay(10);
        }
    }
    
    OBD2Value* getValues(int* count) { *count = value_count; return values; }
    DTCode* getDTCs(int* count) { *count = dtc_list.size(); return dtc_list.data(); }
    int getDTCCount() { return dtc_list.size(); }
    char* getVIN() { return vin; }
    bool isVINRead() { return vin_read_done; }
    
    // ============================================================
    // JSON برای Web Dashboard
    // ============================================================
    String toJSON() {
        String json = "{\n";
        json += "\"obd2_ready\": " + String(initialized ? "true" : "false") + ",\n";
        json += "\"vin\": \"" + String(vin) + "\",\n";
        json += "\"dtc_count\": " + String(dtc_list.size()) + ",\n";
        json += "\"values\": [\n";
        
        for (int i = 0; i < value_count; i++) {
            if (i > 0) json += ",\n";
            json += "  {\n";
            json += "    \"name\": \"" + String(values[i].name) + "\",\n";
            json += "    \"value\": " + String(values[i].value, 1) + ",\n";
            json += "    \"unit\": \"" + String(values[i].unit) + "\"\n";
            json += "  }";
        }
        
        json += "\n]\n}";
        return json;
    }
    
private:
    const char* dtcLookup(const char* code) {
        // --- DTC lookup table مختصر ---
        if (strcmp(code, "P0101") == 0) return "MAF sensor circuit range/performance";
        if (strcmp(code, "P0102") == 0) return "MAF sensor low input";
        if (strcmp(code, "P0103") == 0) return "MAF sensor high input";
        if (strcmp(code, "P0111") == 0) return "IAT sensor circuit range/performance";
        if (strcmp(code, "P0112") == 0) return "IAT sensor low input";
        if (strcmp(code, "P0113") == 0) return "IAT sensor high input";
        if (strcmp(code, "P0115") == 0) return "ECT sensor circuit";
        if (strcmp(code, "P0121") == 0) return "TPS circuit range/performance";
        if (strcmp(code, "P0122") == 0) return "TPS circuit low input";
        if (strcmp(code, "P0123") == 0) return "TPS circuit high input";
        if (strcmp(code, "P0135") == 0) return "O2 heater circuit";
        if (strcmp(code, "P0171") == 0) return "System too lean";
        if (strcmp(code, "P0172") == 0) return "System too rich";
        if (strcmp(code, "P0174") == 0) return "System too lean (bank 2)";
        if (strcmp(code, "P0175") == 0) return "System too rich (bank 2)";
        if (strcmp(code, "P0300") == 0) return "Random misfire detected";
        if (strcmp(code, "P0301") == 0) return "Cylinder 1 misfire";
        if (strcmp(code, "P0302") == 0) return "Cylinder 2 misfire";
        if (strcmp(code, "P0303") == 0) return "Cylinder 3 misfire";
        if (strcmp(code, "P0304") == 0) return "Cylinder 4 misfire";
        if (strcmp(code, "P0325") == 0) return "Knock sensor circuit";
        if (strcmp(code, "P0335") == 0) return "CKP sensor circuit";
        if (strcmp(code, "P0340") == 0) return "CMP sensor circuit";
        if (strcmp(code, "P0420") == 0) return "Catalyst efficiency below threshold";
        if (strcmp(code, "P0430") == 0) return "Catalyst efficiency (bank 2)";
        if (strcmp(code, "P0442") == 0) return "EVAP small leak";
        if (strcmp(code, "P0455") == 0) return "EVAP large leak";
        if (strcmp(code, "P0500") == 0) return "VSS circuit";
        if (strcmp(code, "P0505") == 0) return "IAC system";
        if (strcmp(code, "P0507") == 0) return "IAC system high";
        if (strcmp(code, "P0562") == 0) return "System voltage low";
        if (strcmp(code, "P0563") == 0) return "System voltage high";
        if (strcmp(code, "P0606") == 0) return "ECU internal fault";
        if (strcmp(code, "P0700") == 0) return "Transmission control system";
        if (strcmp(code, "P0705") == 0) return "Transmission range sensor";
        if (strcmp(code, "P0711") == 0) return "TFT sensor circuit";
        if (strcmp(code, "P0715") == 0) return "Input speed sensor";
        if (strcmp(code, "P0720") == 0) return "Output speed sensor";
        if (strcmp(code, "P0725") == 0) return "Engine speed input circuit";
        if (strcmp(code, "P1135") == 0) return "AFR sensor heater circuit";
        return "Unknown DTC";
    }
};

#endif
