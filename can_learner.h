#ifndef CAN_LEARNER_H
#define CAN_LEARNER_H

#include "config.h"
#include <map>
#include <vector>
#include <functional>

// ============================================================
// Auto CAN ID Learning
// به صورت خودکار CAN IDهای خودرو را شناسایی می‌کند
// ============================================================

// --- نوع CAN ID از روی الگو ---
typedef enum {
    SIG_UNKNOWN     = 0,
    SIG_ENGINE_RPM  = 1,      // دور موتور
    SIG_VEHICLE_SPEED,        // سرعت
    SIG_COOLANT_TEMP,         // دمای آب
    SIG_THROTTLE_POS,        // وضعیت دریچه گاز
    SIG_BRAKE_PEDAL,         // ترمز
    SIG_STEERING_ANGLE,      // زاویه فرمان
    SIG_ENGINE_LOAD,         // بار موتور
    SIG_FUEL_LEVEL,          // سطح بنزین
    SIG_ODO_METER,           // کیلومتر
    SIG_GEAR_POSITION,       // دنده
    SIG_DOOR_STATUS,         // درب‌ها
    SIG_WINDOW_STATUS,       // شیشه‌ها
    SIG_LIGHT_STATUS,        // چراغ‌ها
    SIG_CRUISE_STATUS,       // کروز کنترل
    SIG_ABS_ACTIVE,          // ABS
    SIG_AIRBAG_STATUS,       // ایربگ
    SIG_VIN                  // VIN (از UDS)
} SignalType;

// --- یک کانال CAN ---
typedef struct {
    uint32_t can_id;           // CAN ID
    bool     is_extended;      // 11-bit یا 29-bit
    uint8_t  dlc;             // طول داده
    uint8_t  data[8];          // آخرین داده
    uint32_t last_seen;        // آخرین بار (ms)
    uint32_t frequency;        // تعداد دفعات
    uint8_t  bit_changes[8];   // آمار تغییر بیت‌ها
    bool     is_known;         // شناسایی شده؟
    SignalType signal_type;    // نوع سیگنال
    uint8_t  sig_start_bit;    // بیت شروع سیگنال
    uint8_t  sig_length;       // طول سیگنال
    bool     sig_big_endian;   // Big Endian?
    float    sig_scale;        // ضریب
    float    sig_offset;       // افست
    char     sig_name[32];     // نام
} CANChannel;

// --- نتیجه یادگیری ---
typedef struct {
    uint32_t can_id;
    SignalType type;
    uint8_t start_bit;
    uint8_t length;
    float scale;
    float offset;
    char name[32];
    float confidence;          // 0.0 - 1.0
} LearnedSignal;

class CANLearner {
private:
    std::map<uint32_t, CANChannel> channels;
    std::vector<LearnedSignal> learned_signals;
    uint32_t total_frames_received;
    uint32_t learn_start_ms;
    bool learning_active;
    uint16_t min_samples_for_analysis;
    
    // --- الگوهای شناخته شده برای تشخیص هوشمند ---
    struct KnownPattern {
        SignalType type;
        const char* name;
        uint16_t expected_range_min;   // مقدار حداقل
        uint16_t expected_range_max;   // مقدار حداکثر
        uint8_t  typical_bit_count;
        float    typical_scale;
        float    typical_offset;
        bool     (*validator)(uint8_t* data, uint8_t len); // تابع اعتبارسنجی
    };
    
    static bool validateRPM(uint8_t* data, uint8_t len) {
        // RPM معمولاً بین 0 تا 8000
        if (len < 2) return false;
        uint16_t val = (data[0] << 8) | data[1];
        return (val > 0 && val < 8000);
    }
    
    static bool validateSpeed(uint8_t* data, uint8_t len) {
        if (len < 1) return false;
        return (data[0] >= 0 && data[0] < 300);  // km/h
    }
    
    static bool validateCoolant(uint8_t* data, uint8_t len) {
        if (len < 1) return false;
        return (data[0] >= 60 && data[0] <= 120);  // درجه سانتیگراد
    }
    
    static bool validateBrake(uint8_t* data, uint8_t len) {
        // ترمز معمولاً 0 یا 1 است
        if (len < 1) return false;
        return (data[0] == 0 || data[0] == 1);
    }
    
    KnownPattern patterns[6] = {
        { SIG_ENGINE_RPM,   "Engine RPM",   0, 8000,  2, 1.0,  0.0, validateRPM },
        { SIG_VEHICLE_SPEED, "Vehicle Speed", 0, 300,  1, 1.0,  0.0, validateSpeed },
        { SIG_COOLANT_TEMP, "Coolant Temp", 60, 120,  1, 1.0,  40.0, validateCoolant },
        { SIG_BRAKE_PEDAL,  "Brake Pedal",  0, 1,     1, 1.0,  0.0, validateBrake },
        { SIG_DOOR_STATUS,  "Door Status",  0, 15,    1, 1.0,  0.0, nullptr },
        { SIG_LIGHT_STATUS, "Light Status", 0, 7,     1, 1.0,  0.0, nullptr }
    };

public:
    CANLearner() : total_frames_received(0), learn_start_ms(0),
                   learning_active(false), min_samples_for_analysis(100) {}
    
    void begin() {
        learning_active = true;
        learn_start_ms = millis();
        total_frames_received = 0;
        channels.clear();
        learned_signals.clear();
    }
    
    void stop() {
        learning_active = false;
        analyze();
    }
    
    // ============================================================
    // ثبت یک فریم CAN برای یادگیری
    // ============================================================
    void feedFrame(uint32_t can_id, bool is_extended, uint8_t* data, uint8_t len) {
        if (!learning_active) return;
        
        total_frames_received++;
        
        // --- پیدا کردن یا ایجاد کانال ---
        auto it = channels.find(can_id);
        if (it == channels.end()) {
            CANChannel ch;
            ch.can_id = can_id;
            ch.is_extended = is_extended;
            ch.dlc = len;
            memcpy(ch.data, data, len);
            ch.last_seen = millis();
            ch.frequency = 1;
            memset(ch.bit_changes, 0, sizeof(ch.bit_changes));
            ch.is_known = false;
            ch.signal_type = SIG_UNKNOWN;
            ch.sig_start_bit = 0;
            ch.sig_length = 0;
            ch.sig_scale = 1.0;
            ch.sig_offset = 0.0;
            ch.sig_name[0] = '\0';
            channels[can_id] = ch;
        } else {
            CANChannel* ch = &it->second;
            
            // --- محاسبه بیت‌هایی که تغییر کرده‌اند ---
            for (int i = 0; i < len && i < 8; i++) {
                if (data[i] != ch->data[i]) {
                    ch->bit_changes[i]++;
                }
            }
            
            memcpy(ch->data, data, (len < 8) ? len : 8);
            ch->dlc = len;
            ch->last_seen = millis();
            ch->frequency++;
        }
    }
    
    // ============================================================
    // تحلیل داده‌ها و شناسایی خودکار سیگنال‌ها
    // ============================================================
    void analyze() {
        learning_active = false;
        learned_signals.clear();
        
        int active_channels = 0;
        for (auto& pair : channels) {
            CANChannel* ch = &pair.second;
            if (ch->frequency >= min_samples_for_analysis) {
                active_channels++;
                analyzeChannel(ch);
            }
        }
    }
    
    // ============================================================
    // تحلیل یک کانال
    // ============================================================
    void analyzeChannel(CANChannel* ch) {
        // --- بررسی فرکانس ---
        uint32_t elapsed = (millis() - learn_start_ms) / 1000;
        if (elapsed == 0) elapsed = 1;
        uint32_t hz = ch->frequency / elapsed;
        
        // --- کانال‌های پرتکرار معمولاً سنسورها هستند ---
        if (hz >= 50 && hz <= 200) {
            // احتمالاً RPM یا Speed
            tryMatchPattern(ch->data, ch->dlc, ch);
        }
        
        // --- کانال‌های با فرکانس متوسط (10-50Hz) ---
        else if (hz >= 10 && hz < 50) {
            // احتمالاً coolant, throttle
            tryMatchPattern(ch->data, ch->dlc, ch);
        }
        
        // --- کانال‌های کمانرژی (< 10Hz) ---
        else if (hz < 10 && ch->bit_changes[0] > 0) {
            // احتمالاً دکمه‌ها، وضعیت‌ها
            tryMatchPattern(ch->data, ch->dlc, ch);
        }
    }
    
    // ============================================================
    // تطبیق با الگوهای شناخته شده
    // ============================================================
    void tryMatchPattern(uint8_t* data, uint8_t len, CANChannel* ch) {
        for (int p = 0; p < 6; p++) {
            KnownPattern* pat = &patterns[p];
            
            if (pat->validator && pat->validator(data, len)) {
                // --- الگو تطبیق خورد! ---
                ch->is_known = true;
                ch->signal_type = pat->type;
                strncpy(ch->sig_name, pat->name, sizeof(ch->sig_name) - 1);
                ch->sig_scale = pat->typical_scale;
                ch->sig_offset = pat->typical_offset;
                
                LearnedSignal sig;
                sig.can_id = ch->can_id;
                sig.type = pat->type;
                sig.start_bit = 0;
                sig.length = pat->typical_bit_count * 8;
                sig.scale = pat->typical_scale;
                sig.offset = pat->typical_offset;
                strncpy(sig.name, pat->name, sizeof(sig.name) - 1);
                sig.confidence = 0.85;  // confidence بالا
                learned_signals.push_back(sig);
                return;
            }
        }
        
        // --- اگر هیچ الگویی تطبیق نخورد، کانال را به عنوان raw ذخیره کن ---
        if (ch->frequency > min_samples_for_analysis * 2) {
            LearnedSignal sig;
            sig.can_id = ch->can_id;
            sig.type = SIG_UNKNOWN;
            sig.start_bit = 0;
            sig.length = ch->dlc * 8;
            sig.scale = 1.0;
            sig.offset = 0.0;
            snprintf(sig.name, sizeof(sig.name), "CAN_ID_0x%03lX", ch->can_id);
            sig.confidence = 0.3;
            learned_signals.push_back(sig);
        }
    }
    
    // ============================================================
    // دریافت نتایج یادگیری
    // ============================================================
    void getResults(LearnedSignal* out, int* count) {
        int max_out = *count;
        *count = 0;
        for (size_t i = 0; i < learned_signals.size() && *count < max_out; i++) {
            out[*count] = learned_signals[i];
            (*count)++;
        }
    }
    
    int getSignalCount() { return learned_signals.size(); }
    int getChannelCount() { return channels.size(); }
    uint32_t getTotalFrames() { return total_frames_received; }
    bool isLearning() { return learning_active; }
    
    // ============================================================
    // تولید خروجی JSON برای Web Dashboard
    // ============================================================
    String toJSON() {
        String json = "{\n";
        json += "\"status\": \"" + String(learning_active ? "learning" : "done") + "\",\n";
        json += "\"total_frames\": " + String(total_frames_received) + ",\n";
        json += "\"channels\": " + String(channels.size()) + ",\n";
        json += "\"signals\": [\n";
        
        bool first = true;
        for (auto& sig : learned_signals) {
            if (!first) json += ",\n";
            first = false;
            json += "  {\n";
            json += "    \"can_id\": \"0x" + String(sig.can_id, HEX) + "\",\n";
            json += "    \"name\": \"" + String(sig.name) + "\",\n";
            json += "    \"confidence\": " + String(sig.confidence, 2) + ",\n";
            json += "    \"scale\": " + String(sig.scale, 3) + ",\n";
            json += "    \"offset\": " + String(sig.offset, 3) + "\n";
            json += "  }";
        }
        
        json += "\n]\n}";
        return json;
    }
};

#endif
