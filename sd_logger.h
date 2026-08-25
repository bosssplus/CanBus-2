#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "config.h"
#include <SD.h>
#include <FS.h>

// ============================================================
// SD Card Logger — ذخیره تمام CAN Frameها روی SD
// ============================================================

class SDLogger {
private:
    bool initialized;
    bool card_present;
    bool logging_active;
    File log_file;
    char filename[32];
    uint32_t frames_logged;
    uint32_t start_time_ms;
    uint32_t last_flush_ms;
    uint32_t session_id;
    
public:
    SDLogger() : initialized(false), card_present(false), logging_active(false),
                 frames_logged(0), start_time_ms(0), last_flush_ms(0), session_id(0) {
        filename[0] = '\0';
    }
    
    // ============================================================
    // مقداردهی اولیه SD Card
    // ============================================================
    bool begin() {
        pinMode(SD_CS_PIN, OUTPUT);
        digitalWrite(SD_CS_PIN, HIGH);
        delay(10);
        
        if (!SD.begin(SD_CS_PIN)) {
            Serial.println("[SD] Card not found");
            card_present = false;
            initialized = true;
            return false;
        }
        
        card_present = true;
        initialized = true;
        
        // خواندن session ID از فایل شمارنده
        File counter = SD.open("/session.cnt", FILE_READ);
        if (counter) {
            char buf[16];
            int idx = 0;
            while (counter.available() && idx < 15) {
                char c = counter.read();
                if (c >= '0' && c <= '9') buf[idx++] = c;
            }
            buf[idx] = '\0';
            session_id = atoi(buf) + 1;
            counter.close();
        } else {
            session_id = 1;
        }
        
        // ذخیره session ID جدید
        counter = SD.open("/session.cnt", FILE_WRITE);
        if (counter) {
            counter.printf("%lu", session_id);
            counter.close();
        }
        
        Serial.printf("[SD] OK - Session #%lu\n", session_id);
        return true;
    }
    
    // ============================================================
    // شروع یک جلسه Logging جدید
    // ============================================================
    bool startSession() {
        if (!card_present) return false;
        
        closeSession();
        
        snprintf(filename, sizeof(filename), "/can_%04lu.csv", session_id);
        
        log_file = SD.open(filename, FILE_WRITE);
        if (!log_file) {
            Serial.println("[SD] Failed to create log file");
            return false;
        }
        
        // Header CSV
        log_file.println("Timestamp_ms,CAN_ID,Extended,DLC,B0,B1,B2,B3,B4,B5,B6,B7");
        log_file.flush();
        
        logging_active = true;
        frames_logged = 0;
        start_time_ms = millis();
        last_flush_ms = millis();
        
        Serial.printf("[SD] Logging started: %s\n", filename);
        return true;
    }
    
    // ============================================================
    // بستن جلسه جاری
    // ============================================================
    void closeSession() {
        if (log_file) {
            log_file.flush();
            log_file.close();
        }
        logging_active = false;
        if (frames_logged > 0) {
            Serial.printf("[SD] Session #%lu closed. %lu frames logged.\n", 
                          session_id, frames_logged);
        }
    }
    
    // ============================================================
    // ثبت یک فریم CAN در CSV
    // ============================================================
    bool logFrame(uint32_t id, bool extended, uint8_t* data, uint8_t len) {
        if (!logging_active || !log_file) return false;
        
        uint32_t elapsed = millis() - start_time_ms;
        
        log_file.printf("%lu,0x%03lX,%s,%d",
            elapsed, id, extended ? "29" : "11", len);
        
        for (int i = 0; i < 8; i++) {
            if (i < len)
                log_file.printf(",0x%02X", data[i]);
            else
                log_file.printf(",");
        }
        log_file.println();
        
        frames_logged++;
        
        // Flush هر ۱ ثانیه
        if (millis() - last_flush_ms > 1000) {
            log_file.flush();
            last_flush_ms = millis();
        }
        
        return true;
    }
    
    // ============================================================
    // متوقف کردن کامل
    // ============================================================
    void end() {
        closeSession();
        if (card_present) {
            SD.end();
        }
        initialized = false;
        card_present = false;
    }
    
    // ============================================================
    // آمار
    // ============================================================
    bool isCardPresent() { return card_present; }
    bool isLogging() { return logging_active; }
    uint32_t getFramesLogged() { return frames_logged; }
    uint32_t getSessionID() { return session_id; }
    
    String getLogFilename() { return String(filename); }
    
    String toJSON() {
        String json = "{\n";
        json += "\"card\": " + String(card_present ? "true" : "false") + ",\n";
        json += "\"logging\": " + String(logging_active ? "true" : "false") + ",\n";
        json += "\"frames\": " + String(frames_logged) + ",\n";
        json += "\"session\": " + String(session_id) + ",\n";
        json += "\"file\": \"" + String(filename) + "\"\n";
        json += "}";
        return json;
    }
};

#endif
