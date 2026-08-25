#ifndef ISO_TP_H
#define ISO_TP_H

#include "config.h"
#include <stdint.h>
#include <vector>

// ============================================================
// ISO 15765-2 Transport Protocol (ISO-TP)
// برای پیام‌های UDS طولانیتر از 8 بایت
// ============================================================

#define ISO_TP_SINGLE_FRAME     0x00    // SF (1-7 bytes payload)
#define ISO_TP_FIRST_FRAME      0x10    // FF (8-4095 bytes)
#define ISO_TP_CONSECUTIVE_FRAME 0x20   // CF
#define ISO_TP_FLOW_CONTROL     0x30    // FC
#define ISO_TP_PADDING          0x55

#define ISO_TP_DEFAULT_BS       8       // Block Size
#define ISO_TP_DEFAULT_ST_MIN   10      // Separation Time (ms)

// --- Frame Types ---
typedef struct {
    uint8_t type;           // SF, FF, CF, FC
    uint16_t total_len;     // طول کل payload
    uint8_t data[4096];     // بافر داده
    uint16_t data_len;      // طول واقعی داده در این فریم
    uint8_t seq_num;        // شماره توالی (برای CF)
} ISOTPFrame;

class ISOTPTransport {
private:
    uint32_t target_id;          // CAN ID مقصد
    uint32_t source_id;          // CAN ID مبدأ
    uint16_t can_speed;          // سرعت CAN
    uint8_t tx_buffer[8];        // بافر ارسال
    uint8_t rx_buffer[256];      // بافر دریافت
    
    // --- حالت‌های انتقال ---
    typedef enum {
        IDLE,
        WAIT_FC,        // منتظر Flow Control
        SEND_CF,        // ارسال Consecutive Frame
        RECV_FF,        // دریافت First Frame
        RECV_CF,        // دریافت Consecutive Frame
        COMPLETE,
        ERROR
    } TransferState;
    
    TransferState state;
    uint16_t remaining_len;
    uint16_t already_sent;
    uint16_t total_message_len;
    uint8_t block_size;
    uint8_t st_min;
    uint8_t current_seq;
    
    // --- callback برای ارسال/دریافت واقعی CAN ---
    bool (*can_send)(uint32_t id, uint8_t* data, uint8_t len);
    bool (*can_poll)(uint32_t* id, uint8_t* data, uint8_t* len);
    
public:
    ISOTPTransport() : state(IDLE), target_id(0), source_id(0),
                       can_speed(500000), can_send(nullptr), can_poll(nullptr) {}
    
    void begin(uint32_t target, uint32_t source, uint32_t speed = 500000) {
        target_id = target;
        source_id = source;
        can_speed = speed;
        state = IDLE;
    }
    
    void setCallbacks(bool (*send)(uint32_t, uint8_t*, uint8_t),
                      bool (*poll)(uint32_t*, uint8_t*, uint8_t*)) {
        can_send = send;
        can_poll = poll;
    }
    
    // ============================================================
    // ارسال پیام بلند از طریق ISO-TP
    // ============================================================
    int send(const uint8_t* data, uint16_t len, uint32_t timeout_ms = 1000) {
        if (!can_send || !can_poll) return ERR_PARAM;
        if (len == 0) return ERR_PARAM;
        
        uint32_t start = millis();
        
        if (len <= 7) {
            // --- Single Frame ---
            tx_buffer[0] = (len & 0x0F);  // SF با طول
            memcpy(tx_buffer + 1, data, len);
            if (!can_send(target_id, tx_buffer, len + 1)) return ERR_BUS_OFF;
            return ERR_OK;
        }
        
        // --- First Frame (طول 8 تا 4095) ---
        tx_buffer[0] = ISO_TP_FIRST_FRAME | ((len >> 8) & 0x0F);
        tx_buffer[1] = len & 0xFF;
        uint8_t ff_payload = (len > 6) ? 6 : (len - 2);
        memcpy(tx_buffer + 2, data, ff_payload);
        if (!can_send(target_id, tx_buffer, ff_payload + 2)) return ERR_BUS_OFF;
        
        state = WAIT_FC;
        remaining_len = len - ff_payload;
        already_sent = ff_payload;
        current_seq = 1;
        
        // --- منتظر Flow Control ---
        while (state == WAIT_FC || state == SEND_CF) {
            if (millis() - start > timeout_ms) {
                state = ERROR;
                return ERR_TIMEOUT;
            }
            
            uint32_t rx_id;
            uint8_t rx_data[8];
            uint8_t rx_len;
            
            if (can_poll(&rx_id, rx_data, &rx_len)) {
                if (rx_id == source_id && rx_len >= 3) {
                    uint8_t fc_type = (rx_data[0] >> 4) & 0x0F;
                    if (fc_type == 0x03) {  // Flow Control
                        block_size = rx_data[1];
                        st_min = rx_data[2];
                        state = SEND_CF;
                        
                        // --- ارسال Consecutive Frames ---
                        uint8_t sent_in_block = 0;
                        while (remaining_len > 0) {
                            uint8_t cf_len = (remaining_len > 7) ? 7 : remaining_len;
                            tx_buffer[0] = ISO_TP_CONSECUTIVE_FRAME | (current_seq & 0x0F);
                            memcpy(tx_buffer + 1, data + already_sent, cf_len);
                            
                            if (!can_send(target_id, tx_buffer, cf_len + 1)) return ERR_BUS_OFF;
                            
                            already_sent += cf_len;
                            remaining_len -= cf_len;
                            current_seq = (current_seq + 1) % 16;
                            sent_in_block++;
                            
                            // کنترل جریان بر اساس Block Size
                            if (block_size > 0 && sent_in_block >= block_size && remaining_len > 0) {
                                state = WAIT_FC;
                                // منتظر FC بعدی
                                while (state == WAIT_FC) {
                                    if (millis() - start > timeout_ms) return ERR_TIMEOUT;
                                    delay(1);
                                    if (can_poll(&rx_id, rx_data, &rx_len)) {
                                        if (rx_id == source_id && ((rx_data[0] >> 4) & 0x0F) == 0x03) {
                                            block_size = rx_data[1];
                                            st_min = rx_data[2];
                                            state = SEND_CF;
                                            sent_in_block = 0;
                                        }
                                    }
                                }
                            }
                            
                            if (st_min < 127) delay(st_min);
                            else delayMicroseconds((st_min - 128) * 100 + 100);
                        }
                        
                        state = COMPLETE;
                        return ERR_OK;
                    }
                }
            }
            delay(1);
        }
        
        return ERR_OK;
    }
    
    // ============================================================
    // دریافت پیام بلند
    // ============================================================
    int receive(uint8_t* out_buf, uint16_t* out_len, uint32_t timeout_ms = 2000) {
        if (!can_poll) return ERR_PARAM;
        
        uint32_t start = millis();
        uint16_t total_len = 0;
        uint16_t received = 0;
        uint8_t seq = 0;
        uint8_t rx_data[8];
        uint8_t rx_len;
        uint32_t rx_id;
        
        // --- منتظر First Frame ---
        while (true) {
            if (millis() - start > timeout_ms) return ERR_TIMEOUT;
            
            if (can_poll(&rx_id, rx_data, &rx_len)) {
                if (rx_len == 0) continue;
                
                uint8_t n_pci = (rx_data[0] >> 4) & 0x0F;
                
                if (n_pci == 0x00) {  // Single Frame
                    uint8_t sf_len = rx_data[0] & 0x0F;
                    memcpy(out_buf, rx_data + 1, sf_len);
                    *out_len = sf_len;
                    return ERR_OK;
                }
                else if (n_pci == 0x01) {  // First Frame
                    total_len = ((rx_data[0] & 0x0F) << 8) | rx_data[1];
                    uint8_t ff_len = (total_len > 6) ? 6 : (total_len - 2);
                    memcpy(out_buf, rx_data + 2, ff_len);
                    received = ff_len;
                    
                    // ارسال Flow Control
                    uint8_t fc[3] = { 0x30, ISO_TP_DEFAULT_BS, ISO_TP_DEFAULT_ST_MIN };
                    can_send(target_id, fc, 3);
                    
                    // دریافت Consecutive Frames
                    while (received < total_len) {
                        if (millis() - start > timeout_ms) return ERR_TIMEOUT;
                        
                        if (can_poll(&rx_id, rx_data, &rx_len)) {
                            uint8_t cf_seq = rx_data[0] & 0x0F;
                            if (cf_seq == seq) {
                                uint8_t cf_len = rx_len - 1;
                                if (received + cf_len > total_len) cf_len = total_len - received;
                                memcpy(out_buf + received, rx_data + 1, cf_len);
                                received += cf_len;
                                seq = (seq + 1) % 16;
                            }
                        }
                        delay(1);
                    }
                    
                    *out_len = total_len;
                    return ERR_OK;
                }
            }
            delay(1);
        }
    }
};

#endif
