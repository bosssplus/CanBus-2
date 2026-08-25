#ifndef VEHICLE_DB_H
#define VEHICLE_DB_H

#include "config.h"
#include <vector>
#include <cstring>

// ============================================================
// Vehicle Database Pro — با پشتیبانی کامل از خودروهای ایرانی
// ============================================================

// --- ساختار یک CAN ID ---
typedef struct {
    uint32_t id;           // CAN ID
    bool     extended;     // 29-bit?
    uint8_t  dlc;          // Data Length Code
    const char* desc;      // توضیح
    uint8_t  priority;     // اولویت (0-255, 0=بالاترین)
} CANIDEntry;

// --- ساختار یک خودرو ---
typedef struct {
    const char* make;           // برند
    const char* model;          // مدل
    uint16_t year_start;        // سال شروع
    uint16_t year_end;          // سال پایان
    uint32_t wmi_code;          // کد WMI (۳ رقم اول VIN)
    uint32_t can_speed;         // سرعت CAN (bps)
    uint16_t can_id_count;      // تعداد CAN IDها
    const CANIDEntry* can_ids;  // لیست CAN IDها
    uint8_t diagnostic_type;    // 0=OBD2, 1=UDS on CAN, 2=UDS on ISO-TP
    uint8_t security_algo;      // 0=none, 1=PSA, 2=BMW CAS4, 3=MB AES
    uint32_t capabilities;      // capability flags
    const char* notes;          // یادداشت
} VehicleProfile;

// ============================================================
// پلاگین مخصوص پژو/سیتروئن
// ============================================================
static const CANIDEntry peugeot_can_ids[] = {
    { 0x116, false, 8, "Engine RPM", 0 },
    { 0x116, false, 8, "Vehicle Speed", 1 },
    { 0x116, false, 8, "Coolant Temp", 2 },
    { 0x132, false, 8, "Brake Pedal", 0 },
    { 0x132, false, 8, "Clutch Pedal", 1 },
    { 0x132, false, 8, "Accelerator Pedal", 2 },
    { 0x268, false, 8, "Steering Angle", 0 },
    { 0x228, false, 8, "ABS Status", 0 },
    { 0x3C0, false, 8, "Airbag Status", 0 },
    { 0x426, false, 8, "Door Status", 0 },
    { 0x436, false, 8, "Window Status", 0 },
    { 0x4E0, false, 8, "Light Status", 0 },
    { 0x540, false, 8, "Cruise Control", 0 },
    { 0x7E0, false, 8, "UDS Request", 0 },
    { 0x7E8, false, 8, "UDS Reply", 0 },
};

// ============================================================
// پلاگین مخصوص تویوتا/لکسوس
// ============================================================
static const CANIDEntry toyota_can_ids[] = {
    { 0x0C4, false, 8, "Engine RPM", 0 },
    { 0x0B4, false, 8, "Vehicle Speed", 0 },
    { 0x0A0, false, 8, "Throttle Position", 0 },
    { 0x0C9, false, 8, "Coolant Temp", 0 },
    { 0x0D0, false, 8, "Brake Status", 0 },
    { 0x0E0, false, 8, "Steering Angle", 0 },
    { 0x030, false, 8, "Door Status", 0 },
    { 0x040, false, 8, "Light Status", 0 },
    { 0x7E0, false, 8, "UDS Request", 0 },
    { 0x7E8, false, 8, "UDS Reply", 0 },
};

// ============================================================
// پلاگین مخصوص BMW
// ============================================================
static const CANIDEntry bmw_can_ids[] = {
    { 0x0AA, false, 8, "Engine RPM", 0 },
    { 0x0B0, false, 8, "Vehicle Speed", 0 },
    { 0x1A0, false, 8, "Coolant Temp", 0 },
    { 0x1D0, false, 8, "Brake Status", 0 },
    { 0x0F0, false, 8, "Steering Angle", 0 },
    { 0x1F0, false, 8, "ABS/DSC", 0 },
    { 0x130, false, 8, "Door Status", 0 },
    { 0x1C0, false, 8, "Light Status", 0 },
    { 0x7E0, false, 8, "UDS Request", 0 },
    { 0x7E8, false, 8, "UDS Reply", 0 },
};

// ============================================================
// پلاگین مخصوص هیوندای/کیا
// ============================================================
static const CANIDEntry hyundai_can_ids[] = {
    { 0x316, false, 8, "Engine RPM", 0 },
    { 0x316, false, 8, "Vehicle Speed", 1 },
    { 0x316, false, 8, "Coolant Temp", 2 },
    { 0x420, false, 8, "Brake Status", 0 },
    { 0x426, false, 8, "Steering Angle", 0 },
    { 0x3C0, false, 8, "Door Status", 0 },
    { 0x130, false, 8, "Light Status", 0 },
    { 0x7E0, false, 8, "UDS Request", 0 },
    { 0x7E8, false, 8, "UDS Reply", 0 },
};

// ============================================================
// دیتابیس کامل خودروها
// ============================================================
static const VehicleProfile vehicle_profiles[] = {
    // ===== خودروهای ایرانی (IKCO/Saipa) =====
    { "IKCO", "Peugeot 206 (Pars/Tip2)", 2006, 2020, 0x937,
      500000, 15, peugeot_can_ids, 1, 1, 
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_UDS_SECURITY | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "پژو 206 تیپ ۲/۳/۵/۶ — پشتیبانی کامل CAN + UDS + ISO-TP" },
    
    { "IKCO", "Peugeot 207 (iDrive)", 2017, 2024, 0x937,
      500000, 15, peugeot_can_ids, 1, 1,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_UDS_SECURITY | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "پژو 207 دنده‌ای و اتومات — UDS Security Access از نوع PSA" },
    
    { "IKCO", "Samand (Soren/Sevome)", 2005, 2024, 0x937,
      500000, 15, peugeot_can_ids, 1, 1,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_UDS_SECURITY | CAP_SNIFFER | CAP_AUTOLEARN,
      "سمند/سورن/سوم — CAN Bus با پروتکل PSA" },
    
    { "IKCO", "Dena (Denay Plus)", 2010, 2024, 0x937,
      500000, 15, peugeot_can_ids, 1, 1,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_UDS_SECURITY | CAP_SNIFFER | CAP_AUTOLEARN,
      "دنا/دنا پلاس — معماری CAN مثل پژو 206" },
    
    { "IKCO", "Rana (Peugeot 405)", 2002, 2024, 0x937,
      250000, 15, peugeot_can_ids, 1, 1,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN,
      "رانا/پژو 405 — سرعت CAN پایین‌تر (250kbps)" },
    
    { "Saipa", "Tiba (Saba)", 2010, 2024, 0x934,
      500000, 15, peugeot_can_ids, 1, 1,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN,
      "تیبا/ساینا — OBD2 + UDS پایه" },
    
    { "Saipa", "Quick (Quik)", 2017, 2024, 0x934,
      500000, 15, peugeot_can_ids, 1, 1,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN,
      "کوییک — بر پایه تیبا" },
    
    { "Saipa", "Saina (Sina)", 2014, 2024, 0x934,
      500000, 15, peugeot_can_ids, 1, 1,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN,
      "ساینا — OBD2 standard" },
    
    { "Saipa", "Changan CS35 (Shahin)", 2021, 2024, 0x9C9,
      500000, 15, hyundai_can_ids, 1, 1,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "شاهین (چانگان CS35) — UDS + ISO-TP" },
    
    // ===== خودروهای خارجی پرتیراژ =====
    { "Toyota", "Corolla (E150/E210)", 2007, 2024, 0x8B1,
      500000, 10, toyota_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "تویوتا کرولا — OBD2 + UDS کامل" },
    
    { "Toyota", "Camry (XV50/XV70)", 2012, 2024, 0x8B1,
      500000, 10, toyota_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "کمری — پشتیبانی کامل" },
    
    { "Toyota", "RAV4 (XA40/XA50)", 2013, 2024, 0x8B1,
      500000, 10, toyota_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "RAV4 — پشتیبانی کامل" },
    
    { "Toyota", "Prius (XW30/XW50)", 2010, 2024, 0x8B1,
      500000, 10, toyota_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "پریوس — هیبرید + CAN کامل" },
    
    { "BMW", "3 Series (E90/F30/G20)", 2005, 2024, 0xWBA,
      500000, 10, bmw_can_ids, 1, 2,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "BMW سری ۳ — UDS با CAS4+" },
    
    { "BMW", "5 Series (E60/F10/G30)", 2005, 2024, 0xWBA,
      500000, 10, bmw_can_ids, 1, 2,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "BMW سری ۵ — UDS با CAS4+" },
    
    { "BMW", "X Series (E70/F15/G05)", 2007, 2024, 0xWBA,
      500000, 10, bmw_can_ids, 1, 2,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "BMW X5/X6 — UDS کامل" },
    
    { "Hyundai", "Elantra (MD/AD/CN7)", 2011, 2024, 0xHMC,
      500000, 9, hyundai_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "آوانته/الانترا — OBD2 + UDS" },
    
    { "Hyundai", "Sonata (YF/LF/DN8)", 2010, 2024, 0xHMC,
      500000, 9, hyundai_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "سوناتا — پشتیبانی کامل" },
    
    { "Hyundai", "Tucson (TL/NX4)", 2015, 2024, 0xHMC,
      500000, 9, hyundai_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "توسان — پشتیبانی کامل" },
    
    { "Kia", "Sportage (QL/NQ5)", 2016, 2024, 0xKIA,
      500000, 9, hyundai_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "اسپورتیج — برادر توسان" },
    
    { "Kia", "Cerato (BD/BN/CNC)", 2010, 2024, 0xKIA,
      500000, 9, hyundai_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "سراتو — پشتیبانی کامل" },
    
    { "Mercedes", "C-Class (W204/W205/W206)", 2008, 2024, 0xWDD,
      500000, 10, peugeot_can_ids, 1, 3,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "ماشین — UDS روی CAN" },
    
    { "Mercedes", "E-Class (W212/W213)", 2009, 2024, 0xWDD,
      500000, 10, peugeot_can_ids, 1, 3,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "E کلاس — UDS + AES Security" },
    
    { "Volkswagen", "Golf (MK6/MK7/MK8)", 2009, 2024, 0xWVW,
      500000, 10, hyundai_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "گل‌ف — OBD2 + UDS" },
    
    { "Volkswagen", "Passat (B7/B8)", 2011, 2024, 0xWVW,
      500000, 10, hyundai_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "پاسات — پشتیبانی کامل" },
    
    { "Nissan", "Altima (L33/L34)", 2013, 2024, 0xJN1,
      500000, 10, toyota_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "آلتیما/تیانا" },
    
    { "Nissan", "Qashqai (J11/J12)", 2014, 2024, 0xJN1,
      500000, 10, toyota_can_ids, 1, 0,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "کاشکای — SUV" },
    
    // ===== خودروهای جدید ایرانی (Farsi platform) =====
    { "IKCO", "Tara (Dongo)", 2021, 2024, 0x937,
      500000, 15, peugeot_can_ids, 1, 1,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_UDS_SECURITY | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "تارا — پلتفرم جدید IKCO با AES-128 در ریموت" },
    
    { "IKCO", "Reera", 2024, 2026, 0x937,
      500000, 15, peugeot_can_ids, 1, 1,
      CAP_CAN_READ | CAP_CAN_WRITE | CAP_OBD2 | CAP_UDS_DIAG | CAP_UDS_SECURITY | CAP_SNIFFER | CAP_AUTOLEARN | CAP_ISO_TP,
      "ریرا — جدیدترین محصول IKCO" },
};

#define VEHICLE_COUNT (sizeof(vehicle_profiles) / sizeof(vehicle_profiles[0]))

// ============================================================
// Vehicle Database Class
// ============================================================
class VehicleDB {
public:
    // ============================================================
    // پیدا کردن خودرو از روی VIN (WMI)
    // ============================================================
    static const VehicleProfile* findByVIN(const char* vin) {
        if (!vin || strlen(vin) < 3) return nullptr;
        
        // استخراج کد ۳ رقمی WMI
        char wmi_str[4] = { vin[0], vin[1], vin[2], '\0' };
        uint32_t wmi = 0;
        for (int i = 0; i < 3; i++) {
            wmi = (wmi << 8) | wmi_str[i];
        }
        
        // جستجو
        for (int i = 0; i < VEHICLE_COUNT; i++) {
            if (vehicle_profiles[i].wmi_code == wmi) {
                return &vehicle_profiles[i];
            }
        }
        
        // جستجوی approximate با ۲ رقم اول
        uint32_t wmi2 = (wmi >> 8) & 0xFFFF;
        for (int i = 0; i < VEHICLE_COUNT; i++) {
            uint32_t cmp = (vehicle_profiles[i].wmi_code >> 8) & 0xFFFF;
            if (cmp == wmi2) {
                return &vehicle_profiles[i];
            }
        }
        
        return nullptr;
    }
    
    // ============================================================
    // جستجو با برند و مدل
    // ============================================================
    static const VehicleProfile* findByModel(const char* make, const char* model) {
        for (int i = 0; i < VEHICLE_COUNT; i++) {
            bool make_match = (strcasecmp(vehicle_profiles[i].make, make) == 0);
            bool model_match = (strcasecmp(vehicle_profiles[i].model, model) == 0);
            if (make_match && model_match) {
                return &vehicle_profiles[i];
            }
        }
        return nullptr;
    }
    
    // ============================================================
    // جستجوی partial name
    // ============================================================
    static const VehicleProfile* findByPartial(const char* partial) {
        for (int i = 0; i < VEHICLE_COUNT; i++) {
            if (strstr(vehicle_profiles[i].model, partial) != nullptr ||
                strstr(vehicle_profiles[i].make, partial) != nullptr) {
                return &vehicle_profiles[i];
            }
        }
        return nullptr;
    }
    
    // ============================================================
    // گرفتن CAN IDهای یک خودرو
    // ============================================================
    static int getCANIDs(const VehicleProfile* profile, uint32_t* ids, int max_count) {
        if (!profile) return 0;
        int count = 0;
        for (int i = 0; i < profile->can_id_count && count < max_count; i++) {
            ids[count++] = profile->can_ids[i].id;
        }
        return count;
    }
    
    // ============================================================
    // JSON برای Web Dashboard
    // ============================================================
    static String toJSON() {
        String json = "[\n";
        for (int i = 0; i < VEHICLE_COUNT; i++) {
            if (i > 0) json += ",\n";
            json += "  {\n";
            json += "    \"index\": " + String(i) + ",\n";
            json += "    \"make\": \"" + String(vehicle_profiles[i].make) + "\",\n";
            json += "    \"model\": \"" + String(vehicle_profiles[i].model) + "\",\n";
            json += "    \"year_start\": " + String(vehicle_profiles[i].year_start) + ",\n";
            json += "    \"year_end\": " + String(vehicle_profiles[i].year_end) + ",\n";
            json += "    \"can_speed\": " + String(vehicle_profiles[i].can_speed) + ",\n";
            json += "    \"can_ids\": " + String(vehicle_profiles[i].can_id_count) + ",\n";
            json += "    \"notes\": \"" + String(vehicle_profiles[i].notes) + "\"\n";
            json += "  }";
        }
        json += "\n]";
        return json;
    }
};

#endif
