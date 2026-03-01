#ifndef GPS_UBX_H
#define GPS_UBX_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern UART_HandleTypeDef huart1;


// UART handle used for the NEO module(s)
#define GPS_UART_HANDLE      huart1

// Debug LED
#define GPS_DEBUG_GPIO_Port  GPIOE
#define GPS_DEBUG_GPIO_Pin   GPIO_PIN_13

// ─────────────────────────────────────────────
//  Module profiles
// ─────────────────────────────────────────────

#define GPS_MODULE_M8N   1
#define GPS_MODULE_M8U   2

#ifndef GPS_MODULE_TYPE
    // 기본값은 M8N 프로파일
    #define GPS_MODULE_TYPE GPS_MODULE_M8U
#endif

#if GPS_MODULE_TYPE == GPS_MODULE_M8U

    // NEO-M8U: HNR 엔진 지원
    #define GPS_ENABLE_HNR      0
    #define GPS_NAV_RATE_MS     500U   // 2 Hz nav solution
    #define GPS_HNR_RATE_HZ     0U    // 20 Hz high-rate PVT

#elif GPS_MODULE_TYPE == GPS_MODULE_M8N

    // NEO-M8N: HNR 없음, NAV-PVT 10 Hz
    #define GPS_ENABLE_HNR      0
    #define GPS_NAV_RATE_MS     100U   // 10 Hz nav solution
    #define GPS_HNR_RATE_HZ     0U     // 사용 안 함

#else
    #error "Unsupported GPS_MODULE_TYPE in gps_ubx.h"
#endif

// Max payload we want to parse (NAV-SAT can be large)
#define GPS_UBX_MAX_PAYLOAD  400U

// ---------- Raw UBX message structures we care about ----------

#pragma pack(push, 1)

// UBX-HNR-PVT (0x28 0x00), payload length 72 bytes on M8U
typedef struct
{
    uint32_t iTOW;     // ms
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;    // bitfield
    int32_t  nano;     // ns
    uint8_t  gpsFix;   // 0..5
    uint8_t  flags;    // bitfield
    uint8_t  reserved1[2];

    int32_t  lon;      // 1e-7 deg
    int32_t  lat;      // 1e-7 deg
    int32_t  height;   // mm
    int32_t  hMSL;     // mm

    int32_t  gSpeed;   // mm/s (2D ground speed)
    int32_t  speed;    // mm/s (3D speed)
    int32_t  headMot;  // 1e-5 deg
    int32_t  headVeh;  // 1e-5 deg

    uint32_t hAcc;     // mm
    uint32_t vAcc;     // mm
    uint32_t sAcc;     // mm/s
    uint32_t headAcc;  // 1e-5 deg

    uint8_t  reserved2[4];
} ubx_hnr_pvt_t;

// UBX-NAV-PVT (0x01 0x07), payload length 92 bytes
typedef struct
{
    uint32_t iTOW;   // ms
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;   // X1
    uint32_t tAcc;    // ns
    int32_t  nano;    // ns

    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;

    int32_t  lon;     // 1e-7 deg
    int32_t  lat;     // 1e-7 deg
    int32_t  height;  // mm
    int32_t  hMSL;    // mm

    uint32_t hAcc;    // mm
    uint32_t vAcc;    // mm

    int32_t  velN;    // mm/s
    int32_t  velE;    // mm/s
    int32_t  velD;    // mm/s
    int32_t  gSpeed;  // mm/s
    int32_t  headMot; // 1e-5 deg

    uint32_t sAcc;    // mm/s
    uint32_t headAcc; // 1e-5 deg
    uint16_t pDOP;    // 0.01

    uint8_t  flags3;
    uint8_t  reserved1[5];

    int32_t  headVeh; // 1e-5 deg
    int16_t  magDec;  // 1e-2 deg
    uint16_t magAcc;  // 1e-2 deg
} ubx_nav_pvt_t;

#pragma pack(pop)

// Simplified "what the application cares about" fix structure.
typedef struct
{
    bool     valid;       // 주행용 위치 fix 유효 여부 (2D/3D)
    bool     time_valid;  // UTC 날짜/시간이 유효한지

    uint8_t  fixType;         // 0..5 as in datasheet
    bool     fixOk;           // gnssFixOK flag
    uint8_t  numSV_used;      // satellites used in solution (NAV-PVT.numSV)
    uint8_t  numSV_visible;   // satellites visible/tracked (NAV-SAT.numSvs)

    // UTC date/time
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;

    // Position / velocity / heading (raw from UBX)
    int32_t  lon;       // 1e-7 deg
    int32_t  lat;       // 1e-7 deg
    int32_t  height;    // mm
    int32_t  hMSL;      // mm
    int32_t  gSpeed;    // mm/s (2D ground speed from chip)
    int32_t  headMot;   // 1e-5 deg (heading of motion from chip)

    // Accuracy & quality
    uint32_t hAcc;      // mm
    uint32_t vAcc;      // mm
    uint32_t sAcc;      // mm/s
    uint32_t headAcc;   // 1e-5 deg
    uint16_t pDOP;      // 0.01

    // Misc
    uint32_t iTOW_ms;       // GPS time of week [ms]
    uint8_t  raw_valid;     // copy of UBX valid bitfield (for debug)

    // ★ 보드 공통 시간축 기준 호스트 timestamp
    uint32_t host_time_ms;  // APP_TIME_GetMs() when this fix was updated

    // ★ LAT/LON 기반 파생 수평 속도 (HNR 20 Hz)
    float    speed_llh_mps;   // [m/s]
    float    speed_llh_kmh;   // [km/h]

    // ★ LAT/LON 기반 파생 헤딩
    float    heading_llh_deg;   // [deg], 0 = North, 90 = East
    uint8_t  heading_llh_valid; // 0 = invalid / not enough speed, 1 = valid



} gps_fix_basic_t;


// Global latest raw frames (optional to use)
extern volatile ubx_hnr_pvt_t g_hnr_pvt;
extern volatile bool          g_hnr_pvt_valid;
extern volatile ubx_nav_pvt_t g_nav_pvt;
extern volatile bool          g_nav_pvt_valid;

// High-level merged fix (what app usually needs)
extern volatile gps_fix_basic_t g_gps_fix;
extern volatile bool            g_gps_fix_new;

// API
void GPS_UBX_InitAndConfigure(void);
void GPS_UBX_StartUartRx(void);

// Called from HAL UART IRQ – already provided by gps_ubx.c.
// You usually don't need to call this yourself.
void GPS_UBX_OnByte(uint8_t byte);

// Copy latest fix atomically. Returns true if there was *new* data since last call.
bool GPS_UBX_GetLatestFix(gps_fix_basic_t *out);

#ifdef __cplusplus
}
#endif

#endif // GPS_UBX_H
