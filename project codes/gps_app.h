#ifndef GPS_APP_H
#define GPS_APP_H

#include "gps_ubx.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    bool     valid;

    // UTC time
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;

    // Position (deg / m)
    double   lat_deg;
    double   lon_deg;
    float    height_m;
    float    hmsl_m;

    // Fix info
    uint8_t  fixType;
    bool     fixOk;
    uint8_t  numSV_used;
    uint8_t  numSV_visible;

    // Derived kinematics from LLH (HNR 20 Hz)
    float    speed_mps;        // derived horiz speed
    float    speed_kmh;        // derived horiz speed
    float    heading_deg;      // derived heading (0..360)
    bool     heading_valid;    // true if speed >= 2 km/h when updated

    // Raw chip-reported (for debug / 비교용)
    float    raw_speed_mps;    // from gSpeed
    float    raw_heading_deg;  // from headMot

    // ★ 보드 공통 시간축(SysTick/HAL_GetTick) 기준의 호스트 timestamp
    uint32_t host_time_ms;   // HAL_GetTick() 결과, 이 fix가 갱신된 시점
    uint32_t tow_ms;           // GPS time-of-week [ms]

} app_gps_state_t;

// GPS 모듈 설정 + UART RX 시작
void APP_GPS_Init(void);

// 주기적으로 호출해서 내부 state 갱신 (예: main loop에서)
void APP_GPS_Update(void);

// 최신 상태를 복사해서 가져오기 (APP은 이 함수만 보면 됨)
bool APP_GPS_GetState(app_gps_state_t *out);

#ifdef __cplusplus
}
#endif

#endif // GPS_APP_H
