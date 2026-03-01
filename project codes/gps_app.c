// gps_app.c
#include "gps_app.h"
#include <string.h>
#include <math.h>

static volatile app_gps_state_t s_app_gps_state;

// 헤딩 저역필터 상태 (raw headMot → 부드러운 heading_deg)
static uint8_t  s_heading_initialized = 0u;
static float    s_heading_filtered_deg = 0.0f;
static uint32_t s_heading_prev_tow_ms  = 0u;

// LLH 기반 속도/헤딩 계산용 내부 상태
typedef struct
{
    uint8_t  has_prev;
    double   prev_lat_deg;
    double   prev_lon_deg;
    uint32_t prev_tow_ms;      // GPS time-of-week [ms]
    float    filt_speed_mps;   // 저역필터된 수평 속도
    float    last_heading_deg; // 마지막 유효 헤딩
} app_gps_dyn_state_t;

static app_gps_dyn_state_t s_dyn;

// deg 단위 위경도 두 점 사이의 수평 거리 [m]
static float gps_deg_distance_m(double lat1_deg, double lon1_deg,
                                double lat2_deg, double lon2_deg)
{
    const double R = 6371000.0;                 // 지구 반지름 [m]
    const double DEG2RAD = M_PI / 180.0;

    double lat1 = lat1_deg * DEG2RAD;
    double lon1 = lon1_deg * DEG2RAD;
    double lat2 = lat2_deg * DEG2RAD;
    double lon2 = lon2_deg * DEG2RAD;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double cos_lat = cos(0.5 * (lat1 + lat2));

    double dx = R * dlon * cos_lat;
    double dy = R * dlat;

    return (float)sqrt(dx * dx + dy * dy);
}

// 두 점 사이 방위각 [deg], 0=북, 시계방향 증가
static float gps_deg_heading_deg(double lat1_deg, double lon1_deg,
                                 double lat2_deg, double lon2_deg)
{
    const double DEG2RAD = M_PI / 180.0;
    const double RAD2DEG = 180.0 / M_PI;

    double lat1 = lat1_deg * DEG2RAD;
    double lon1 = lon1_deg * DEG2RAD;
    double lat2 = lat2_deg * DEG2RAD;
    double lon2 = lon2_deg * DEG2RAD;

    double dlon = lon2 - lon1;

    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);

    double brad = atan2(y, x);
    double bdeg = brad * RAD2DEG;
    if (bdeg < 0.0) {
        bdeg += 360.0;
    }
    return (float)bdeg;
}

void APP_GPS_Init(void)
{
    memset((void *)&s_app_gps_state, 0, sizeof(s_app_gps_state));
    memset((void *)&s_dyn, 0, sizeof(s_dyn));

    // UBX 모듈 설정 + UART RX 시작
    GPS_UBX_InitAndConfigure();
    GPS_UBX_StartUartRx();
}

void APP_GPS_Update(void)
{
    gps_fix_basic_t fix;

    if (!GPS_UBX_GetLatestFix(&fix)) {
        // 새 샘플 없음
        return;
    }

    app_gps_state_t next;
    memset(&next, 0, sizeof(next));

    next.valid = fix.valid;

    // UTC 시간
    next.year  = fix.year;
    next.month = fix.month;
    next.day   = fix.day;
    next.hour  = fix.hour;
    next.min   = fix.min;
    next.sec   = fix.sec;

    // 위치 (deg / m)
    next.lat_deg  = (double)fix.lat * 1e-7;
    next.lon_deg  = (double)fix.lon * 1e-7;
    next.height_m = (float)fix.height * 0.001f;
    next.hmsl_m   = (float)fix.hMSL   * 0.001f;

    // Fix 정보
    next.fixType       = fix.fixType;
    next.fixOk         = fix.fixOk;
    next.numSV_used    = fix.numSV_used;
    next.numSV_visible = fix.numSV_visible;

    // 칩에서 직접 주는 ground speed / heading (NAV-PVT 2 Hz 기준)
    next.raw_speed_mps   = (float)fix.gSpeed * 0.001f;  // mm/s → m/s
    next.raw_heading_deg = (float)fix.headMot * 1e-5f;  // 1e-5 deg → deg

    // 호스트 시간축 / GPS time-of-week
    next.host_time_ms = HAL_GetTick();
    next.tow_ms       = fix.iTOW_ms;

    // --------- 속도 / 헤딩 (GPS raw 기반) ---------
    if (fix.valid) {
        // 항상 칩에서 주는 gSpeed 사용
        next.speed_mps = next.raw_speed_mps;
        next.speed_kmh = next.raw_speed_mps * 3.6f;

        // raw headMot를 0~360도로 정규화
        float heading_raw = next.raw_heading_deg;
        if (heading_raw < 0.0f) {
            heading_raw += 360.0f;
        } else if (heading_raw >= 360.0f) {
            heading_raw -= 360.0f;
        }

        // 2 Hz NAV 기준 헤딩 저역필터 (원형 상에서 1차 IIR)
        //  - tau ~0.7 s: 2 Hz에서 2~3 샘플 정도에 걸쳐 부드럽게 이동
        //  - 해상도는 유지하면서 튐만 줄이는 쪽으로 튜닝
        if (!s_heading_initialized) {
            s_heading_initialized  = 1u;
            s_heading_filtered_deg = heading_raw;
            s_heading_prev_tow_ms  = next.tow_ms;
        } else {
            uint32_t dt_ms = next.tow_ms - s_heading_prev_tow_ms;
            if (dt_ms > 5000u) {
                // 너무 긴 갭은 정지로 보고 필터 step을 생략
                dt_ms = 0u;
            }

            float dt_s = (float)dt_ms * 0.001f;
            if (dt_s > 0.0f) {
                const float tau   = 0.7f;  // [s]
                float alpha       = dt_s / (tau + dt_s);
                if (alpha > 1.0f) alpha = 1.0f;
                if (alpha < 0.0f) alpha = 0.0f;

                // 원형(0~360)에서의 최소 각도 차이 [-180, 180]
                float diff = heading_raw - s_heading_filtered_deg;
                if (diff > 180.0f) {
                    diff -= 360.0f;
                } else if (diff < -180.0f) {
                    diff += 360.0f;
                }

                s_heading_filtered_deg += alpha * diff;

                // 다시 0~360 정규화
                if (s_heading_filtered_deg < 0.0f) {
                    s_heading_filtered_deg += 360.0f;
                } else if (s_heading_filtered_deg >= 360.0f) {
                    s_heading_filtered_deg -= 360.0f;
                }
            }

            s_heading_prev_tow_ms = next.tow_ms;
        }

        next.heading_deg   = s_heading_filtered_deg;
        // ★ 20 km/h 이상에서만 헤딩 표시
        next.heading_valid = (next.speed_kmh >= 20.0f);
    } else {
        // fix invalid → 속도/헤딩은 0, invalid 로 처리
        next.speed_mps     = 0.0f;
        next.speed_kmh     = 0.0f;
        next.heading_deg   = 0.0f;
        next.heading_valid = false;

        // 헤딩 필터 상태도 리셋
        s_heading_initialized  = 0u;
        s_heading_filtered_deg = 0.0f;
        s_heading_prev_tow_ms  = 0u;
    }



    // 전역 상태에 publish (임계구역)
    __disable_irq();
    s_app_gps_state = next;
    __enable_irq();
}


bool APP_GPS_GetState(app_gps_state_t *out)
{
    if (out == NULL) {
        return false;
    }

    __disable_irq();
    *out = s_app_gps_state;
    __enable_irq();

    return out->valid;
}
