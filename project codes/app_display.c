// app_display.c
#include "app_display.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <buzzer.h>
#include <max7219.h>


// app_display.c 상단
volatile app_display_mode_t g_display_mode = APP_DISPLAY_SPEED_AND_HEADING;
static app_display_mode_t s_last_mode      = APP_DISPLAY_SPEED_AND_HEADING;


// ★ 새 전역: 현재 뱅크 (1필드 / 2필드 / 0~100)
volatile app_display_bank_t g_display_bank = APP_DISPLAY_BANK_DUAL;


// 최근 모드 변경이 사용자 버튼(NextMode/SetMode)에서 온 것인지 표시
static uint8_t s_mode_change_from_button   = 0u;

// ★ overspeed 진입 직전 모드 저장용
static app_display_mode_t s_mode_before_overspeed = APP_DISPLAY_SAT_STATUS;

// 100ms 타이머 기반 블링크 상태 (0/1 토글)
volatile uint8_t g_blink_2hz = 0;  // 2 Hz → 500ms마다 토글
volatile uint8_t g_blink_5hz = 0;  // 5 Hz → 100ms마다 토글

// overspeed 상태 등도 여기 같이 둘 예정 (4번에서 사용)
static uint8_t s_overspeed_active = 0u;
static uint8_t s_speed_warning_level = 0u; // 0: <150, 1: ≥150, 2: ≥160, 3: ≥170
static uint8_t s_fix_ready_prev = 0u;



// 2 Hz 화면 속도 클램프 상태 (0=off, 1=on)
// - 데이터 파이프라인은 항상 20 Hz HNR 기준으로 유지하고,
//   이 플래그가 1일 때만 화면 갱신 속도만 2 Hz로 제한한다.
static uint8_t  s_speed_2hz_clamp_enabled  = 0u;
static uint32_t s_speed_2hz_last_update_ms = 0u;
static float    s_speed_2hz_last_speed_mps = 0.0f;

void APP_Display_SetSpeedClamp2HzEnabled(bool enabled)
{
    s_speed_2hz_clamp_enabled = enabled ? 1u : 0u;
}


// 자동 밝기 (SUNRISE/SUNSET 기반) 관련 상태
static uint8_t s_global_brightness_level = 2u;   // 1~3 (설정값 그대로 반영)
static uint8_t s_current_brightness_step = 0u;   // 0~3 (밤/새벽/낮/해질녘)
static uint8_t s_current_intensity       = 0x08; // MAX7219 REG_INTENSITY 값

// 일출/일몰 캐시
typedef struct
{
    uint8_t  valid;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    int16_t  sunrise_min_local;  // 로컬 시각 기준 [분]
    int16_t  sunset_min_local;   // 로컬 시각 기준 [분]
} sun_event_cache_t;

static sun_event_cache_t s_sun_cache = {0u, 0u, 0u, 0u, -1, -1};

// GLOBAL(1~3) × 시간대(0~3) → INTENSITY(0x00~0x0F)
// step 0 = 밤, 1 = 해뜨는 주변(±30분), 2 = 낮, 3 = 해지는 주변(±30분)
static const uint8_t s_brightness_table[3][4] =
{
    // GLOBAL = 1 (가장 어둡게)
    { 1u,  2u,  5u,  15u },
    // GLOBAL = 2 (중간)
    { 1u,  4u,  7u, 15u },
    // GLOBAL = 3 (가장 밝게)
    { 2u,  5u, 7u, 15u }
};



// AUTO 모드 설정값
#define AUTO_ALT_RATE_THRESHOLD_MPS      1.0f   // [m/s] 이상이면 고도 변화로 판단
#define AUTO_ALT_RATE_MIN_DURATION_MS    3000u  // [ms] 위 상태가 지속되어야 하는 최소 시간
#define AUTO_STOP_SPEED_THRESHOLD_MPS    1.0f   // [m/s] 미만이면 정지로 간주
#define AUTO_STOP_HOLD_TIME_MS           5000u  // [ms] 위 속도가 지속될 때 정지 레지스터 시간

// AUTO 모드 / 위도·경도 표시 관련 내부 상태
static uint8_t             s_auto_mode_enabled            = 0u;
static uint8_t             s_auto_active                  = 0u;
static app_display_mode_t  s_auto_prev_mode               = APP_DISPLAY_SAT_STATUS;
static app_display_mode_t  s_auto_prev_mode_before_stop   = APP_DISPLAY_SAT_STATUS;
static app_display_mode_t  s_auto_active_mode             = APP_DISPLAY_SAT_STATUS;

static uint8_t             s_auto_has_last_alt            = 0u;
static float               s_auto_last_alt_m              = 0.0f;
static uint32_t            s_auto_last_alt_ms             = 0u;

static uint8_t             s_auto_alt_cond_active         = 0u;
static uint32_t            s_auto_alt_cond_start_ms       = 0u;
static uint8_t             s_auto_alt_pick_toggle         = 0u;

static uint8_t             s_auto_stop_cond_active        = 0u;
static uint32_t            s_auto_stop_cond_start_ms      = 0u;

// LAT/LON 모드 토글 상태
static uint8_t             s_latlon_show_lat              = 1u;
static uint32_t            s_latlon_last_toggle_ms        = 0u;






// 내부 상태
typedef struct
{
    uint8_t  initialized;

    app_gps_state_t last_gps;
    uint8_t  has_last;
    uint32_t last_host_ms;

    // 트립 누적
    float    trip_distance_m;      // [m]
    float    trip_top_speed_kmh;   // [km/h]
    float    trip_avg_speed_kmh;   // [km/h]
    uint32_t trip_time_ms_total;   // 전원 이후 전체 시간

    // 경사도 (grade, %)
    float    grade_filtered;       // [%]

    // 0-100 km/h 테스트
    uint8_t  zto100_running;
    uint8_t  zto100_done;
    uint32_t zto100_start_ms;
    float    zto100_time_s;
    float    zto100_speed_kmh;

} app_display_state_t;

static app_display_state_t s_disp;
static int8_t s_timezone_hours = 9;

// ----------------- 유틸리티 / 헬퍼 -----------------
// 내부 헬퍼 프로토타입
static float get_speed_kmh_for_feature(const app_gps_state_t *gps, uint8_t use_hnr_always_for_feature);
static float get_speed_mps_for_feature(const app_gps_state_t *gps, uint8_t use_hnr_always_for_feature);
static float get_heading_for_feature(const app_gps_state_t *gps,
                                     uint8_t use_hnr_always_for_feature,
                                     bool *out_valid);
// ----------------- 뱅크별 모드 테이블 -----------------

#define BANK_MODE_COUNT(arr) ((uint8_t)(sizeof(arr) / sizeof((arr)[0])))

// 1필드 전용 뱅크에서 순환할 모드들
//  - 여기서 단일 필드 화면들만 넣어줌
//  - 요구사항: SAT_STATUS / TRIP_TIME / LOCAL_TIME / LATLON 포함
static const app_display_mode_t s_bank_single_modes[] =
{
    APP_DISPLAY_SAT_STATUS,
    APP_DISPLAY_SPEED,
    APP_DISPLAY_TOP_SPEED,
    APP_DISPLAY_AVG_SPEED,
    APP_DISPLAY_ALTITUDE,
    APP_DISPLAY_HEADING,
    APP_DISPLAY_GRADE,
    APP_DISPLAY_DISTANCE,
    APP_DISPLAY_TRIP_TIME,   // ★ 필수 포함
    APP_DISPLAY_LOCAL_TIME,  // ★ 필수 포함
    APP_DISPLAY_LOCAL_DATE,
    APP_DISPLAY_LATLON       // ★ 필수 포함
};

// 2필드 전용 뱅크에서 순환할 모드들
//  - 기본은 2필드 UI (SPD+GRD / SPD+HDG / SPD+ALT)
//  - 요구사항대로 SAT_STATUS / TRIP_TIME / LOCAL_TIME / LATLON도
//    이 뱅크 순환에 포함시켜줌
static const app_display_mode_t s_bank_dual_modes[] =
{
    APP_DISPLAY_SPEED_AND_GRADE,
    APP_DISPLAY_SPEED_AND_HEADING,
    APP_DISPLAY_SPEED_AND_ALTITUDE,

    // ★ 공통으로 항상 포함해야 하는 페이지들 (단일필드지만 강제 포함)
    APP_DISPLAY_SAT_STATUS,
    APP_DISPLAY_TRIP_TIME,
    APP_DISPLAY_LOCAL_TIME,
    APP_DISPLAY_LATLON
};

// 뱅크 바꿀 때 잠깐 보여주는 라벨
static void show_bank_label(app_display_bank_t bank)
{
    const char *txt = NULL;

    switch (bank)
    {
    case APP_DISPLAY_BANK_SINGLE:
        txt = "1 FIELd";   // 1필드 뱅크
        break;

    case APP_DISPLAY_BANK_DUAL:
        txt = "2 FIELd";   // 2필드 뱅크
        break;

    case APP_DISPLAY_BANK_ZTO100:
        txt = "0 to 100";  // 0~100 제로백 뱅크
        break;

    default:
        txt = "";
        break;
    }

    if (txt && txt[0] != '\0')
    {
        max7219_Clean();
        max7219_WriteStringInRange((char *)txt, 0, 7, false);
        HAL_Delay(600u);   // 0.6초 정도 보여주고 원래 모드로 돌아가기
    }
}


// AUTO 모드 상태 업데이트
static void update_auto_mode(const app_gps_state_t *gps,
                             uint8_t fix_ready,
                             app_display_mode_t *mode)
{
    // ★ 오버스피드가 켜져 있으면 AUTO는 아예 건드리지 않음
    if (s_overspeed_active) {
        s_auto_active           = 0u;
        s_auto_has_last_alt     = 0u;
        s_auto_alt_cond_active  = 0u;
        s_auto_stop_cond_active = 0u;

        *mode = APP_DISPLAY_SPEED;
        return;
    }


    if (!s_auto_mode_enabled) {
        // AUTO 전체 비활성화 → 상태 초기화만
        s_auto_active           = 0u;
        s_auto_has_last_alt     = 0u;
        s_auto_alt_cond_active  = 0u;
        s_auto_stop_cond_active = 0u;
        return;
    }

    if (!gps || !fix_ready) {
        // 유효한 FIX 없으면 오토 모드도 강제 해제
        if (s_auto_active) {
            *mode        = g_display_mode;
            s_auto_active = 0u;
        }
        s_auto_has_last_alt     = 0u;
        s_auto_alt_cond_active  = 0u;
        s_auto_stop_cond_active = 0u;
        return;
    }

    const uint32_t now_ms = gps->host_time_ms;

    // --------- 고도 변화율 기반 ALT/GRD 자동 모드 ---------
    if (!s_auto_has_last_alt) {
        s_auto_has_last_alt = 1u;
        s_auto_last_alt_m   = gps->hmsl_m;
        s_auto_last_alt_ms  = now_ms;
    } else {
        uint32_t dt_ms = now_ms - s_auto_last_alt_ms;
        if (dt_ms > 0u) {
            float dz   = gps->hmsl_m - s_auto_last_alt_m;
            float rate = fabsf(dz) / ((float)dt_ms * 0.001f); // m/s

            s_auto_last_alt_m  = gps->hmsl_m;
            s_auto_last_alt_ms = now_ms;

            if (rate >= AUTO_ALT_RATE_THRESHOLD_MPS) {
                if (!s_auto_alt_cond_active) {
                    s_auto_alt_cond_active   = 1u;
                    s_auto_alt_cond_start_ms = now_ms;
                } else if ((now_ms - s_auto_alt_cond_start_ms) >= AUTO_ALT_RATE_MIN_DURATION_MS) {
                    // 고도 변화 조건 충족, 정지 조건이 우선이면 패스
                    if (!s_auto_stop_cond_active) {
                        if (!s_auto_active ||
                            (s_auto_active_mode != APP_DISPLAY_SPEED_AND_ALTITUDE &&
                             s_auto_active_mode != APP_DISPLAY_SPEED_AND_GRADE)) {

                            s_auto_prev_mode = *mode;

                            // ALT/SPD <-> GRD/SPD 번갈아 선택 (랜덤 대용)
                            s_auto_alt_pick_toggle ^= 1u;
                            s_auto_active_mode = s_auto_alt_pick_toggle
                                ? APP_DISPLAY_SPEED_AND_ALTITUDE
                                : APP_DISPLAY_SPEED_AND_GRADE;
                        }

                        *mode         = s_auto_active_mode;
                        s_auto_active = 1u;
                    }
                }
            } else {
                // 고도 변화율이 임계치 아래로 내려감 → ALT/GRD 자동 모드 해제
                s_auto_alt_cond_active   = 0u;
                s_auto_alt_cond_start_ms = now_ms;

                if (s_auto_active &&
                    (s_auto_active_mode == APP_DISPLAY_SPEED_AND_ALTITUDE ||
                     s_auto_active_mode == APP_DISPLAY_SPEED_AND_GRADE) &&
                    !s_auto_stop_cond_active) {
                    *mode         = s_auto_prev_mode;
                    s_auto_active = 0u;
                }
            }
        }
    }

    // --------- 속도 기반 정지 감지 → 시간 모드 ---------
    float v_mps = get_speed_mps_for_feature(gps, 0u);
    if (v_mps < AUTO_STOP_SPEED_THRESHOLD_MPS) {
        if (!s_auto_stop_cond_active) {
            s_auto_stop_cond_active   = 1u;
            s_auto_stop_cond_start_ms = now_ms;
            s_auto_prev_mode_before_stop = *mode;
        } else if ((now_ms - s_auto_stop_cond_start_ms) >= AUTO_STOP_HOLD_TIME_MS) {
            *mode              = APP_DISPLAY_LOCAL_TIME;
            s_auto_active_mode = APP_DISPLAY_LOCAL_TIME;
            s_auto_active      = 1u;
        }
    } else {
        if (s_auto_stop_cond_active) {
            s_auto_stop_cond_active = 0u;
            // 정지 기반으로 시간 모드에 있었으면 이전 모드로 복귀
            if (s_auto_active_mode == APP_DISPLAY_LOCAL_TIME) {
                *mode = s_auto_prev_mode_before_stop;
                // 이전 모드가 자동 ALT/GRD 모드였는지는 상관없이 그대로 복귀
                s_auto_active = (*mode != g_display_mode) ? 1u : 0u;
            }
        }
    }
}

// ----------------- SUNRISE / SUNSET 기반 자동 밝기 -----------------

static int is_leap_year_uint16(uint16_t year)
{
    return ((year % 4u == 0u && year % 100u != 0u) || (year % 400u == 0u));
}

static int day_of_year_uint16(uint16_t year, uint8_t month, uint8_t day)
{
    // month: 1~12, day: 1~31
    static const uint16_t days_before_month[13] =
    {
        0u,   // dummy
        0u,   // Jan
        31u,  // Feb
        59u,  // Mar
        90u,  // Apr
        120u, // May
        151u, // Jun
        181u, // Jul
        212u, // Aug
        243u, // Sep
        273u, // Oct
        304u, // Nov
        334u  // Dec
    };

    if (month < 1u || month > 12u || day < 1u || day > 31u) {
        return -1;
    }

    int n = (int)days_before_month[month] + (int)day;
    if (month > 2u && is_leap_year_uint16(year)) {
        n += 1;
    }
    return n;
}

static double deg_sin(double deg)
{
    return sin(deg * (3.14159265358979323846 / 180.0));
}

static double deg_cos(double deg)
{
    return cos(deg * (3.14159265358979323846 / 180.0));
}

static double deg_tan(double deg)
{
    return tan(deg * (3.14159265358979323846 / 180.0));
}

static double deg_asin(double x)
{
    return asin(x) * (180.0 / 3.14159265358979323846);
}

static double deg_acos(double x)
{
    return acos(x) * (180.0 / 3.14159265358979323846);
}

static double deg_atan(double x)
{
    return atan(x) * (180.0 / 3.14159265358979323846);
}

static double normalize_deg_360(double deg)
{
    while (deg < 0.0) {
        deg += 360.0;
    }
    while (deg >= 360.0) {
        deg -= 360.0;
    }
    return deg;
}

// NOAA 알고리즘 기반 일출/일몰 계산 (분 단위, 로컬 타임존 기준)
static uint8_t compute_sun_event_minutes_local(
    uint16_t year,
    uint8_t  month,
    uint8_t  day,
    double   latitude_deg,
    double   longitude_deg,
    int8_t   timezone_hours,
    uint8_t  is_sunrise,
    int     *out_minutes_local)
{
    int N = day_of_year_uint16(year, month, day);
    if (N <= 0) {
        return 0u;
    }

    const double zenith = 90.833;  // civil sunrise/sunset

    double lngHour = longitude_deg / 15.0;
    double t = (double)N + (((is_sunrise ? 6.0 : 18.0) - lngHour) / 24.0);

    // Sun's mean anomaly
    double M = (0.9856 * t) - 3.289;

    // Sun's true longitude
    double L = M + (1.916 * deg_sin(M)) + (0.020 * deg_sin(2.0 * M)) + 282.634;
    L = normalize_deg_360(L);

    // Right ascension
    double RA = deg_atan(0.91764 * deg_tan(L));
    RA = normalize_deg_360(RA);

    double Lquadrant  = floor(L / 90.0) * 90.0;
    double RAquadrant = floor(RA / 90.0) * 90.0;
    RA = RA + (Lquadrant - RAquadrant);
    RA /= 15.0; // RA in hours

    // Declination
    double sinDec = 0.39782 * deg_sin(L);
    double cosDec = cos(asin(sinDec));

    // Local hour angle
    double cosH = (deg_cos(zenith) - (sinDec * deg_sin(latitude_deg))) /
                  (cosDec * deg_cos(latitude_deg));

    if (cosH > 1.0 || cosH < -1.0) {
        // 극지방 등: sunrise/sunset 없음
        return 0u;
    }

    double H;
    if (is_sunrise) {
        H = 360.0 - deg_acos(cosH);
    } else {
        H = deg_acos(cosH);
    }
    H /= 15.0; // hour

    double T  = H + RA - (0.06571 * t) - 6.622;
    double UT = T - lngHour;

    while (UT < 0.0) {
        UT += 24.0;
    }
    while (UT >= 24.0) {
        UT -= 24.0;
    }

    double localT = UT + (double)timezone_hours;
    while (localT < 0.0) {
        localT += 24.0;
    }
    while (localT >= 24.0) {
        localT -= 24.0;
    }

    int minutes = (int)(localT * 60.0 + 0.5);
    if (minutes < 0) {
        minutes = 0;
    }
    if (minutes >= (24 * 60)) {
        minutes = (24 * 60) - 1;
    }

    if (out_minutes_local) {
        *out_minutes_local = minutes;
    }
    return 1u;
}

static uint8_t compute_sunrise_sunset_minutes_local(
    uint16_t year,
    uint8_t  month,
    uint8_t  day,
    double   latitude_deg,
    double   longitude_deg,
    int8_t   timezone_hours,
    int     *out_sunrise_minutes,
    int     *out_sunset_minutes)
{
    if (!out_sunrise_minutes || !out_sunset_minutes) {
        return 0u;
    }

    int sunrise_min = -1;
    int sunset_min  = -1;

    uint8_t ok_rise = compute_sun_event_minutes_local(
        year, month, day, latitude_deg, longitude_deg, timezone_hours,
        1u, &sunrise_min);
    uint8_t ok_set  = compute_sun_event_minutes_local(
        year, month, day, latitude_deg, longitude_deg, timezone_hours,
        0u, &sunset_min);

    if (!ok_rise || !ok_set) {
        return 0u;
    }

    *out_sunrise_minutes = sunrise_min;
    *out_sunset_minutes  = sunset_min;
    return 1u;
}

// UTC 시간 + 타임존 → 로컬 시각 [분]
static int utc_to_local_minutes(uint8_t hour_utc, uint8_t min_utc, int8_t tz_hours)
{
    int total = (int)hour_utc * 60 + (int)min_utc + (int)tz_hours * 60;
    while (total < 0) {
        total += 24 * 60;
    }
    while (total >= 24 * 60) {
        total -= 24 * 60;
    }
    return total;
}

// 일출/일몰과의 관계로 0~3 단계 결정
// 0: 완전 밤, 1: 해 뜨는 주변(±30분), 2: 낮, 3: 해 지는 주변(±30분)
static uint8_t pick_brightness_step(int local_minute, int sunrise_minute, int sunset_minute)
{
    const int TWILIGHT_MIN = 30;

    if (sunrise_minute < 0 || sunset_minute < 0) {
        return 2u; // sunrise/sunset 계산 실패 → 기본: 낮 밝기
    }

    if (sunrise_minute >= (24 * 60) || sunset_minute >= (24 * 60) ||
        sunrise_minute >= sunset_minute) {
        // 비정상 값 → 안전하게 낮 단계
        return 2u;
    }

    int sunrise_start = sunrise_minute - TWILIGHT_MIN;
    int sunrise_end   = sunrise_minute + TWILIGHT_MIN;
    int sunset_start  = sunset_minute - TWILIGHT_MIN;
    int sunset_end    = sunset_minute + TWILIGHT_MIN;

    if (sunrise_start < 0) {
        sunrise_start = 0;
    }
    if (sunset_end >= (24 * 60)) {
        sunset_end = (24 * 60) - 1;
    }

    // 0: 완전 밤
    if (local_minute < sunrise_start || local_minute >= sunset_end) {
        return 0u;
    }

    // 1: 해 뜨는 주변
    if (local_minute < sunrise_end) {
        return 1u;
    }

    // 2: 낮
    if (local_minute < sunset_start) {
        return 2u;
    }

    // 3: 해 지는 주변
    return 3u;
}

// GPS + 글로벌 밝기 레벨 → MAX7219 INTENSITY 자동 설정
static void update_auto_brightness(const app_gps_state_t *gps, uint8_t fix_ready)
{
    if (!gps || !gps->valid || !fix_ready) {
        // 유효한 FIX 없으면 자동 밝기 적용 불가 → 현재 값 유지
        return;
    }

    if (s_global_brightness_level < 1u) {
        s_global_brightness_level = 1u;
    } else if (s_global_brightness_level > 3u) {
        s_global_brightness_level = 3u;
    }

    // 날짜 바뀌었으면 일출/일몰 다시 계산
    if (!s_sun_cache.valid ||
        s_sun_cache.year  != gps->year ||
        s_sun_cache.month != gps->month ||
        s_sun_cache.day   != gps->day)
    {
        int sunrise_min = -1;
        int sunset_min  = -1;

        uint8_t ok = compute_sunrise_sunset_minutes_local(
            gps->year,
            gps->month,
            gps->day,
            gps->lat_deg,
            gps->lon_deg,
            s_timezone_hours,
            &sunrise_min,
            &sunset_min);

        if (ok) {
            s_sun_cache.valid             = 1u;
            s_sun_cache.year              = gps->year;
            s_sun_cache.month             = gps->month;
            s_sun_cache.day               = gps->day;
            s_sun_cache.sunrise_min_local = (int16_t)sunrise_min;
            s_sun_cache.sunset_min_local  = (int16_t)sunset_min;
        } else {
            s_sun_cache.valid = 0u;
        }
    }

    int local_min = utc_to_local_minutes(gps->hour, gps->min, s_timezone_hours);

    uint8_t step = 2u;
    if (s_sun_cache.valid) {
        step = pick_brightness_step(
            local_min,
            (int)s_sun_cache.sunrise_min_local,
            (int)s_sun_cache.sunset_min_local);
    }

    if (step > 3u) {
        step = 3u;
    }
    s_current_brightness_step = step;

    uint8_t global_index = s_global_brightness_level - 1u;
    if (global_index > 2u) {
        global_index = 1u;
    }

    uint8_t new_intensity = s_brightness_table[global_index][step];

    if (new_intensity != s_current_intensity) {
        s_current_intensity = new_intensity;
        max7219_SetIntensivity(new_intensity);
    }
}


static void update_speed_warnings_and_overspeed(const app_gps_state_t *gps)
{
    uint8_t prev_overspeed = s_overspeed_active;

    if (!gps || !gps->valid || !gps->fixOk || (gps->fixType < 2u)) {
        // FIX 없으면 경고/오버스피드 모두 리셋
        s_overspeed_active    = 0u;
        s_speed_warning_level = 0u;

        // 만약 직전에 오버스피드였다면 모드 복구
        if (prev_overspeed) {
            g_display_mode = s_mode_before_overspeed;
        }
        return;
    }

    float v = get_speed_kmh_for_feature(gps, 0u);

    // --- 속도 경고 레벨 업데이트 (간단한 히스테리시스) ---
    uint8_t level = s_speed_warning_level;

    // 상향 경계: 150 / 160 / 170 km/h
    if (v >= 170.0f && level < 3u) {
        level = 3u;
        Buzzer_PlaySequence(BEEP_SEQ_WARNING3);
    } else if (v >= 160.0f && level < 2u) {
        level = 2u;
        Buzzer_PlaySequence(BEEP_SEQ_WARNING2);
    } else if (v >= 150.0f && level < 1u) {
        level = 1u;
        Buzzer_PlaySequence(BEEP_SEQ_WARNING1);
    }
    // 하향 리셋 경계: 145 km/h 미만이면 완전히 리셋
    else if (v < 145.0f) {
        level = 0u;
    }

    s_speed_warning_level = level;


    // --- 오버스피드 상태 관리 + 디스플레이 모드 제어 ---
    uint8_t new_overspeed = s_overspeed_active;

    if (!s_overspeed_active && v > 160.0f) {
        // 160km/h 초과로 처음 진입
        new_overspeed = 1u;
    } else if (s_overspeed_active && v < 160.0f) {
        // 160km/h 미만으로 내려오면서 해제
        new_overspeed = 0u;
    }

    s_overspeed_active = new_overspeed;

    if (!prev_overspeed && new_overspeed) {
        // 오버스피드 진입 시점: 현재 모드 저장
        s_mode_before_overspeed = g_display_mode;
    }

    if (new_overspeed) {
        // 오버스피드 동안에는 항상 단일 SPEED 모드로 강제
        g_display_mode = APP_DISPLAY_SPEED;
    } else if (prev_overspeed && !new_overspeed) {
        // 오버스피드 해제 시 이전 모드로 복귀
        g_display_mode = s_mode_before_overspeed;
    }
}



// app_display.c 상단 쪽 (이미 있는 is_fix_ready를 이걸로 완전히 교체)
static bool is_fix_ready(const app_gps_state_t *gps)
{
    if (!gps) {
        return false;
    }

    // 날짜/시간 유효 + Fix OK + 2D 이상이면 "실제 주행에 쓸 수 있는 상태"로 간주
    return gps->valid && gps->fixOk && (gps->fixType >= 2u);
}

// 속도/헤딩 선택 유틸리티
// 속도/헤딩 선택 유틸리티
//  - GPS 파이프라인은 항상 20 Hz HNR/LLH 기준으로 동작
//  - 2 Hz 클램프가 켜져 있을 때만 화면 갱신 속도를 2 Hz로 제한
static float apply_speed_2hz_clamp(float speed_mps,
                                   uint8_t use_hnr_always_for_feature)
{
    // 클램프가 꺼져 있으면 그대로 통과
    if (!s_speed_2hz_clamp_enabled) {
        return speed_mps;
    }

    // 제로백 등 고속 응답이 필요한 기능은 항상 풀레이트
    if (use_hnr_always_for_feature != 0u) {
        return speed_mps;
    }

    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - s_speed_2hz_last_update_ms;

    // 500 ms 이상 지났을 때만 새 값 반영 → 약 2 Hz
    if (dt >= 500u) {
        s_speed_2hz_last_update_ms = now;
        s_speed_2hz_last_speed_mps = speed_mps;
    }

    return s_speed_2hz_last_speed_mps;
}

static float get_speed_mps_for_feature(const app_gps_state_t *gps,
                                       uint8_t use_hnr_always_for_feature)
{
    if (!gps || !gps->valid) {
        return 0.0f;
    }

    // gps_app에서 이미 HNR/LLH 기반으로 필터링 된 속도 사용
    float v_mps = gps->speed_mps;

    return apply_speed_2hz_clamp(v_mps, use_hnr_always_for_feature);
}

static float get_speed_kmh_for_feature(const app_gps_state_t *gps,
                                       uint8_t use_hnr_always_for_feature)
{
    float v_mps = get_speed_mps_for_feature(gps, use_hnr_always_for_feature);
    return v_mps * 3.6f;
}

static float get_heading_for_feature(const app_gps_state_t *gps,
                                     uint8_t use_hnr_always_for_feature,
                                     bool *out_valid)
{
    (void)use_hnr_always_for_feature;  // 현재는 heading 쪽은 모드 분기 없음

    if (out_valid) {
        *out_valid = false;
    }

    if (!gps || !gps->valid) {
        return 0.0f;
    }

    // gps_app에서 미리 파생된 heading / valid 플래그 사용
    if (out_valid) {
        *out_valid = gps->heading_valid;
    }
    return gps->heading_deg;
}



void APP_Display_BlinkTick_100ms(void)
{
    static uint8_t cnt2 = 0;

    // 5 Hz: 100 ms마다 on/off 토글
    g_blink_5hz = (uint8_t)!g_blink_5hz;

    // 2 Hz: 500 ms마다 토글 (100ms × 5)
    cnt2++;
    if (cnt2 >= 5u) {
        cnt2 = 0;
        g_blink_2hz = (uint8_t)!g_blink_2hz;
    }
}


// value(양수)를 XXX.X 형식으로 start_pos ~ start_pos+3 에 출력
// 예: start_pos=4 → [4]=hundreds, [5]=tens, [6]=ones(DP on), [7]=1/10
// max_x10: 값*10 의 최대치 (예: 9999 => 999.9)
static void ui_print_fixed_1(float value, uint8_t start_pos, uint16_t max_x10)
{
    if (value < 0.0f) {
        value = -value;
    }

    float max_val = (float)max_x10 / 10.0f;
    if (value > max_val) {
        value = max_val;
    }

    uint16_t v10 = (uint16_t)(value * 10.0f + 0.5f);
    if (v10 > max_x10) {
        v10 = max_x10;
    }

    uint8_t d[4];
    d[0] = (uint8_t)((v10 / 1000u) % 10u);
    d[1] = (uint8_t)((v10 / 100u) % 10u);
    d[2] = (uint8_t)((v10 / 10u)  % 10u);
    d[3] = (uint8_t)(v10 % 10u);

    // 리딩 제로 제거
    bool show[4] = { false, false, true, true };
    if (v10 >= 1000u) {
        show[0] = true;
        show[1] = true;
        show[2] = true;
    } else if (v10 >= 100u) {
        show[1] = true;
        show[2] = true;
    } else {
        show[2] = true;
    }

    for (uint8_t i = 0; i < 4; ++i) {
        uint8_t pos = (uint8_t)(start_pos + i);
        char ch = show[i] ? (char)('0' + d[i]) : ' ';
        bool point = (i == 2); // ones 자리에서 dot (XXX.X)
        max7219_WriteCharAt(pos, ch, point);
    }
}

// value(>=0)를 정수로 start~end 에 우측 정렬 출력
static void ui_print_uint_right(uint32_t value, uint8_t start, uint8_t end)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)value);
    max7219_WriteStringInRange(buf, start, end, true);
}

// -XX.X 형식 (GRd 모드) 출력용
// sign_pos: '-' 위치, digit_pos: "XX.X" 세 자리의 시작 위치
static void ui_print_signed_grade(float grade, uint8_t sign_pos, uint8_t digit_pos)
{
    if (grade > 99.9f)  grade = 99.9f;
    if (grade < -99.9f) grade = -99.9f;

    char sign = ' ';
    if (grade < -0.05f) {
        sign = '-';
    }

    max7219_WriteCharAt(sign_pos, sign, false);

    float mag = fabsf(grade);
    uint16_t v10 = (uint16_t)(mag * 10.0f + 0.5f);
    if (v10 > 999u) {
        v10 = 999u;
    }

    uint8_t d[3];
    d[0] = (uint8_t)((v10 / 100u) % 10u);
    d[1] = (uint8_t)((v10 / 10u)  % 10u);
    d[2] = (uint8_t)(v10 % 10u);

    bool show0 = (v10 >= 100u);
    bool show1 = true;  // 일의 자리 0까지 항상 표시 (0.0 포함)

    uint8_t pos = digit_pos;
    max7219_WriteCharAt(pos++, show0 ? (char)('0' + d[0]) : ' ', false);
    max7219_WriteCharAt(pos++, show1 ? (char)('0' + d[1]) : ' ', true);  // DP on
    max7219_WriteCharAt(pos,   (char)('0' + d[2]), false);
}

// 경사도 0.0 형식 (절대값, 부호 없음) 출력용
// 예: pos0=0, pos1=1 → [0]=tens(DP on) "0.", [1]=ones "0" → "0.0"
static void ui_print_grade_abs_0p0(float grade, uint8_t pos0, uint8_t pos1)
{
    if (grade < 0.0f) {
        grade = -grade;
    }

    // 0.0 ~ 9.9 [%]로 제한
    if (grade > 9.9f) {
        grade = 9.9f;
    }

    uint16_t v10 = (uint16_t)(grade * 10.0f + 0.5f);
    if (v10 > 99u) {
        v10 = 99u;
    }

    uint8_t tens = (uint8_t)(v10 / 10u);
    uint8_t ones = (uint8_t)(v10 % 10u);

    char t_ch = (char)('0' + tens);
    char o_ch = (char)('0' + ones);

    // 첫 번째 자리(dp on) 뒤에 dot 을 찍어서 "T. O" → "T.O"
    max7219_WriteCharAt(pos0, t_ch, true);
    max7219_WriteCharAt(pos1, o_ch, false);
}


// 경사도 정수 2자리 (속도+경사 모드) 출력용
// 패턴: sign tens ones
static void ui_print_grade_int2(float grade, uint8_t sign_pos, uint8_t tens_pos, uint8_t ones_pos)
{
    int g = (int)(grade >= 0.0f ? grade + 0.5f : grade - 0.5f);
    if (g > 99)  g = 99;
    if (g < -99) g = -99;

    char sign = (g < 0) ? '-' : ' ';
    int mag = (g < 0) ? -g : g;

    uint8_t tens = (uint8_t)(mag / 10);
    uint8_t ones = (uint8_t)(mag % 10);

    char t_ch = (tens > 0) ? (char)('0' + tens) : ' ';
    char o_ch = (char)('0' + ones);

    max7219_WriteCharAt(sign_pos, sign, false);
    max7219_WriteCharAt(tens_pos, t_ch,  false);
    max7219_WriteCharAt(ones_pos, o_ch,  false);
}

// 속도 정수 3자리 (XXX) 출력
static void ui_print_speed_int3(float speed_kmh, uint8_t start_pos)
{
    if (speed_kmh < 0.0f)   speed_kmh = 0.0f;
    if (speed_kmh > 199.0f) speed_kmh = 199.0f;

    uint16_t v = (uint16_t)(speed_kmh + 0.5f);

    uint8_t h = (uint8_t)((v / 100u) % 10u);
    uint8_t t = (uint8_t)((v / 10u)  % 10u);
    uint8_t o = (uint8_t)(v % 10u);

    char h_ch = (h > 0) ? (char)('0' + h) : ' ';
    char t_ch = ((h > 0) || (t > 0)) ? (char)('0' + t) : ' ';
    char o_ch = (char)('0' + o);

    max7219_WriteCharAt(start_pos + 0u, h_ch, false);
    max7219_WriteCharAt(start_pos + 1u, t_ch, false);
    max7219_WriteCharAt(start_pos + 2u, o_ch, false);
}

// 타임존 계산용: 월별 일수
static int ui_days_in_month(int year, int month)
{
    static const int days[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
    int d = days[month - 1];
    int leap = ((year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0)));
    if (month == 2 && leap) {
        d = 29;
    }
    return d;
}

// GPS UTC + 타임존 → 로컬 시간/날짜
static bool ui_compute_local_datetime(const app_gps_state_t *gps,
                                      int8_t tz_hours,
                                      int *out_year, int *out_month, int *out_day,
                                      int *out_hour, int *out_min, int *out_sec)
{
    if (!gps || !gps->valid) {
        return false;
    }

    int year  = (int)gps->year;
    int month = (int)gps->month;
    int day   = (int)gps->day;
    int hour  = (int)gps->hour + (int)tz_hours;
    int min   = (int)gps->min;
    int sec   = (int)gps->sec;

    while (hour >= 24) {
        hour -= 24;
        day += 1;
        int dim = ui_days_in_month(year, month);
        if (day > dim) {
            day = 1;
            month += 1;
            if (month > 12) {
                month = 1;
                year += 1;
            }
        }
    }

    while (hour < 0) {
        hour += 24;
        day -= 1;
        if (day < 1) {
            month -= 1;
            if (month < 1) {
                month = 12;
                year -= 1;
            }
            day = ui_days_in_month(year, month);
        }
    }

    if (out_year)  *out_year  = year;
    if (out_month) *out_month = month;
    if (out_day)   *out_day   = day;
    if (out_hour)  *out_hour  = hour;
    if (out_min)   *out_min   = min;
    if (out_sec)   *out_sec   = sec;

    return true;
}

// ----------------- 트립/경사/0-100 상태 업데이트 -----------------

static void update_trip_state(const app_gps_state_t *gps)
{
    app_display_state_t *s = &s_disp;

    if (!s->initialized) {
        memset(s, 0, sizeof(*s));
        s->initialized  = 1u;
        s->last_gps     = *gps;
        s->has_last     = 1u;
        s->last_host_ms = gps->host_time_ms;
        return;
    }

    if (!s->has_last) {
        s->last_gps     = *gps;
        s->has_last     = 1u;
        s->last_host_ms = gps->host_time_ms;
        return;
    }

    uint32_t now_ms = gps->host_time_ms;
    uint32_t dt_ms  = now_ms - s->last_host_ms;
    s->last_host_ms = now_ms;

    if (dt_ms > 5000u) {
        // 너무 긴 gap 은 무시 (정지 상태로 본다)
        dt_ms = 0u;
    }

    float dt_s = (float)dt_ms * 0.001f;

    // 전원 후 전체 시간
    s->trip_time_ms_total += dt_ms;

    // 속도 기반 거리 적분
    float v_trip_mps = get_speed_mps_for_feature(gps, 0u);
    if (dt_s > 0.0f && v_trip_mps > 0.05f) {
        float dist_m = v_trip_mps * dt_s;

        s->trip_distance_m += dist_m;

        // 경사도 업데이트 (거리 0.5m 이상일 때만)
        float dz = gps->hmsl_m - s->last_gps.hmsl_m;
        if (fabsf(dist_m) > 0.5f) {
            float grade = (dz / dist_m) * 100.0f;   // [%]

            // ★ 가민 사이클링 컴퓨터 수준의 느린 경사 반응
            //   - 2 Hz 입력 기준으로 약 4초 타임콘스턴트
            //   - 고도 노이즈/잔떨림을 많이 죽이고, 큰 지형 변화만 천천히 반영
            const float tau = 4.0f;                 // ~4초 타임콘스턴트
            float alpha = dt_s / (tau + dt_s);

            s->grade_filtered += alpha * (grade - s->grade_filtered);

            if (s->grade_filtered > 45.0f)  s->grade_filtered = 45.0f;
            if (s->grade_filtered < -45.0f) s->grade_filtered = -45.0f;
        }

    }

    // 최고속도
    // 최고속도
    float v_trip_kmh = get_speed_kmh_for_feature(gps, 0u);
    if (v_trip_kmh > s->trip_top_speed_kmh) {
        s->trip_top_speed_kmh = v_trip_kmh;
    }

    // 평균속도 = 총 거리 / 총 시간
    if (s->trip_time_ms_total > 1000u) {
        float hours = (float)s->trip_time_ms_total / 3600000.0f;
        if (hours > 0.0f) {
            s->trip_avg_speed_kmh = (s->trip_distance_m / 1000.0f) / hours;
        }
    }

    s->last_gps = *gps;
}

// 0-100 km/h 상태 업데이트 (이 모드일 때만 호출)
static void update_zero_to_100_state(const app_gps_state_t *gps)
{
    app_display_state_t *s = &s_disp;

    if (!gps || !gps->valid) {
        return;
    }

    float v = get_speed_kmh_for_feature(gps, 1u);

    // 카운트다운 후 GO! 에서 시작된 러닝 상태만 감시해서 100km/h 도달 시점을 기록
    if (s->zto100_running && !s->zto100_done) {
        if (v >= 100.0f) {
            uint32_t dt_ms = gps->host_time_ms - s->zto100_start_ms;
            s->zto100_time_s    = (float)dt_ms * 0.001f;
            s->zto100_speed_kmh = v;
            s->zto100_running   = 0u;
            s->zto100_done      = 1u;
        }
    }
}


// 모드 진입 시 1회 호출


static void zero_to_100_countdown_and_start(void)
{
    // 0-100 모드 진입 시 4번째 자리에서 5→4→3→2→1→GO! 카운트다운 + 비프
    const uint32_t step_delay_ms = 600u;

    max7219_Clean();

    // 5,4,3,2,1 카운트다운 (4번째 자리, index 3)
    for (int n = 5; n >= 1; --n) {
        // 숫자 표시
        max7219_WriteCharAt(0, ' ', false);
        max7219_WriteCharAt(1, ' ', false);
        max7219_WriteCharAt(2, ' ', false);
        max7219_WriteCharAt(3, (char)('0' + n), false);
        max7219_WriteCharAt(4, ' ', false);
        max7219_WriteCharAt(5, ' ', false);
        max7219_WriteCharAt(6, ' ', false);
        max7219_WriteCharAt(7, ' ', false);

        // 낮은 비프음
        Buzzer_PlaySequence(BEEP_SEQ_USER2);
        HAL_Delay(step_delay_ms);
    }

    // GO! 표시 (4번째 자리부터)
    max7219_WriteCharAt(0, ' ', false);
    max7219_WriteCharAt(1, ' ', false);
    max7219_WriteCharAt(2, 'G', false);
    max7219_WriteCharAt(3, 'O', false);
    max7219_WriteCharAt(4, '!', false);
    max7219_WriteCharAt(5, ' ', false);
    max7219_WriteCharAt(6, ' ', false);
    max7219_WriteCharAt(7, ' ', false);

    // 높은 비프음
    Buzzer_PlaySequence(BEEP_SEQ_USER8);
    HAL_Delay(step_delay_ms);

    // 카운터 시작 (속도와 무관하게 GO! 이후 바로 증가)
    s_disp.zto100_running   = 1u;
    s_disp.zto100_done      = 0u;
    s_disp.zto100_start_ms  = HAL_GetTick();
    s_disp.zto100_time_s    = 0.0f;
    s_disp.zto100_speed_kmh = 0.0f;
}

static const char *get_mode_title(app_display_mode_t mode)
{
    switch (mode) {
    case APP_DISPLAY_SAT_STATUS:        return "SAT STATUS";
    case APP_DISPLAY_SPEED:             return "SPEED";
    case APP_DISPLAY_ALTITUDE:          return "ALTITUDE";
    case APP_DISPLAY_HEADING:           return "HEADING";
    case APP_DISPLAY_GRADE:             return "GRADE";
    case APP_DISPLAY_DISTANCE:          return "DISTANCE";
    case APP_DISPLAY_TOP_SPEED:         return "TOP SPEED";
    case APP_DISPLAY_AVG_SPEED:         return "AVG SPEED";
    case APP_DISPLAY_ZERO_TO_100:       return "0 TO 100";
    case APP_DISPLAY_TRIP_TIME:         return "TRIP TIME";
    case APP_DISPLAY_LOCAL_TIME:        return "LOCAL TIME";
    case APP_DISPLAY_LOCAL_DATE:        return "LOCAL DATE";
    case APP_DISPLAY_SPEED_AND_GRADE:   return "SPD+GRD";
    case APP_DISPLAY_SPEED_AND_ALTITUDE:return "SPD+ALT";
    case APP_DISPLAY_LATLON:            return "LAT/LON";
    default:                            return "";
    }
}

static void on_mode_enter(app_display_mode_t mode, bool from_button)
{
    // 기본적으로 전체 클리어
    max7219_Clean();

    if (mode == APP_DISPLAY_ZERO_TO_100) {
        // 상태 리셋
        s_disp.zto100_running   = 0u;
        s_disp.zto100_done      = 0u;
        s_disp.zto100_time_s    = 0.0f;
        s_disp.zto100_speed_kmh = 0.0f;

        // 사용자 버튼으로 진입한 경우에만 카운트다운 + GO! 실행
        if (from_button) {
            zero_to_100_countdown_and_start();
        }
        return;
    }

    // 그 외 모드는 사용자 버튼으로 모드가 바뀐 경우에만 스플래시 텍스트 스크롤
    if (from_button) {
        // 2-field 속도 모드(HDG/SPD, ALT/SPD, GRD/SPD)는 스플래시 생략
        if (mode != APP_DISPLAY_SPEED_AND_GRADE &&
            mode != APP_DISPLAY_SPEED_AND_HEADING &&
            mode != APP_DISPLAY_SPEED_AND_ALTITUDE)
        {
            const char *title = get_mode_title(mode);
            if (title && title[0] != '\0') {
                max7219_ScrollText((char *)title,
                                   0, 7,
                                   MAX7219_SCROLL_LEFT,
                                   80u,
                                   500u);
            }
        }
    }
}





















// ----------------- 개별 화면 그리기 함수 -----------------
// SAt.   NN   또는   SAt.  Err
static void ui_show_sat_status(const app_gps_state_t *gps)
{
    // 타이틀 "SAt."
    max7219_WriteCharAt(0, 'S', false);
    max7219_WriteCharAt(1, 'A', false);
    max7219_WriteCharAt(2, 't', true);   // t.
    max7219_WriteCharAt(3, ' ', false);

    uint32_t now = HAL_GetTick();

    // ----------------------------------------------------
    // 1) GPS 구조체 자체가 없거나
    // 2) host_time_ms가 0이거나
    // 3) 최근 5초 이내 업데이트가 없으면 → "SAt. Err"
    //    (에러 화면은 깜빡이지 않음)
    // ----------------------------------------------------
    if (!gps ||
        (gps->host_time_ms == 0u) ||
        ((now - gps->host_time_ms) > 5000u))
    {
        // 여기서는 4~7 자리를 "한 번만" 세팅
        max7219_WriteCharAt(4, ' ', false);
        max7219_WriteCharAt(5, 'E', false);
        max7219_WriteCharAt(6, 'r', false);
        max7219_WriteCharAt(7, 'r', false);
        return;
    }

    // ----------------------------------------------------
    // 여기까지 왔으면 최소한 최근 5초 내에 무언가 들어왔음
    //  - used: FIX에 실제로 사용 중인 위성 수
    //  - visible: 시야에 잡히는 위성 수
    // used가 너무 작으면(<=2) visible 모드로 전환해서
    // "쓸 수 있는 위성 자체는 더 많다"는 힌트를 줌
    // ----------------------------------------------------
    uint8_t used     = gps->numSV_used;
    uint8_t visible  = gps->numSV_visible;
    uint8_t sat_disp = used;
    bool    visible_mode = false;

    // FIXED SAT이 2개 이하 → VISIBLE SAT 모드로 전환
    if (used <= 2u) {
        sat_disp     = visible;
        visible_mode = true;
    }

    // 2자리 제한
    if (sat_disp > 99u) {
        sat_disp = 99u;
    }

    char tens = (sat_disp / 10u) ? (char)('0' + (sat_disp / 10u)) : ' ';
    char ones = (char)('0' + (sat_disp % 10u));

    // 자리는 항상 동일하게 정리:
    //  S(0) A(1) t.(2) ' '(3) ' '(4) ' '(5) TENS(6) ONES(7)
    // 4,5는 항상 공백으로 통일해서, 상태 전환 시 쓰레기 글자 방지
    max7219_WriteCharAt(4, ' ', false);
    max7219_WriteCharAt(5, ' ', false);

    // ----------------------------------------------------
    // visible_mode:
    //   - true  → VISIBLE SAT: 숫자만 5 Hz 블링킹
    //   - false → USED SAT: 항상 숫자 표시 (블링크 X)
    // ----------------------------------------------------
    if (visible_mode) {
        // VISIBLE SAT 모드: 숫자만 5 Hz로 깜빡임
        if (g_blink_5hz) {
            max7219_WriteCharAt(6, tens, false);
            max7219_WriteCharAt(7, ones, false);
        } else {
            max7219_WriteCharAt(6, ' ', false);
            max7219_WriteCharAt(7, ' ', false);
        }
    } else {
        // USED SAT 모드: 항상 숫자 고정 표시
        max7219_WriteCharAt(6, tens, false);
        max7219_WriteCharAt(7, ones, false);
    }
}



// SPd. XXX.X
// SPd. XXX.X
static void ui_show_speed(const app_gps_state_t *gps)
{
    max7219_WriteCharAt(0, 'S', false);
    max7219_WriteCharAt(1, 'P', false);
    max7219_WriteCharAt(2, 'd', true);
    max7219_WriteCharAt(3, ' ', false);

    if (!gps || !gps->valid) {
        // GPS 미고정 시 숫자 대신 --- 표시
        max7219_WriteCharAt(4, ' ', false);
        max7219_WriteCharAt(5, '-', false);
        max7219_WriteCharAt(6, '-', false);
        max7219_WriteCharAt(7, '-', false);
        return;
    }
    // 오버스피드 상태에서는 숫자만 5Hz 블링킹
    if (s_overspeed_active && !g_blink_5hz) {
        for (uint8_t pos = 4; pos < 8; ++pos) {
            max7219_WriteCharAt(pos, ' ', false);
        }
        return;
    }

    ui_print_fixed_1(get_speed_kmh_for_feature(gps, 0u), 4, 1999u); // 0.0 ~ 199.9 km/h
}


// ALt. XXXX
static void ui_show_altitude(const app_gps_state_t *gps)
{
    max7219_WriteCharAt(0, 'A', false);
    max7219_WriteCharAt(1, 'L', false);
    max7219_WriteCharAt(2, 't', true);
    max7219_WriteCharAt(3, ' ', false);

    if (!gps || !gps->valid) {
        // GPS 미고정 시 숫자 대신 --- 표시
        max7219_WriteCharAt(4, ' ', false);
        max7219_WriteCharAt(5, '-', false);
        max7219_WriteCharAt(6, '-', false);
        max7219_WriteCharAt(7, '-', false);
        return;
    }

    int alt_m = (int)(gps->hmsl_m >= 0.0f ? gps->hmsl_m + 0.5f : gps->hmsl_m - 0.5f);
    if (alt_m < -999) alt_m = -999;
    if (alt_m > 9999) alt_m = 9999;

    // 음수는 간단히 0으로 클램프 (원하면 나중에 부호 처리 추가 가능)
    if (alt_m < 0) alt_m = 0;

    ui_print_uint_right((uint32_t)alt_m, 4, 7);
}

// hdg. xxx^
static void ui_show_heading(const app_gps_state_t *gps)
{
    max7219_WriteCharAt(0, 'h', false);
    max7219_WriteCharAt(1, 'd', false);
    max7219_WriteCharAt(2, 'g', true);
    max7219_WriteCharAt(3, ' ', false);

    bool heading_valid = false;
    float heading = get_heading_for_feature(gps, 0u, &heading_valid);

    if (!gps || !gps->valid || !heading_valid) {
        // 헤딩이 유효하지 않으면 숫자 자리에 ---^ 표시로 클램프
        max7219_WriteCharAt(4, '-', false);
        max7219_WriteCharAt(5, '-', false);
        max7219_WriteCharAt(6, '-', false);
        max7219_WriteCharAt(7, '^', false);
        return;
    }

    uint16_t h = (uint16_t)(heading + 0.5f);
    if (h >= 360u) {
        h = 359u;
    }

    uint8_t d0 = (uint8_t)((h / 100u) % 10u);
    uint8_t d1 = (uint8_t)((h / 10u)  % 10u);
    uint8_t d2 = (uint8_t)(h % 10u);

    char c0 = (d0 > 0u) ? (char)('0' + d0) : ' ';
    char c1 = ((d0 > 0u) || (d1 > 0u)) ? (char)('0' + d1) : ' ';
    char c2 = (char)('0' + d2);

    max7219_WriteCharAt(4, c0, false);
    max7219_WriteCharAt(5, c1, false);
    max7219_WriteCharAt(6, c2, false);
    max7219_WriteCharAt(7, '^', false);  // 커스텀 각도 문자
}


// GRd. -XX.X
static void ui_show_grade(const app_gps_state_t *gps)
{
    (void)gps;

    max7219_WriteCharAt(0, 'G', false);
    max7219_WriteCharAt(1, 'R', false);
    max7219_WriteCharAt(2, 'd', true);
    max7219_WriteCharAt(3, ' ', false);

    ui_print_signed_grade(s_disp.grade_filtered, 4, 5);
}

// dSt. XXX.X  (km)
static void ui_show_distance(const app_gps_state_t *gps)
{
    (void)gps;

    max7219_WriteCharAt(0, 'd', false);
    max7219_WriteCharAt(1, 'S', false);
    max7219_WriteCharAt(2, 't', true);
    max7219_WriteCharAt(3, ' ', false);

    float dist_km = s_disp.trip_distance_m / 1000.0f;
    ui_print_fixed_1(dist_km, 4, 9999u); // 0.0 ~ 999.9 km
}

// tOP. XXX.X
static void ui_show_top_speed(const app_gps_state_t *gps)
{
    (void)gps;

    max7219_WriteCharAt(0, 't', false);
    max7219_WriteCharAt(1, 'O', false);
    max7219_WriteCharAt(2, 'P', true);
    max7219_WriteCharAt(3, ' ', false);

    ui_print_fixed_1(s_disp.trip_top_speed_kmh, 4, 1999u);
}

// AUg. XXX.X
static void ui_show_avg_speed(const app_gps_state_t *gps)
{
    (void)gps;

    max7219_WriteCharAt(0, 'A', false);
    max7219_WriteCharAt(1, 'U', false);
    max7219_WriteCharAt(2, '9', true);   // 요청대로 AUg
    max7219_WriteCharAt(3, ' ', false);

    ui_print_fixed_1(s_disp.trip_avg_speed_kmh, 4, 1999u);
}

// T  XX-XX (HH-MM)
static void ui_show_trip_time(void)
{
    max7219_WriteCharAt(0, 'T', false);
    max7219_WriteCharAt(1, ' ', false);
    max7219_WriteCharAt(2, ' ', false);

    uint32_t total_ms  = s_disp.trip_time_ms_total;
    uint32_t total_sec = total_ms / 1000u;
    uint32_t hh        = total_sec / 3600u;
    uint32_t mm        = (total_sec % 3600u) / 60u;

    if (hh > 99u) hh = 99u;

    max7219_WriteCharAt(3, (char)('0' + (uint8_t)((hh / 10u) % 10u)), false);
    max7219_WriteCharAt(4, (char)('0' + (uint8_t)(hh % 10u)),         false);
    max7219_WriteCharAt(5, '-', false);
    max7219_WriteCharAt(6, (char)('0' + (uint8_t)((mm / 10u) % 10u)), false);
    max7219_WriteCharAt(7, (char)('0' + (uint8_t)(mm % 10u)),         false);
}

// C  XX-XX (HH-MM)
static void ui_show_local_time(const app_gps_state_t *gps)
{
    max7219_WriteCharAt(0, 'C', false);
    max7219_WriteCharAt(1, 'T', false);
    max7219_WriteCharAt(2, ' ', false);

    int year, month, day, hour, min, sec;
    if (!ui_compute_local_datetime(gps, s_timezone_hours,
                                   &year, &month, &day,
                                   &hour, &min, &sec)) {
        // GPS 시간이 없으면 시간 영역에 ----- 로 표시
        for (uint8_t pos = 3; pos < 8; ++pos) {
            max7219_WriteCharAt(pos, '-', false);
        }
        return;
    }
    uint8_t hh = (uint8_t)(hour & 0xFF);
    uint8_t mm = (uint8_t)(min  & 0xFF);

    max7219_WriteCharAt(3, (char)('0' + (hh / 10u)), false);
    max7219_WriteCharAt(4, (char)('0' + (hh % 10u)), false);
    max7219_WriteCharAt(5, '-', false);
    max7219_WriteCharAt(6, (char)('0' + (mm / 10u)), false);
    max7219_WriteCharAt(7, (char)('0' + (mm % 10u)), false);
}

// XXXX.XX.XX (연.월.일)
static void ui_show_local_date(const app_gps_state_t *gps)
{
    int year, month, day, hour, min, sec;
    if (!ui_compute_local_datetime(gps, s_timezone_hours,
                                   &year, &month, &day,
                                   &hour, &min, &sec)) {
        // GPS 시간이 없으면 날짜 영역 전체를 -------- 로 표시
        for (uint8_t pos = 0; pos < 8; ++pos) {
            max7219_WriteCharAt(pos, '-', false);
        }
        return;
    }


    int y = year;
    if (y < 0)    y = 0;
    if (y > 9999) y = 9999;

    uint8_t y_th = (uint8_t)((y / 1000)      % 10);
    uint8_t y_h  = (uint8_t)((y / 100)       % 10);
    uint8_t y_t  = (uint8_t)((y / 10)        % 10);
    uint8_t y_o  = (uint8_t)(y % 10);

    uint8_t m_t  = (uint8_t)((month / 10) % 10);
    uint8_t m_o  = (uint8_t)(month % 10);

    uint8_t d_t  = (uint8_t)((day / 10) % 10);
    uint8_t d_o  = (uint8_t)(day % 10);

    max7219_WriteCharAt(0, (char)('0' + y_th), false);
    max7219_WriteCharAt(1, (char)('0' + y_h),  false);
    max7219_WriteCharAt(2, (char)('0' + y_t),  false);
    max7219_WriteCharAt(3, (char)('0' + y_o),  true);  // YYYY.

    max7219_WriteCharAt(4, (char)('0' + m_t),  false);
    max7219_WriteCharAt(5, (char)('0' + m_o),  true);  // MM.
    max7219_WriteCharAt(6, (char)('0' + d_t),  false);
    max7219_WriteCharAt(7, (char)('0' + d_o),  false);
}

// 0-100 모드 표시: ' XX.X XXX'
static void ui_show_zero_to_100(const app_gps_state_t *gps)
{
    // 측정이 안 돌아가는 상태에서는 "0TO100" 고정 표시
    if (!gps || !gps->valid || (!s_disp.zto100_running && !s_disp.zto100_done)) {
        max7219_WriteStringInRange((char*)"0TO100", 1, 6, true);
        return;
    }

    float time_s;
    float speed_kmh;

    if (s_disp.zto100_done) {
        time_s    = s_disp.zto100_time_s;
        speed_kmh = s_disp.zto100_speed_kmh;
    } else {
        uint32_t dt_ms = gps->host_time_ms - s_disp.zto100_start_ms;
        time_s    = (float)dt_ms * 0.001f;
        speed_kmh = get_speed_kmh_for_feature(gps, 1u);

    }

    // ' XX.X XXX'
    ui_print_fixed_1(time_s, 0, 999u); // 0.0 ~ 99.9 s
    max7219_WriteCharAt(4, ' ', false);
    ui_print_speed_int3(speed_kmh, 5);
}

// 속도 + 경사: '-XX XXX.X'
// 속도 + 경사: 'XXX-XX  '
//  - 속도: 정수 3자리 (0..199 km/h) → [0..2]
//  - 경사: 부호 + 2자리 정수 → [3..5] (4번째 8 자리부터)
// 속도 + 경사 모드: "0.0GXXX.X" 형식
static void ui_show_speed_and_grade(const app_gps_state_t *gps)
{
    if (!gps || !gps->valid) {
        // NO GPS 표시는 기존 그대로 둠
        max7219_WriteCharAt(0, 'N', false);
        max7219_WriteCharAt(1, 'O', false);
        max7219_WriteCharAt(2, ' ', false);
        max7219_WriteCharAt(3, 'G', false);
        max7219_WriteCharAt(4, 'P', false);
        max7219_WriteCharAt(5, 'S', false);
        max7219_WriteCharAt(6, ' ', false);
        max7219_WriteCharAt(7, ' ', false);
        return;
    }

    // 새 포맷: "--.- ---.-"
    //  [0..3] : 경사도 "sXX.X" (부호 + 두 자리 + 소수 1)
    //  [4..7] : 속도   "XXX.X" (0.1 km/h)
    ui_print_signed_grade(s_disp.grade_filtered, 0u, 1u);
    ui_print_fixed_1(get_speed_kmh_for_feature(gps, 0u), 4u, 1999u);
}



// 속도 + 헤딩: 'XXX^XXX.X'
// 속도 + 헤딩: 'XXX^XXX.X'
static void ui_show_speed_and_heading(const app_gps_state_t *gps)
{
    // 1) GPS 자체가 유효하지 않으면 둘 다 --- 처리
    if (!gps || !gps->valid) {
        max7219_WriteCharAt(0, '-', false);
        max7219_WriteCharAt(1, '-', false);
        max7219_WriteCharAt(2, '-', false);
        max7219_WriteCharAt(3, '^', false);

        max7219_WriteCharAt(4, ' ', false);
        max7219_WriteCharAt(5, '-', false);
        max7219_WriteCharAt(6, '-', false);
        max7219_WriteCharAt(7, '-', false);
        return;
    }

    bool heading_valid = false;
    float heading = get_heading_for_feature(gps, 0u, &heading_valid);

    // 2) 헤딩 부분
    if (!heading_valid) {
        // 속도는 정상 표시, 헤딩만 ---^
        max7219_WriteCharAt(0, '-', false);
        max7219_WriteCharAt(1, '-', false);
        max7219_WriteCharAt(2, '-', false);
        max7219_WriteCharAt(3, '^', false);
    } else {
        uint16_t h = (uint16_t)(heading + 0.5f);
        if (h >= 360u) {
            h = 359u;
        }

        uint8_t d0 = (uint8_t)((h / 100u) % 10u);
        uint8_t d1 = (uint8_t)((h / 10u)  % 10u);
        uint8_t d2 = (uint8_t)(h % 10u);

        char c0 = (d0 > 0u) ? (char)('0' + d0) : ' ';
        char c1 = ((d0 > 0u) || (d1 > 0u)) ? (char)('0' + d1) : ' ';
        char c2 = (char)('0' + d2);

        max7219_WriteCharAt(0, c0, false);
        max7219_WriteCharAt(1, c1, false);
        max7219_WriteCharAt(2, c2, false);
        max7219_WriteCharAt(3, '^', false);
    }

    // 3) 속도 부분: GPS만 유효하면 항상 표시
    ui_print_fixed_1(get_speed_kmh_for_feature(gps, 0u), 4, 1999u); // 0.0 ~ 199.9 km/h
}




// 속도 + 고도: 'XXXmXXX.X'
static void ui_show_speed_and_altitude(const app_gps_state_t *gps)
{
    if (!gps || !gps->valid) {
        // GPS 미고정 시 고도/속도 자리에 각각 ---m / --- 표시
        max7219_WriteCharAt(0, '-', false);
        max7219_WriteCharAt(1, '-', false);
        max7219_WriteCharAt(2, '-', false);
        max7219_WriteCharAt(3, 'n', false);

        max7219_WriteCharAt(4, ' ', false);
        max7219_WriteCharAt(5, '-', false);
        max7219_WriteCharAt(6, '-', false);
        max7219_WriteCharAt(7, '-', false);
        return;
    }

    int alt_m = (int)(gps->hmsl_m >= 0.0f ? gps->hmsl_m + 0.5f : gps->hmsl_m - 0.5f);
    if (alt_m < 0)   alt_m = 0;
    if (alt_m > 999) alt_m = 999;

    uint8_t d0 = (uint8_t)((alt_m / 100) % 10);
    uint8_t d1 = (uint8_t)((alt_m / 10)  % 10);
    uint8_t d2 = (uint8_t)(alt_m % 10);

    char c0 = (d0 > 0) ? (char)('0' + d0) : ' ';
    char c1 = ((d0 > 0) || (d1 > 0)) ? (char)('0' + d1) : ' ';
    char c2 = (char)('0' + d2);

    max7219_WriteCharAt(0, c0, false);
    max7219_WriteCharAt(1, c1, false);
    max7219_WriteCharAt(2, c2, false);
    max7219_WriteCharAt(3, 'n', false);

    ui_print_fixed_1(get_speed_kmh_for_feature(gps, 0u), 4, 1999u);
}

static void ui_show_latlon(const app_gps_state_t *gps)
{
    if (!gps || !gps->valid) {
        // NO FIX 표시 재사용 (CLEAN 없이 자리만 덮어쓰기)
        max7219_WriteCharAt(0, 'N', false);
        max7219_WriteCharAt(1, 'O', false);
        max7219_WriteCharAt(2, ' ', false);
        max7219_WriteCharAt(3, '9', false);
        max7219_WriteCharAt(4, 'P', false);
        max7219_WriteCharAt(5, '5', false);
        max7219_WriteCharAt(6, ' ', false);
        max7219_WriteCharAt(7, ' ', false);
        return;
    }

    uint32_t now_ms = gps->host_time_ms;

    if (s_latlon_last_toggle_ms == 0u) {
        s_latlon_last_toggle_ms = now_ms;
        s_latlon_show_lat       = 1u;      // 처음엔 위도부터
    } else if ((now_ms - s_latlon_last_toggle_ms) >= 5000u) {
        s_latlon_show_lat       = (uint8_t)!s_latlon_show_lat;
        s_latlon_last_toggle_ms = now_ms;
    }

    double value_deg = s_latlon_show_lat ? gps->lat_deg : gps->lon_deg;
    double abs_deg   = value_deg;
    if (abs_deg < 0.0) {
        abs_deg = -abs_deg;
    }

    // 안전 클램프
    if (abs_deg > 999.9999) {
        abs_deg = 999.9999;
    }

    // 소수점 이하 4자리까지 표현
    uint32_t scaled  = (uint32_t)(abs_deg * 10000.0 + 0.5);
    uint32_t deg_int = scaled / 10000u;
    uint32_t frac    = scaled % 10000u;

    if (deg_int > 999u) {
        deg_int = 999u;
    }

    char dir_char;
    if (s_latlon_show_lat) {
        // 기존 로직 그대로 유지
        dir_char = (value_deg >= 0.0) ? 'n' : 'S';
    } else {
        dir_char = (value_deg >= 0.0) ? 'E' : 'u';
    }

    // ★ 여기서 더 이상 max7219_Clean() 호출 안 함

    // 0번째 자리에 방향 표기
    max7219_WriteCharAt(0, dir_char, false);

    // 1~3번째 자리에 정수부 (리딩 제로 없이)
    uint32_t hundreds = (deg_int / 100u) % 10u;
    uint32_t tens     = (deg_int / 10u)  % 10u;
    uint32_t ones     = deg_int % 10u;

    char ch1 = ' ';
    char ch2 = ' ';
    char ch3 = (char)('0' + ones);

    if (deg_int >= 100u) {
        ch1 = (char)('0' + hundreds);
        ch2 = (char)('0' + tens);
    } else if (deg_int >= 10u) {
        ch2 = (char)('0' + tens);
    }

    max7219_WriteCharAt(1, ch1, false);
    max7219_WriteCharAt(2, ch2, false);
    // 소수점은 정수부 끝(3번째 자리)에 찍기
    max7219_WriteCharAt(3, ch3, true);

    // 4~7번째 자리에 소수부 4자리
    uint32_t d0 = (frac / 1000u) % 10u;
    uint32_t d1 = (frac / 100u)  % 10u;
    uint32_t d2 = (frac / 10u)   % 10u;
    uint32_t d3 = frac % 10u;

    max7219_WriteCharAt(4, (char)('0' + d0), false);
    max7219_WriteCharAt(5, (char)('0' + d1), false);
    max7219_WriteCharAt(6, (char)('0' + d2), false);
    max7219_WriteCharAt(7, (char)('0' + d3), false);
}



// ----------------- 부팅 설정 메뉴 화면 -----------------

// 수정 후
void APP_Display_ShowSetupBeepVolume(uint8_t vol_0_to_4)
{
    if (vol_0_to_4 > 4u) {
        vol_0_to_4 = 4u;
    }

    // "bEP X" 형태로 표시 (X는 0~4, 5 Hz 블링크)
    max7219_WriteCharAt(0, 'b', false);
    max7219_WriteCharAt(1, 'E', false);
    max7219_WriteCharAt(2, 'P', false);
    max7219_WriteCharAt(3, ' ', false);

    char digit = (char)('0' + vol_0_to_4);
    if (g_blink_5hz) {
        max7219_WriteCharAt(4, digit, false);
    } else {
        max7219_WriteCharAt(4, ' ', false);
    }

    max7219_WriteCharAt(5, ' ', false);
    max7219_WriteCharAt(6, ' ', false);
    max7219_WriteCharAt(7, ' ', false);
}



void APP_Display_ShowSetupGMT(int8_t offset_hours)
{
    // gnt  -XX 형식, 부호/숫자만 5Hz 블링킹
    max7219_WriteCharAt(0, 'g', false);
    max7219_WriteCharAt(1, 'n', false);
    max7219_WriteCharAt(2, 't', false);
    max7219_WriteCharAt(3, ' ', false);
    max7219_WriteCharAt(4, ' ', false);

    // 허용 범위: -12 ~ +14 정도로 클램프
    if (offset_hours < -12) {
        offset_hours = -12;
    } else if (offset_hours > 14) {
        offset_hours = 14;
    }

    int8_t abs_val = (offset_hours < 0) ? -offset_hours : offset_hours;
    if (abs_val > 99) {
        abs_val = 99;
    }

    char    sign = (offset_hours < 0) ? '-' : ' ';
    uint8_t tens = (uint8_t)(abs_val / 10);
    uint8_t ones = (uint8_t)(abs_val % 10);

    if (g_blink_5hz) {
        max7219_WriteCharAt(5, sign, false);
        max7219_WriteCharAt(6, (char)('0' + tens), false);
        max7219_WriteCharAt(7, (char)('0' + ones), false);
    } else {
        max7219_WriteCharAt(5, ' ', false);
        max7219_WriteCharAt(6, ' ', false);
        max7219_WriteCharAt(7, ' ', false);
    }
}

void APP_Display_ShowSetupBrightness(uint8_t level_1_to_3)
{
    // brt    X, X만 5Hz 블링킹
    if (level_1_to_3 < 1u) {
        level_1_to_3 = 1u;
    } else if (level_1_to_3 > 3u) {
        level_1_to_3 = 3u;
    }

    max7219_WriteCharAt(0, 'b', false);
    max7219_WriteCharAt(1, 'r', false);
    max7219_WriteCharAt(2, 't', false);
    max7219_WriteCharAt(3, ' ', false);
    max7219_WriteCharAt(4, ' ', false);
    max7219_WriteCharAt(5, ' ', false);
    max7219_WriteCharAt(6, ' ', false);

    if (g_blink_5hz) {
        max7219_WriteCharAt(7, (char)('0' + level_1_to_3), false);
    } else {
        max7219_WriteCharAt(7, ' ', false);
    }
}

void APP_Display_ShowSetupAutoMode(bool enabled)
{
    // "AUto  on" / "AUto oFF", on/off 텍스트만 5Hz 블링킹
    max7219_WriteCharAt(0, 'A', false);
    max7219_WriteCharAt(1, 'U', false);
    max7219_WriteCharAt(2, 't', false);
    max7219_WriteCharAt(3, 'o', false);

    if (enabled) {
        // "AUto  on"
        max7219_WriteCharAt(4, ' ', false);
        max7219_WriteCharAt(5, ' ', false);
        if (g_blink_5hz) {
            max7219_WriteCharAt(6, 'o', false);
            max7219_WriteCharAt(7, 'n', false);
        } else {
            max7219_WriteCharAt(6, ' ', false);
            max7219_WriteCharAt(7, ' ', false);
        }
    } else {
        // "AUto oFF"
        max7219_WriteCharAt(4, ' ', false);
        if (g_blink_5hz) {
            max7219_WriteCharAt(5, 'o', false);
            max7219_WriteCharAt(6, 'F', false);
            max7219_WriteCharAt(7, 'F', false);
        } else {
            max7219_WriteCharAt(5, ' ', false);
            max7219_WriteCharAt(6, ' ', false);
            max7219_WriteCharAt(7, ' ', false);
        }
    }
}

void APP_Display_ShowSetupHwTest(void)
{
    // dEu tESt, 블링킹 없음
    max7219_WriteCharAt(0, 'd', false);
    max7219_WriteCharAt(1, 'E', false);
    max7219_WriteCharAt(2, 'u', false);
    max7219_WriteCharAt(3, ' ', false);
    max7219_WriteCharAt(4, 't', false);
    max7219_WriteCharAt(5, 'E', false);
    max7219_WriteCharAt(6, 'S', false);
    max7219_WriteCharAt(7, 't', false);
}

void APP_Display_ShowDataError(void)
{
    // "dAtA Err"
    max7219_WriteCharAt(0, 'd', false);
    max7219_WriteCharAt(1, 'A', false);
    max7219_WriteCharAt(2, 't', false);
    max7219_WriteCharAt(3, 'A', false);
    max7219_WriteCharAt(4, ' ', false);
    max7219_WriteCharAt(5, 'E', false);
    max7219_WriteCharAt(6, 'r', false);
    max7219_WriteCharAt(7, 'r', false);
}

// ----------------- 외부 API -----------------

void APP_Display_Init(void)
{
    memset(&s_disp, 0, sizeof(s_disp));
    s_last_mode      = g_display_mode;
    s_timezone_hours = 9;

    max7219_Clean();
}

void APP_Display_Update(void)
{
    app_gps_state_t gps;
    bool valid_fix_struct = APP_GPS_GetState(&gps); // 반환값은 gps.valid와 동일

    bool fix_ready = is_fix_ready(&gps);

    // FIX 상태 변화 감지 → Sat OK 비프
    if (fix_ready && !s_fix_ready_prev) {
        Buzzer_PlaySequence(BEEP_SEQ_SAT_OK);
    }
    s_fix_ready_prev = (uint8_t)fix_ready;

    // Trip / 0→100은 "유효한 fix"에서만 업데이트
    if (fix_ready) {
        update_trip_state(&gps);

        if (g_display_mode == APP_DISPLAY_ZERO_TO_100) {
            update_zero_to_100_state(&gps);
        }
    } else {

    }

    // overspeed 및 속도 경고 업데이트 (4번에서 구현)
    update_speed_warnings_and_overspeed(&gps);


    // SUNRISE/SUNSET 기반 자동 밝기 업데이트
    update_auto_brightness(&gps, (uint8_t)(fix_ready && valid_fix_struct));

    // ---- 실제로 어떤 모드를 그릴지 결정 ----
    app_display_mode_t mode = g_display_mode;

    // AUTO 모드가 켜져 있으면 자동 모드 선택 로직 적용
    update_auto_mode(&gps, fix_ready, &mode);

    /*

    if (!fix_ready) {
        // FIX 되기 전에는 무조건 SAT 화면으로 클램프
        mode = APP_DISPLAY_SAT_STATUS;
    } else if (s_overspeed_active) {
        // 오버스피드 시에는 모드를 강제로 SPEED로
        mode = APP_DISPLAY_SPEED;
    }

    */

    if (mode != s_last_mode) {
        bool from_button = (s_mode_change_from_button != 0u);
        on_mode_enter(mode, from_button);
        s_last_mode = mode;
        s_mode_change_from_button = 0u;
    }


    const app_gps_state_t *pgps = &gps;

    switch (mode) {
    case APP_DISPLAY_SAT_STATUS:
        ui_show_sat_status(pgps);
        break;

    case APP_DISPLAY_SPEED:
        ui_show_speed(pgps);
        break;

    case APP_DISPLAY_ALTITUDE:
        ui_show_altitude(pgps);
        break;

    case APP_DISPLAY_HEADING:
        ui_show_heading(pgps);
        break;

    case APP_DISPLAY_GRADE:
        ui_show_grade(pgps);
        break;

    case APP_DISPLAY_DISTANCE:
        ui_show_distance(pgps);
        break;

    case APP_DISPLAY_TOP_SPEED:
        ui_show_top_speed(pgps);
        break;

    case APP_DISPLAY_AVG_SPEED:
        ui_show_avg_speed(pgps);
        break;

    case APP_DISPLAY_ZERO_TO_100:
        ui_show_zero_to_100(pgps);
        break;

    case APP_DISPLAY_TRIP_TIME:
        ui_show_trip_time();
        break;

    case APP_DISPLAY_LOCAL_TIME:
        ui_show_local_time(pgps);
        break;

    case APP_DISPLAY_LOCAL_DATE:
        ui_show_local_date(pgps);
        break;

    case APP_DISPLAY_LATLON:
        ui_show_latlon(pgps);
        break;

    case APP_DISPLAY_SPEED_AND_GRADE:
        ui_show_speed_and_grade(pgps);
        break;

    case APP_DISPLAY_SPEED_AND_HEADING:
        ui_show_speed_and_heading(pgps);
        break;

    case APP_DISPLAY_SPEED_AND_ALTITUDE:
        ui_show_speed_and_altitude(pgps);
        break;
    }
}

void APP_Display_SetMode(app_display_mode_t mode)
{
    if (mode >= APP_DISPLAY_MODE_COUNT) {
        mode = APP_DISPLAY_SPEED;
    }

    if (mode != g_display_mode) {
        g_display_mode = mode;
        s_mode_change_from_button = 1u;
    }
}

// 0-100 카운트다운 & 시작 함수는 이미 아래쪽에 정의되어 있음
// static void zero_to_100_countdown_and_start(void);

void APP_Display_NextMode(void)
{
    const app_display_mode_t *table = NULL;
    uint8_t count = 0u;

    switch (g_display_bank)
    {
    case APP_DISPLAY_BANK_SINGLE:
    default:
        // 1필드 뱅크: 단일 필드 모드들만 순환
        table = s_bank_single_modes;
        count = BANK_MODE_COUNT(s_bank_single_modes);
        break;

    case APP_DISPLAY_BANK_DUAL:
        // 2필드 뱅크: 2필드 모드 + 필수 단일필드들 순환
        table = s_bank_dual_modes;
        count = BANK_MODE_COUNT(s_bank_dual_modes);
        break;

    case APP_DISPLAY_BANK_ZTO100:
        // 제로백 뱅크에서는 짧게 누르면
        //  - 항상 0-100 화면 유지
        //  - 카운트다운 + 측정 재시작
        g_display_mode = APP_DISPLAY_ZERO_TO_100;
        s_mode_change_from_button = 1u;
        zero_to_100_countdown_and_start();
        return;
    }

    if (!table || count == 0u)
        return;

    app_display_mode_t cur = g_display_mode;

    // 현재 모드가 테이블 안에 없으면 첫 번째로 강제
    uint8_t idx   = 0u;
    uint8_t found = 0u;
    for (uint8_t i = 0u; i < count; ++i)
    {
        if (table[i] == cur)
        {
            idx   = i;
            found = 1u;
            break;
        }
    }

    if (!found)
    {
        APP_Display_SetMode(table[0]);
        return;
    }

    // 같은 뱅크 내에서만 다음 모드로 이동
    idx = (uint8_t)((idx + 1u) % count);
    APP_Display_SetMode(table[idx]);
}

void APP_Display_NextBank(void)
{
    // 뱅크 +1 (0 → 1 → 2 → 0 ...)
    app_display_bank_t bank = g_display_bank;
    bank = (app_display_bank_t)((bank + 1u) % APP_DISPLAY_BANK_COUNT);
    g_display_bank = bank;

    // 뱅크 이름 잠깐 보여주기 ("1 FIELd" / "2 FIELd" / "0 to 100")
    show_bank_label(bank);

    // 뱅크에 진입할 때 기본 모드 결정
    switch (bank)
    {
    case APP_DISPLAY_BANK_SINGLE:
        if (BANK_MODE_COUNT(s_bank_single_modes) > 0u)
        {
            APP_Display_SetMode(s_bank_single_modes[0]);
        }
        break;

    case APP_DISPLAY_BANK_DUAL:
        if (BANK_MODE_COUNT(s_bank_dual_modes) > 0u)
        {
            APP_Display_SetMode(s_bank_dual_modes[0]);
        }
        break;

    case APP_DISPLAY_BANK_ZTO100:
        // 제로백 뱅크: 0-100 모드로 진입 + 카운트다운은
        // on_mode_enter()에서 from_button 플래그 보고 처리
        APP_Display_SetMode(APP_DISPLAY_ZERO_TO_100);
        break;

    default:
        break;
    }
}





void APP_Display_SetTimezone(int8_t offset_hours)
{
    if (offset_hours < -12) {
        offset_hours = -12;
    } else if (offset_hours > 14) {
        offset_hours = 14;
    }
    s_timezone_hours = offset_hours;
}


void APP_Display_SetBrightnessLevel(uint8_t level_1_to_3)
{
    if (level_1_to_3 < 1u) {
        level_1_to_3 = 1u;
    } else if (level_1_to_3 > 3u) {
        level_1_to_3 = 3u;
    }

    s_global_brightness_level = level_1_to_3;

    // FIX가 아직 없어도 대략적인 기본 밝기(낮 단계)로 한 번 반영
    uint8_t global_index = s_global_brightness_level - 1u;
    if (global_index > 2u) {
        global_index = 1u;
    }

    uint8_t step = 2u; // 기본: 낮
    s_current_brightness_step = step;

    uint8_t new_intensity = s_brightness_table[global_index][step];
    s_current_intensity = new_intensity;
    max7219_SetIntensivity(new_intensity);
}

// 부팅 시 밝기 범위를 눈으로 튜닝하기 위한 "brIgHt" 스윕
void APP_Display_RunBrightnessSweepTest(void)
{
    max7219_Clean();

    max7219_WriteCharAt(0, ' ', false);
    max7219_WriteCharAt(1, 'H', false);
    max7219_WriteCharAt(2, 'E', false);
    max7219_WriteCharAt(3, 'L', false);
    max7219_WriteCharAt(4, 'L', false);
    max7219_WriteCharAt(5, 'O', false);
    max7219_WriteCharAt(6, ' ', false);
    max7219_WriteCharAt(7, ' ', false);

    // ↑ 밝기 스윕 (0 -> 15)
    for (uint8_t intens = 0u; intens <= 0x0Fu; ++intens) {
        max7219_SetIntensivity(intens);
        HAL_Delay(50);
    }

    // ↓ 밝기 스윕 (15 -> 0)
    for (int8_t intens = 0x0F; intens >= 0; --intens) {
        max7219_SetIntensivity((uint8_t)intens);
        HAL_Delay(50);
    }

    max7219_Clean();
}


void APP_Display_SetAutoModeEnabled(bool enabled)
{
    bool prev_enabled = (s_auto_mode_enabled != 0u);

    if (enabled) {
        s_auto_mode_enabled = 1u;
    } else {
        s_auto_mode_enabled     = 0u;
        s_auto_active           = 0u;
        s_auto_has_last_alt     = 0u;
        s_auto_alt_cond_active  = 0u;
        s_auto_stop_cond_active = 0u;
    }

    // AUTO 모드 토글 시 "AUTO ON"/"AUTO OFF" 스플래시 텍스트 좌→우 스캔
    /*
     *
     if (enabled != prev_enabled) {
        const char *text = enabled ? "AUTO ON" : "AUTO OFF";

        max7219_Clean();
        max7219_ScrollText((char *)text,
                           0, 7,
                           MAX7219_SCROLL_LEFT,
                           80u,   // step delay (ms)
                           500u); // 마지막 프레임 유지 시간 (ms)
    }
    */
}


void APP_Display_ToggleAutoMode(void)
{
    APP_Display_SetAutoModeEnabled(!s_auto_mode_enabled);
}

bool APP_Display_IsAutoModeEnabled(void)
{
    return (s_auto_mode_enabled != 0u);
}
