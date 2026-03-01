#include "gps_ubx.h"
#include <string.h>
#include <math.h>      // <= 새로 추가


static void GPS_UBX_UpdateDerivedSpeedFromHnr(const ubx_hnr_pvt_t *hnr,
                                              uint32_t host_time_ms);


// ---------- Internal parser state ----------

typedef struct
{
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint16_t index;
    uint8_t  ck_a;
    uint8_t  ck_b;
    uint8_t  state;
    uint8_t  payload[GPS_UBX_MAX_PAYLOAD];
} ubx_parser_t;

enum
{
    UBX_WAIT_SYNC1 = 0,
    UBX_WAIT_SYNC2,
    UBX_WAIT_CLASS,
    UBX_WAIT_ID,
    UBX_WAIT_LEN1,
    UBX_WAIT_LEN2,
    UBX_WAIT_PAYLOAD,
    UBX_WAIT_CK_A,
    UBX_WAIT_CK_B
};

static ubx_parser_t s_parser;

// Latest raw messages
volatile ubx_hnr_pvt_t g_hnr_pvt;
volatile bool          g_hnr_pvt_valid = false;

volatile ubx_nav_pvt_t g_nav_pvt;
volatile bool          g_nav_pvt_valid = false;

// High-level merged fix
volatile gps_fix_basic_t g_gps_fix;
volatile bool            g_gps_fix_new = false;

// UART RX one-byte buffer
static uint8_t s_gps_rx_byte = 0;

// ---------- Small helpers ----------

static inline void gps_debug_pulse(void)
{
    HAL_GPIO_TogglePin(GPS_DEBUG_GPIO_Port, GPS_DEBUG_GPIO_Pin);
}

static void ubx_parser_reset(ubx_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = UBX_WAIT_SYNC1;
}

static void ubx_checksum_reset(ubx_parser_t *p)
{
    p->ck_a = 0;
    p->ck_b = 0;
}

static void ubx_checksum_update(ubx_parser_t *p, uint8_t b)
{
    p->ck_a = (uint8_t)(p->ck_a + b);
    p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
}

// Send one UBX frame (blocking)
static void ubx_send(uint8_t cls, uint8_t id, const void *payload, uint16_t len)
{
    uint8_t header[2] = { 0xB5, 0x62 };
    uint8_t head2[4];

    head2[0] = cls;
    head2[1] = id;
    head2[2] = (uint8_t)(len & 0xFF);
    head2[3] = (uint8_t)(len >> 8);

    uint8_t ck_a = 0, ck_b = 0;

    // checksum over class, id, length and payload
    const uint8_t *ptr;
    size_t i;

    ptr = &head2[0];
    for (i = 0; i < 4; i++) {
        ck_a = (uint8_t)(ck_a + ptr[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }

    if (payload && len) {
        ptr = (const uint8_t *)payload;
        for (i = 0; i < len; i++) {
            ck_a = (uint8_t)(ck_a + ptr[i]);
            ck_b = (uint8_t)(ck_b + ck_a);
        }
    }

    HAL_UART_Transmit(&GPS_UART_HANDLE, header, 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(&GPS_UART_HANDLE, head2, 4, HAL_MAX_DELAY);
    if (payload && len) {
        HAL_UART_Transmit(&GPS_UART_HANDLE, (uint8_t *)payload, len, HAL_MAX_DELAY);
    }
    uint8_t ck[2] = { ck_a, ck_b };
    HAL_UART_Transmit(&GPS_UART_HANDLE, ck, 2, HAL_MAX_DELAY);
}

// ---------- High-level message handlers ----------

static void handle_hnr_pvt(const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(ubx_hnr_pvt_t))
        return;

    ubx_hnr_pvt_t local;
    memcpy(&local, payload, sizeof(local));

    g_hnr_pvt = local;
    g_hnr_pvt_valid = true;

    // ★ 보드 공통 시간축: SysTick 기반 HAL tick 사용
    uint32_t host_now_ms = HAL_GetTick();

    // Update high-level fix with "fast" data
    gps_fix_basic_t fix = g_gps_fix;

    fix.iTOW_ms = local.iTOW;
    fix.year    = local.year;
    fix.month   = local.month;
    fix.day     = local.day;
    fix.hour    = local.hour;
    fix.min     = local.min;
    fix.sec     = local.sec;
    fix.raw_valid = local.valid;

    fix.fixType = local.gpsFix;
    fix.fixOk   = (local.flags & 0x01u) != 0u; // gpsFixOK

    fix.lon     = local.lon;
    fix.lat     = local.lat;
    fix.height  = local.height;
    fix.hMSL    = local.hMSL;
    fix.gSpeed  = local.gSpeed;
    fix.headMot = local.headMot;

    fix.hAcc    = local.hAcc;
    fix.vAcc    = local.vAcc;
    fix.sAcc    = local.sAcc;
    fix.headAcc = local.headAcc;

    // Valid if date+time valid and gpsFixOK and non-zero fix type
    bool date_time_valid = (local.valid & 0x03u) == 0x03u; // validDate | validTime
    fix.valid = date_time_valid && fix.fixOk && (fix.fixType != 0u);

    // ★ SysTick(HAL_GetTick) 기준 host timestamp 저장
    fix.host_time_ms = host_now_ms;


    g_gps_fix = fix;

    // LAT/LON 기반 파생 속도 업데이트 (기존 gSpeed는 그대로 둠)
    GPS_UBX_UpdateDerivedSpeedFromHnr(&local, host_now_ms);
    g_gps_fix_new = true;
}

static void handle_nav_pvt(const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(ubx_nav_pvt_t))
        return;

    ubx_nav_pvt_t local;
    memcpy(&local, payload, sizeof(local));

    g_nav_pvt = local;
    g_nav_pvt_valid = true;

    // NAV-PVT is slower (2 Hz) but gives us numSV + pDOP etc.
    gps_fix_basic_t fix = g_gps_fix;

    fix.iTOW_ms = local.iTOW;
    fix.year    = local.year;
    fix.month   = local.month;
    fix.day     = local.day;
    fix.hour    = local.hour;
    fix.min     = local.min;
    fix.sec     = local.sec;
    fix.raw_valid = local.valid;

    fix.fixType = local.fixType;
    fix.fixOk   = (local.flags & 0x01u) != 0u; // gnssFixOK
    fix.numSV_used    = local.numSV;

    fix.lon     = local.lon;
    fix.lat     = local.lat;
    fix.height  = local.height;
    fix.hMSL    = local.hMSL;
    fix.gSpeed  = local.gSpeed;
    fix.headMot = local.headMot;

    fix.hAcc    = local.hAcc;
    fix.vAcc    = local.vAcc;
    fix.sAcc    = local.sAcc;
    fix.headAcc = local.headAcc;
    fix.pDOP    = local.pDOP;

    bool date_time_valid = (local.valid & 0x03u) == 0x03u; // validDate | validTime
    bool has_pos_fix     = (fix.fixType >= 2u);            // 2D 이상만 인정 (1=DR-only는 제외해도 됨)

    fix.time_valid = date_time_valid;
    fix.valid      = has_pos_fix;

    g_gps_fix = fix;
    g_gps_fix_new = true;
}

// UBX-NAV-SAT: we only care about numSvs (visible / tracked)
static void handle_nav_sat(const uint8_t *payload, uint16_t len)
{
    if (len < 6u)
        return;

    uint8_t numSvs = payload[5];

    gps_fix_basic_t fix = g_gps_fix;
    fix.numSV_visible = numSvs;
    g_gps_fix = fix;
    g_gps_fix_new = true;
}

static void ubx_dispatch(uint8_t cls, uint8_t id, uint16_t len, const uint8_t *payload)
{
#if GPS_ENABLE_HNR
    if (cls == 0x28 && id == 0x00) {
        // UBX-HNR-PVT (M8U only)
        handle_hnr_pvt(payload, len);
    } else
#endif
    if (cls == 0x01 && id == 0x07) {
        // UBX-NAV-PVT
        handle_nav_pvt(payload, len);
    } else if (cls == 0x01 && id == 0x35) {
        // UBX-NAV-SAT
        handle_nav_sat(payload, len);
    } else {
        // ignore
    }
}

// ---------- Parser state machine ----------

void GPS_UBX_OnByte(uint8_t b)
{
    ubx_parser_t *p = &s_parser;

    switch (p->state)
    {
    case UBX_WAIT_SYNC1:
        if (b == 0xB5) {
            p->state = UBX_WAIT_SYNC2;
        }
        break;

    case UBX_WAIT_SYNC2:
        if (b == 0x62) {
            p->state = UBX_WAIT_CLASS;
            ubx_checksum_reset(p);
        } else {
            p->state = UBX_WAIT_SYNC1;
        }
        break;

    case UBX_WAIT_CLASS:
        p->cls = b;
        ubx_checksum_update(p, b);
        p->state = UBX_WAIT_ID;
        break;

    case UBX_WAIT_ID:
        p->id = b;
        ubx_checksum_update(p, b);
        p->state = UBX_WAIT_LEN1;
        break;

    case UBX_WAIT_LEN1:
        p->len = b;
        ubx_checksum_update(p, b);
        p->state = UBX_WAIT_LEN2;
        break;

    case UBX_WAIT_LEN2:
        p->len |= ((uint16_t)b << 8);
        ubx_checksum_update(p, b);

        if (p->len > GPS_UBX_MAX_PAYLOAD) {
            // drop frame
            ubx_parser_reset(p);
        } else if (p->len == 0) {
            p->state = UBX_WAIT_CK_A;
        } else {
            p->index = 0;
            p->state = UBX_WAIT_PAYLOAD;
        }
        break;

    case UBX_WAIT_PAYLOAD:
        p->payload[p->index++] = b;
        ubx_checksum_update(p, b);

        if (p->index >= p->len) {
            p->state = UBX_WAIT_CK_A;
        }
        break;

    case UBX_WAIT_CK_A:
        if (b == p->ck_a) {
            p->state = UBX_WAIT_CK_B;
        } else {
            ubx_parser_reset(p);
        }
        break;

    case UBX_WAIT_CK_B:
        if (b == p->ck_b) {
            // full frame OK
            ubx_dispatch(p->cls, p->id, p->len, p->payload);
        }
        ubx_parser_reset(p);
        break;

    default:
        ubx_parser_reset(p);
        break;
    }
}

// ---------- Public API ----------

void GPS_UBX_StartUartRx(void)
{
    HAL_UART_Receive_IT(&GPS_UART_HANDLE, &s_gps_rx_byte, 1);
}


// Configure the module
void GPS_UBX_InitAndConfigure(void)
{
    ubx_parser_reset(&s_parser);
    memset((void *)&g_gps_fix, 0, sizeof(g_gps_fix));
    g_gps_fix_new   = false;
    g_hnr_pvt_valid = false;
    g_nav_pvt_valid = false;

    // 모듈이 부팅 끝낼 시간 약간 주기
    HAL_Delay(1000);

    // --------------------------------------------------------------------
    // 1) UART auto-baud init: 9600 디폴트/115200 저장 둘 다 커버
    // --------------------------------------------------------------------
    // 전략:
    //   A. MCU UART를 9600으로 재초기화 → CFG-PRT(baud=115200)를 전송
    //      → 디폴트 9600 모듈은 이걸 먹고 115200로 전환
    //   B. MCU UART를 115200으로 재초기화 → 동일 CFG-PRT를 다시 전송
    //      → 이미 115200인 모듈은 이걸 먹고 설정 재확인
    //
    // 결과: 항상 양쪽 모두 115200 + UBX-only 상태로 수렴

    HAL_UART_DeInit(&GPS_UART_HANDLE);
    GPS_UART_HANDLE.Init.BaudRate = 9600;
    if (HAL_UART_Init(&GPS_UART_HANDLE) != HAL_OK)
    {
        // TODO: 에러 처리 (LED 깜빡이거나 assert 등)
    }

    // UBX-CFG-PRT (0x06 0x00), UART1, len=20
    struct __attribute__((packed)) cfg_prt_uart_t
    {
        uint8_t  portID;
        uint8_t  reserved0;
        uint16_t txReady;
        uint32_t mode;
        uint32_t baudrate;
        uint16_t inProtoMask;
        uint16_t outProtoMask;
        uint16_t flags;
        uint16_t reserved5;
    } cfg_prt_uart =
    {
        .portID      = 1,          // UART1
        .reserved0   = 0,
        .txReady     = 0,
        .mode        = 0x000008D0, // 8N1, no parity
        .baudrate    = 115200,     // 모듈 쪽 최종 baudrate
        .inProtoMask = 0x0001,     // UBX in
        .outProtoMask= 0x0001,     // UBX out
        .flags       = 0,
        .reserved5   = 0
    };

    // A) 9600 단계: 디폴트 모듈 잡기
    ubx_send(0x06, 0x00, &cfg_prt_uart, sizeof(cfg_prt_uart));
    HAL_Delay(200);

    // B) 115200 단계: 이미 115200로 저장된 모듈 잡기
    HAL_UART_DeInit(&GPS_UART_HANDLE);
    GPS_UART_HANDLE.Init.BaudRate = 115200;
    if (HAL_UART_Init(&GPS_UART_HANDLE) != HAL_OK)
    {
        // TODO: 에러 처리
    }

    ubx_send(0x06, 0x00, &cfg_prt_uart, sizeof(cfg_prt_uart));

    // --------------------------------------------------------------------
    // 2) 풀파워(연속 모드) 설정: UBX-CFG-RXM
    // --------------------------------------------------------------------
    // lpMode = 0 → continuous tracking / maximum performance
    struct __attribute__((packed)) cfg_rxm_t
    {
        uint8_t reserved;
        uint8_t lpMode;
    } cfg_rxm =
    {
        .reserved = 0,
        .lpMode   = 0   // continuous / max performance
    };
    ubx_send(0x06, 0x11, &cfg_rxm, sizeof(cfg_rxm));

    // --------------------------------------------------------------------
    // 2.5) GNSS 설정: GPS-only (UBX-CFG-GNSS)
    //    - NEO-M8N 기준: GPS 단일 GNSS로 10 Hz 운용
    // --------------------------------------------------------------------
    struct __attribute__((packed)) cfg_gnss_t
    {
        uint8_t msgVer;
        uint8_t numTrkChHw;
        uint8_t numTrkChUse;
        uint8_t numConfigBlocks;
        struct {
            uint8_t  gnssId;
            uint8_t  resTrkCh;
            uint8_t  maxTrkCh;
            uint8_t  reserved1;
            uint32_t flags;
        } block[1];
    } cfg_gnss =
    {
        .msgVer         = 0,     // 이 버전은 0
        .numTrkChHw     = 32,    // M8 계열에서 보통 32로 리포트됨 :contentReference[oaicite:2]{index=2}
        .numTrkChUse    = 32,    // 전부 사용
        .numConfigBlocks= 1,     // GPS 하나만 설정

        .block = {{
            .gnssId   = 0,       // 0 = GPS :contentReference[oaicite:3]{index=3}
            .resTrkCh = 8,       // u-blox 레퍼런스 값 (GPS용 최소 예약 채널) :contentReference[oaicite:4]{index=4}
            .maxTrkCh = 16,      // GPS에 할당할 최대 채널 수
            .reserved1= 0,
            // flags:
            //  - bit0: enable
            //  - 상위 비트: signal config mask
            //  - 0x00010000 = GPS L1C/A 사용
            //  → 0x00010001 = enable + L1C/A
            .flags    = 0x00010001u
        }}
    };

    ubx_send(0x06, 0x3E, &cfg_gnss, sizeof(cfg_gnss));

    // --------------------------------------------------------------------
    // 3) Navigation solution rate 설정: 10 Hz (100 ms)
    // --------------------------------------------------------------------
    // UBX-CFG-RATE (0x06 0x08)
    //
    // ⚠ NEO-M8N 주의:
    //    - GPS-only: 10 Hz 가능
    //    - GPS+GLONASS/Galileo 동시 사용시 ~5 Hz 제한이 있음
    //      (이 경우 모듈이 내부적으로 rate를 clamp할 수 있음)

    // NAV_RATE는 M8N일떄와 M8U일떄를 분리합니다.
    struct __attribute__((packed)) cfg_rate_t
    {
        uint16_t measRate;
        uint16_t navRate;
        uint16_t timeRef;
    } cfg_rate =
    {
        .measRate = GPS_NAV_RATE_MS, // M8N: 100ms(10Hz), M8U: 500ms(2Hz)
        .navRate  = 1,
        .timeRef  = 1
    };

    ubx_send(0x06, 0x08, &cfg_rate, sizeof(cfg_rate));


    // --------------------------------------------------------------------
    // 4) High Navigation Rate (UBX-CFG-HNR)
    // --------------------------------------------------------------------
    //  - NEO-M8N: HNR 엔진 없음 → IFDEF로 정의되어 있을 때만 활성화
    //  - NEO-M8U 등 HNR 지원 칩: GPS_HNR_RATE_HZ에 맞춰 동작

    // 4) High Navigation Rate (UBX-CFG-HNR)
    #if GPS_ENABLE_HNR
    struct __attribute__((packed)) cfg_hnr_t
    {
        uint8_t  highNavRate;
        uint8_t  reserved1;
        uint8_t  reserved2;
        uint8_t  reserved3;
    } cfg_hnr =
    {
        .highNavRate = (uint8_t)GPS_HNR_RATE_HZ, // M8U: 20 Hz
        .reserved1   = 0,
        .reserved2   = 0,
        .reserved3   = 0
    };

    ubx_send(0x06, 0x5C, &cfg_hnr, sizeof(cfg_hnr));
    #endif


    // --------------------------------------------------------------------
    // 5) 메시지 출력 설정
    //    - UBX-HNR-PVT (고속)
    //    - UBX-NAV-PVT (표준 PVT → 10 Hz)
    //    - UBX-NAV-SAT (1× per navRate)
    //    - NMEA 모두 disable
    // --------------------------------------------------------------------

    // 5) 메시지 출력 설정

    #if GPS_ENABLE_HNR
    // UBX-HNR-PVT enable (class 0x28 id 0x00), rate = 1 * highNavRate
    uint8_t cfg_msg_hnr_pvt[3] = { 0x28, 0x00, 1 };
    ubx_send(0x06, 0x01, cfg_msg_hnr_pvt, sizeof(cfg_msg_hnr_pvt));
    #endif

    // UBX-NAV-PVT enable (class 0x01 id 0x07)
    uint8_t cfg_msg_nav_pvt[3] = { 0x01, 0x07, 1 };
    ubx_send(0x06, 0x01, cfg_msg_nav_pvt, sizeof(cfg_msg_nav_pvt));

    // UBX-NAV-SAT enable (class 0x01 id 0x35)
    uint8_t cfg_msg_nav_sat[3] = { 0x01, 0x35, 1 };
    ubx_send(0x06, 0x01, cfg_msg_nav_sat, sizeof(cfg_msg_nav_sat));

    // NMEA 끄기 (GGA/GLL/GSA/GSV/RMC/VTG)
    uint8_t cfg_msg_nmea_gga[3] = { 0xF0, 0x00, 0 }; // NMEA-GxGGA
    uint8_t cfg_msg_nmea_gll[3] = { 0xF0, 0x01, 0 }; // NMEA-GxGLL
    uint8_t cfg_msg_nmea_gsa[3] = { 0xF0, 0x02, 0 }; // NMEA-GxGSA
    uint8_t cfg_msg_nmea_gsv[3] = { 0xF0, 0x03, 0 }; // NMEA-GxGSV
    uint8_t cfg_msg_nmea_rmc[3] = { 0xF0, 0x04, 0 }; // NMEA-GxRMC
    uint8_t cfg_msg_nmea_vtg[3] = { 0xF0, 0x05, 0 }; // NMEA-GxVTG

    ubx_send(0x06, 0x01, cfg_msg_nmea_gga, sizeof(cfg_msg_nmea_gga));
    ubx_send(0x06, 0x01, cfg_msg_nmea_gll, sizeof(cfg_msg_nmea_gll));
    ubx_send(0x06, 0x01, cfg_msg_nmea_gsa, sizeof(cfg_msg_nmea_gsa));
    ubx_send(0x06, 0x01, cfg_msg_nmea_gsv, sizeof(cfg_msg_nmea_gsv));
    ubx_send(0x06, 0x01, cfg_msg_nmea_rmc, sizeof(cfg_msg_nmea_rmc));
    ubx_send(0x06, 0x01, cfg_msg_nmea_vtg, sizeof(cfg_msg_nmea_vtg));

    // --------------------------------------------------------------------
    // 6) RX 시작
    // --------------------------------------------------------------------
    // gps_ubx.c 안의 s_gps_rx_byte 버퍼는 사실 main.c에서 다시 덮으니까,
    // 이 호출은 거의 의미 없지만 있어도 문제는 없음.
    GPS_UBX_StartUartRx();
}


// Atomic copy of latest fix
bool GPS_UBX_GetLatestFix(gps_fix_basic_t *out)
{
    bool has_new = false;

    __disable_irq();
    if (g_gps_fix_new) {
        if (out != NULL) {
            *out = g_gps_fix;
        }
        g_gps_fix_new = false;
        has_new = true;
    }
    __enable_irq();

    return has_new;
}

// HAL callback: feed bytes into UBX parser and toggle debug pin

// ---------- Derived speed & heading from LLH (HNR) ----------

// LLH 기반 파생 속도/헤딩용 상태
typedef struct
{
    uint8_t  has_prev;
    int32_t  prev_lat;        // 1e-7 deg
    int32_t  prev_lon;        // 1e-7 deg
    uint32_t prev_host_ms;    // HAL_GetTick() 기준 이전 샘플 시각 [ms]
    float    filt_speed_mps;  // 저역필터된 수평 속도 [m/s]
    float    last_heading_deg;
    uint8_t  heading_valid;
} gps_llh_state_t;

static gps_llh_state_t s_llh_state;


static inline float gps_deg1e7_to_rad(int32_t v_1e7)
{
    const float DEG_TO_RAD = (float)M_PI / 180.0f;
    return (float)v_1e7 * (1e-7f * DEG_TO_RAD);
}

// 두 위경도 점 사이의 수평 거리 [m] (local tangent plane 근사)
static float gps_llh_distance_m(int32_t lat1_1e7, int32_t lon1_1e7,
                                int32_t lat2_1e7, int32_t lon2_1e7)
{
    const float R = 6371000.0f;  // Earth radius [m]

    float lat1 = gps_deg1e7_to_rad(lat1_1e7);
    float lon1 = gps_deg1e7_to_rad(lon1_1e7);
    float lat2 = gps_deg1e7_to_rad(lat2_1e7);
    float lon2 = gps_deg1e7_to_rad(lon2_1e7);

    float dlat = lat2 - lat1;
    float dlon = lon2 - lon1;
    float cos_lat = cosf(0.5f * (lat1 + lat2));

    float north = R * dlat;
    float east  = R * dlon * cos_lat;

    return sqrtf(north * north + east * east);
}

// 점1 → 점2로 가는 bearing [deg], 0 = North, 90 = East
static float gps_llh_heading_deg(int32_t lat1_1e7, int32_t lon1_1e7,
                                 int32_t lat2_1e7, int32_t lon2_1e7)
{
    float lat1 = gps_deg1e7_to_rad(lat1_1e7);
    float lon1 = gps_deg1e7_to_rad(lon1_1e7);
    float lat2 = gps_deg1e7_to_rad(lat2_1e7);
    float lon2 = gps_deg1e7_to_rad(lon2_1e7);

    float dlat = lat2 - lat1;
    float dlon = lon2 - lon1;
    float cos_lat = cosf(0.5f * (lat1 + lat2));

    float north = dlat;
    float east  = dlon * cos_lat;

    float heading_rad = atan2f(east, north);
    float heading_deg = heading_rad * (180.0f / (float)M_PI);

    if (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }
    return heading_deg;
}

// HNR-PVT 한 샘플 들어올 때마다 호출해서 파생 속도/헤딩 업데이트.
// - 기존 gSpeed/headMot는 건드리지 않고,
//   g_gps_fix.speed_llh_*, heading_llh_*만 갱신.
// HNR-PVT 한 샘플 들어올 때마다 호출해서 LAT/LON 기반 파생 속도/헤딩 업데이트.
// - 기존 gSpeed/headMot (칩이 직접 주는 값)는 건드리지 않고,
//   g_gps_fix.speed_llh_*, heading_llh_*만 갱신한다.
static void GPS_UBX_UpdateDerivedSpeedFromHnr(const ubx_hnr_pvt_t *hnr,
                                              uint32_t host_time_ms)
{
    gps_llh_state_t *s = &s_llh_state;

    // 1) GPS fix 유효성 체크
    //    - gpsFix != 0 && flags의 bit0(gpsFixOK) 세트일 때만 사용
    bool fix_ok = ((hnr->gpsFix != 0u) && ((hnr->flags & 0x01u) != 0u));
    if (!fix_ok) {
        // fix가 깨지면 상태 리셋하고 파생 속도/헤딩도 invalid로
        s->has_prev       = 0;
        s->filt_speed_mps = 0.0f;
        s->heading_valid  = 0;

        g_gps_fix.speed_llh_mps     = 0.0f;
        g_gps_fix.speed_llh_kmh     = 0.0f;
        g_gps_fix.heading_llh_valid = 0;
        // heading_llh_deg는 마지막 값 그대로 두고 싶으면 유지, 완전 리셋하려면 0.0f로 초기화해도 됨.
        return;
    }

    // 2) 첫 샘플 초기화
    if (!s->has_prev) {
        s->prev_lat         = hnr->lat;
        s->prev_lon         = hnr->lon;
        s->prev_host_ms     = host_time_ms;
        s->filt_speed_mps   = 0.0f;
        s->last_heading_deg = 0.0f;
        s->heading_valid    = 0;

        g_gps_fix.speed_llh_mps     = 0.0f;
        g_gps_fix.speed_llh_kmh     = 0.0f;
        g_gps_fix.heading_llh_deg   = 0.0f;
        g_gps_fix.heading_llh_valid = 0;

        s->has_prev = 1;
        return;
    }

    // 3) dt 계산: HAL_GetTick() 기반, unsigned wrap-around 안전
    uint32_t dt_ms = (uint32_t)(host_time_ms - s->prev_host_ms);

    // 20 Hz HNR 기준 정상 dt는 약 50 ms.
    // 너무 작거나 너무 크면 글리치/버스트로 보고 필터 상태만 carry 하고 위치 기준점만 갱신.
    if (dt_ms < 10u || dt_ms > 200u) {
        s->prev_lat     = hnr->lat;
        s->prev_lon     = hnr->lon;
        s->prev_host_ms = host_time_ms;

        // 필터 상태는 그대로 유지
        g_gps_fix.speed_llh_mps     = s->filt_speed_mps;
        g_gps_fix.speed_llh_kmh     = s->filt_speed_mps * 3.6f;
        g_gps_fix.heading_llh_deg   = s->last_heading_deg;
        g_gps_fix.heading_llh_valid = s->heading_valid;
        return;
    }

    float dt = (float)dt_ms * 1e-3f;

    // 4) 수평 거리 [m]
    float dist_m = gps_llh_distance_m(s->prev_lat, s->prev_lon,
                                      hnr->lat, hnr->lon);

    float v_mps = 0.0f;
    if (dt > 0.0f) {
        v_mps = dist_m / dt;
    }

    // 5) 말도 안 되는 outlier 제거 (예: 150 m/s ≈ 540 km/h 이상)
    if (v_mps > 150.0f) {
        v_mps = s->filt_speed_mps;
    }

    // 6) 1차 저역필터 (tau ≈ 0.30 s)
    const float tau   = 0.30f;
    const float alpha = dt / (tau + dt);   // dt > 0이라서 0 < alpha < 1

    s->filt_speed_mps += alpha * (v_mps - s->filt_speed_mps);

    g_gps_fix.speed_llh_mps = s->filt_speed_mps;
    g_gps_fix.speed_llh_kmh = s->filt_speed_mps * 3.6f;

    // 7) 헤딩: 속도 ≥ 2 km/h 일 때만 업데이트
    const float speed_kmh = g_gps_fix.speed_llh_kmh;
    if (speed_kmh >= 2.0f) {
        float heading_deg = gps_llh_heading_deg(s->prev_lat, s->prev_lon,
                                                hnr->lat, hnr->lon);
        if (heading_deg < 0.0f) {
            heading_deg += 360.0f;
        } else if (heading_deg >= 360.0f) {
            heading_deg -= 360.0f;
        }
        s->last_heading_deg = heading_deg;
        s->heading_valid    = 1;
    }

    g_gps_fix.heading_llh_deg   = s->last_heading_deg;
    g_gps_fix.heading_llh_valid = s->heading_valid;

    // 8) 상태 업데이트
    s->prev_lat     = hnr->lat;
    s->prev_lon     = hnr->lon;
    s->prev_host_ms = host_time_ms;
}








///

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &GPS_UART_HANDLE) {
        // 1바이트 파서에 밀어 넣기
        GPS_UBX_OnByte(s_gps_rx_byte);

        // 디버그 토글 (원하면)
        gps_debug_pulse();

        // 다음 바이트 다시 받기
        HAL_UART_Receive_IT(&GPS_UART_HANDLE, &s_gps_rx_byte, 1);
    }
}


