/*
 * app_display.h
 */

#ifndef INC_APP_DISPLAY_H_
#define INC_APP_DISPLAY_H_

#include <stdint.h>
#include <stdbool.h>
#include "gps_app.h"

#ifdef __cplusplus
extern "C" {
#endif

// 7-seg 8자리 화면 모드 정의
typedef enum {
    APP_DISPLAY_SAT_STATUS = 0,
    APP_DISPLAY_SPEED,
    APP_DISPLAY_ALTITUDE,
    APP_DISPLAY_HEADING,
    APP_DISPLAY_GRADE,
    APP_DISPLAY_DISTANCE,
    APP_DISPLAY_TOP_SPEED,
    APP_DISPLAY_AVG_SPEED,
    APP_DISPLAY_ZERO_TO_100,
    APP_DISPLAY_TRIP_TIME,
    APP_DISPLAY_LOCAL_TIME,
    APP_DISPLAY_LOCAL_DATE,
    APP_DISPLAY_LATLON,           // ★ 새 위경도 모드
    APP_DISPLAY_SPEED_AND_GRADE,
    APP_DISPLAY_SPEED_AND_HEADING,
    APP_DISPLAY_SPEED_AND_ALTITUDE,
    APP_DISPLAY_MODE_COUNT
} app_display_mode_t;

// 현재 표시 모드 (버튼 ISR 등에서 바꿀 전역)
extern volatile app_display_mode_t g_display_mode;

extern volatile uint8_t g_blink_2hz;
extern volatile uint8_t g_blink_5hz;

typedef enum
{
    APP_DISPLAY_BANK_SINGLE = 0,   // 1개 필드 화면들
    APP_DISPLAY_BANK_DUAL,         // 2개 필드 화면들
    APP_DISPLAY_BANK_ZTO100,       // 0~100 제로백 측정 모드
    APP_DISPLAY_BANK_COUNT
} app_display_bank_t;

// 현재 선택된 뱅크 (길게 눌러서 전환)
extern volatile app_display_bank_t g_display_bank;
// 새로 추가: 뱅크 전환용
void APP_Display_NextBank(void);

// 100 ms 타이머 틱 (TIM3 인터럽트에서 호출)
void APP_Display_BlinkTick_100ms(void);

// 한 번만 호출: 내부 상태 초기화
void APP_Display_Init(void);

// main loop에서 주기적으로 호출
void APP_Display_Update(void);

// 모드를 직접 지정
void APP_Display_SetMode(app_display_mode_t mode);

// 다음 모드로 순환
void APP_Display_NextMode(void);

// 시간/날짜 모드 타임존 설정 (예: 9 => UTC+9)
void APP_Display_SetTimezone(int8_t offset_hours);

// GLOBAL 밝기 레벨 (1~3) 설정 + MAX7219 밝기 테이블과 연동
void APP_Display_SetBrightnessLevel(uint8_t level_1_to_3);

// 부팅 시 "brIgHt" 텍스트로 밝기 스윕
void APP_Display_RunBrightnessSweepTest(void);

// AUTO 모드 on/off
void APP_Display_SetAutoModeEnabled(bool enabled);
void APP_Display_ToggleAutoMode(void);
bool APP_Display_IsAutoModeEnabled(void);


// 부팅 설정 메뉴용 UI
void APP_Display_ShowSetupGMT(int8_t offset_hours);
void APP_Display_ShowSetupBrightness(uint8_t level_1_to_3);
void APP_Display_ShowSetupAutoMode(bool enabled);


void APP_Display_ShowSetupBeepVolume(uint8_t vol_0_to_4);
void APP_Display_ShowSetupHwTest(void);

// 2 Hz 화면 속도 클램프 on/off
void APP_Display_SetSpeedClamp2HzEnabled(bool enabled);

// 플래시 데이터 에러 표시용 ("dAtA Err")
void APP_Display_ShowDataError(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_APP_DISPLAY_H_ */
