/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * 차량용 GPS 기기용 코드 입니다.
  *
  * 심심해서 장난감삼아 만들어보는 것이므로 성능을 신경쓰지 않고 개발하고 잇음.
  *
  * SECTOR 4를 사용하지 않습니다. DEVICE TEST (dEU TEST) 에서 SECTOR 4를 덮어쓰고 작성하는 방식으로 구현합니다. 따라서 총 코드 사이즈를 64KB 미만으로 사용해야 합니다.
  *
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gps_app.h"
#include "app_display.h"
#include "buzzer.h"
#include "max7219.h"

#include "settings_storage.h"   // ★ 추가

#include "hw_test.h"   // ★ dEu TEST 구현
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// 설정값 전역 변수
int8_t  g_cfg_timezone_hours = 9;   // GMT TIME, 기본 +9 (한국)
uint8_t g_cfg_brightness     = 2;   // MAX7219 밝기 레벨 1~3
uint8_t g_cfg_auto_mode      = 0;   // AUTO MODE: 0=off, 1=on

// Beep volume: 0~4 (0=mute, 4=max)
uint8_t g_cfg_beep_volume    = 4;



// 수정 후
typedef enum {
    SETUP_MENU_GMT = 0,
    SETUP_MENU_BRIGHTNESS,
    SETUP_MENU_AUTO,
    SETUP_MENU_BEEP_VOL,
    SETUP_MENU_HWTEST
} setup_menu_t;















typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT,
    KEY_EVENT_LONG
} key_event_t;

typedef struct {
    uint8_t  stable_level;     // 디바운스 이후 안정 레벨 (1: released, 0: pressed)
    uint8_t  last_sample;      // 마지막 raw 샘플
    uint32_t last_change_ms;   // last_sample 변경 시각
    uint8_t  pressed;          // 눌린 상태로 인정 중인지
    uint32_t press_start_ms;   // 눌리기 시작한 시각
    uint8_t  long_sent;        // long event 이미 보냈는지
} key_state_t;

static key_state_t s_key_pa0;

#define KEY_DEBOUNCE_MS      30u
#define KEY_LONGPRESS_MS    800u
#define KEY_SHORT_MIN_MS     50u   // 50ms 이하는 노이즈로 무시

// PA0 raw 읽기 (풀업 + 스위치 → GND 가정)
static inline uint8_t Key_PA0_ReadRaw(void)
{
    GPIO_PinState s = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    return (s == GPIO_PIN_RESET) ? 0u : 1u;  // 0 = pressed
}

static void Key_Init(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t lvl  = Key_PA0_ReadRaw();

    memset(&s_key_pa0, 0, sizeof(s_key_pa0));
    s_key_pa0.stable_level   = lvl;
    s_key_pa0.last_sample    = lvl;
    s_key_pa0.last_change_ms = now;
}

static key_event_t Key_Poll(uint32_t now_ms)
{
    key_event_t ev = KEY_EVENT_NONE;
    uint8_t raw = Key_PA0_ReadRaw();

    // 샘플 변경 시 디바운스 타이머 리셋
    if (raw != s_key_pa0.last_sample) {
        s_key_pa0.last_sample    = raw;
        s_key_pa0.last_change_ms = now_ms;
    }

    // 디바운스 완료되면 stable_level 반영
    if ((now_ms - s_key_pa0.last_change_ms) >= KEY_DEBOUNCE_MS) {
        if (s_key_pa0.stable_level != raw) {
            s_key_pa0.stable_level = raw;

            if (raw == 0u) {
                // 눌림 시작
                s_key_pa0.pressed        = 1u;
                s_key_pa0.press_start_ms = now_ms;
                s_key_pa0.long_sent      = 0u;
            } else {
                // 떼어짐
                if (s_key_pa0.pressed) {
                    uint32_t dur = now_ms - s_key_pa0.press_start_ms;

                    if (!s_key_pa0.long_sent &&
                        dur >= KEY_SHORT_MIN_MS &&
                        dur < KEY_LONGPRESS_MS) {
                        ev = KEY_EVENT_SHORT;
                    }
                    s_key_pa0.pressed = 0u;
                }
            }
        }
    }

    // 길게 누르기 판정
    if (s_key_pa0.pressed && !s_key_pa0.long_sent) {
        uint32_t dur = now_ms - s_key_pa0.press_start_ms;
        if (dur >= KEY_LONGPRESS_MS) {
            s_key_pa0.long_sent = 1u;
            ev = KEY_EVENT_LONG;
        }
    }

    return ev;
}








// ---------------------- 부팅 설정 모드 ----------------------

static void RunHardwareSelfTest(void)
{
    /* dEu tESt: FLASH + RAM 테스트 1회 실행 (블로킹) */
    HW_RunSelfTest();
}


static void EnterSetupMode(void)
{
    setup_menu_t menu = SETUP_MENU_GMT;
    uint8_t      done = 0u;

    // 설정 모드 진입 시 화면 한번 깨끗하게
    max7219_Clean();

    while (!done) {
        uint32_t now = HAL_GetTick();

        // 현재 메뉴 화면 표시
        switch (menu) {
        case SETUP_MENU_GMT:
            APP_Display_ShowSetupGMT(g_cfg_timezone_hours);
            break;

        case SETUP_MENU_BRIGHTNESS:
            APP_Display_ShowSetupBrightness(g_cfg_brightness);
            break;

        case SETUP_MENU_AUTO:
            APP_Display_ShowSetupAutoMode((g_cfg_auto_mode != 0u));
            break;

        case SETUP_MENU_BEEP_VOL:
            APP_Display_ShowSetupBeepVolume(g_cfg_beep_volume);
            break;

        case SETUP_MENU_HWTEST:
            APP_Display_ShowSetupHwTest();
            break;

        default:
            done = 1u;
            continue;
        }

        // 버튼 이벤트 처리
        key_event_t kev = Key_Poll(now);
        if (kev == KEY_EVENT_SHORT) {
            switch (menu) {
            case SETUP_MENU_GMT:
                // 타임존 +1씩 증가, 끝까지 가면 -12로 순환
                if (g_cfg_timezone_hours < 14) {
                    g_cfg_timezone_hours++;
                } else {
                    g_cfg_timezone_hours = -12;
                }
                APP_Display_SetTimezone(g_cfg_timezone_hours);
                break;

            case SETUP_MENU_BRIGHTNESS:
                // 1 → 2 → 3 → 1 순환
                if (g_cfg_brightness < 3u) {
                    g_cfg_brightness++;
                } else {
                    g_cfg_brightness = 1u;
                }
                // 바로 MAX7219 글로벌 밝기 테이블에 반영
                APP_Display_SetBrightnessLevel(g_cfg_brightness);
                break;

            case SETUP_MENU_AUTO:
                // on/off 토글
                g_cfg_auto_mode ^= 1u;
                APP_Display_SetAutoModeEnabled((bool)(g_cfg_auto_mode != 0u));
                break;

            case SETUP_MENU_BEEP_VOL:
                // 0 → 1 → 2 → 3 → 4 → 0 순환 (0=mute, 4=max)
                if (g_cfg_beep_volume < 4u) {
                    g_cfg_beep_volume++;
                } else {
                    g_cfg_beep_volume = 0u;
                }

                APP_Display_ShowSetupBeepVolume(g_cfg_beep_volume);
                Buzzer_SetVolume(g_cfg_beep_volume);
                break;


            case SETUP_MENU_HWTEST:
            	RunHardwareSelfTest();
                break;

            default:
                break;
            }
        } else if (kev == KEY_EVENT_LONG) {
            switch (menu) {
            case SETUP_MENU_GMT:
            case SETUP_MENU_BRIGHTNESS:
            case SETUP_MENU_AUTO:
            	menu = (setup_menu_t)((int)menu + 1);

                break;

            case SETUP_MENU_BEEP_VOL:
                menu = (setup_menu_t)((int)menu + 1);
                break;

            case SETUP_MENU_HWTEST:
                // 하드웨어 테스트 1회 실행 후 SAT STATUS로 탈출
                done = 1u;
                break;


            default:
                done = 1u;
                break;
            }
        }

        // GPS 상태는 계속 갱신해서 FIX 유지
        APP_GPS_Update();

        // 너무 빠른 루프 방지
        HAL_Delay(10);
    }

    // 설정이 끝났으니 현재 전역 설정값을 플래시에 저장
    app_settings_t cfg;
    // 수정 후
    cfg.timezone_hours = g_cfg_timezone_hours;
    cfg.brightness     = g_cfg_brightness;
    cfg.auto_mode      = g_cfg_auto_mode;
    cfg.beep_volume    = g_cfg_beep_volume;

    Settings_Save(&cfg);
    // 설정 메뉴를 모두 지나치면 SAT STATUS 화면으로
    APP_Display_SetMode(APP_DISPLAY_SAT_STATUS);
}

static void CheckBootAndEnterSetup(void)
{
    const uint32_t window_ms    = 1000u;  // 부팅 후 체크 전체 윈도우
    const uint32_t min_press_ms = 150u;   // 이 시간 이상 연속 LOW여야 인정

    uint32_t start      = HAL_GetTick();
    uint32_t low_start  = 0u;
    uint8_t  enter      = 0u;

    while ((HAL_GetTick() - start) < window_ms) {
        uint32_t now = HAL_GetTick();
        uint8_t  raw = Key_PA0_ReadRaw();  // 0 = pressed (active-low)

        if (raw == 0u) {  // LOW = 눌림
            if (low_start == 0u) {
                // 처음 LOW로 떨어진 시점 기억
                low_start = now;
            } else {
                // LOW 상태가 min_press_ms 이상 유지되면 "진짜로 누르고 있음"
                if ((now - low_start) >= min_press_ms) {
                    enter = 1u;
                    break;
                }
            }
        } else {
            // 중간에 한 번이라도 HIGH로 올라가면 연속성 끊기므로 초기화
            low_start = 0u;
        }
    }

    if (enter) {
        EnterSetupMode();
    }
}







void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // 100 ms tick
        APP_Display_BlinkTick_100ms();
        Buzzer_Tick_100ms();   // 3번에서 만들 Buzzer 모듈
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);



  // MAX7219 초기화
  max7219_Init(0x08);   // 밝기: 0x00 ~ 0x0F
  max7219_Clean();

  APP_Display_RunBrightnessSweepTest();



  APP_GPS_Init();
  APP_Display_Init();




  app_settings_t stored;
  bool loaded = Settings_Load(&stored);
  if (loaded) {
      g_cfg_timezone_hours = stored.timezone_hours;
      g_cfg_brightness     = stored.brightness;
      g_cfg_auto_mode      = stored.auto_mode;

      uint8_t v = stored.beep_volume;
      if (v > 4u) {
          v = 4u;  // 범위 밖이면 최대 볼륨
      }
      g_cfg_beep_volume = v;
  } else {
      // 설정이 없으면: 기본 비프 볼륨 4
      g_cfg_beep_volume = 4u;
  }

  if (g_cfg_beep_volume > 4u) {
      g_cfg_beep_volume = 4u;
  }

  // 속도 클램프는 이제 별도 설정 없이 고정 (기본: off)
  APP_Display_SetSpeedClamp2HzEnabled(false);

  APP_Display_SetTimezone(g_cfg_timezone_hours);
  APP_Display_SetAutoModeEnabled(g_cfg_auto_mode != 0u);
  APP_Display_SetBrightnessLevel(g_cfg_brightness);

  Key_Init();          // 버튼 초기화
  Buzzer_SetVolume(g_cfg_beep_volume);
  Buzzer_Init();       // Buzzer 시작

  HAL_Delay(400);

  Buzzer_PlaySequence(BEEP_SEQ_START);  // 전원 ON 멜로디

  CheckBootAndEnterSetup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	    uint32_t now = HAL_GetTick();

	    // 키 폴링
	    key_event_t kev = Key_Poll(now);
	    if (kev == KEY_EVENT_SHORT) {
	        // 짧게 누름 → 메뉴 한 칸 증가
	        APP_Display_NextMode();
	        Buzzer_PlaySequence(BEEP_SEQ_USER8);
	    } else if (kev == KEY_EVENT_LONG) {
	        // 길게 누름
	        //  - 뱅크 전환:
	        //    1 FIELD → 2 FIELD → 0 to 100 → 1 FIELD → ...
	        //  - 전환 시 "1 FIELd" / "2 FIELd" / "0 to 100" 잠깐 표시
	        APP_Display_NextBank();
	        Buzzer_PlaySequence(BEEP_SEQ_USER9);
	    }

	    // GPS 상태 갱신
	    APP_GPS_Update();

	    // 현재 모드에 따라 7-seg 화면 업데이트
	    APP_Display_Update();


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
