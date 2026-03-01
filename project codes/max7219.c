/*
 * max7219.c
 *
 *  Created on: May 11, 2019
 *      Author: tabur
 */

#include <max7219.h>

#define CS_SET() 	HAL_GPIO_WritePin(CS_MAX7219_GPIO_Port, CS_MAX7219_Pin, GPIO_PIN_RESET)
#define CS_RESET() 	HAL_GPIO_WritePin(CS_MAX7219_GPIO_Port, CS_MAX7219_Pin, GPIO_PIN_SET)

static uint8_t decodeMode = 0x00;  // 0xFF": MAX7219에 내장된 BCD DECODE MODE사용 / 0x00: 생 비트를 그대로 쳐찍는모드

// ★ 화면 더티 캐시
static char    s_digit_cache[NUMBER_OF_DIGITS];
static uint8_t s_dot_mask;
static uint8_t s_cache_initialized = 0;

static void max7219_cache_clear(void)
{
    // "아직 아무 것도 모른다" 상태로 리셋
    for (int i = 0; i < NUMBER_OF_DIGITS; ++i) {
        s_digit_cache[i] = 0;   // 사용하지 않는 값
    }
    s_dot_mask = 0;
    s_cache_initialized = 1;
}

// 7-segment bit mapping
#define SEG_A   0x40u
#define SEG_B   0x20u
#define SEG_C   0x10u
#define SEG_D   0x08u
#define SEG_E   0x04u
#define SEG_F   0x02u
#define SEG_G   0x01u
#define SEG_DP  0x80u

// SYMBOLS[] 인덱스용 매크로 (순서 바꾸면 같이 수정해야 함)
#define SYMBOL_MINUS      10u   // 0~9 다음 MINUS
#define SYMBOL_CARET      23u   // 마지막 SYMBOL_CARET (^)


// 0~9
static const uint8_t FONT_7SEG_DIGITS[10] = {
    0x7E, // 0 : a b c d e f
    0x30, // 1 : b c
    0x6D, // 2 : a b d e g
    0x79, // 3 : a b c d g
    0x33, // 4 : f g b c
    0x5B, // 5 : a f g c d
    0x5F, // 6 : a f e d c g
    0x70, // 7 : a b c
    0x7F, // 8 : a b c d e f g
    0x7B  // 9 : a b c d f g
};

static const uint8_t FONT_7SEG_ALPHA[26] = {
    0x77, // A : a b c e f g
    0x1F, // B : c d e f g (소문자 b 느낌)
    0x4E, // C : a f e d
    0x3D, // D : b c d e g (소문자 d 느낌)
    0x4F, // E : a f e d g
    0x47, // F : a f e g
    0x5E, // G : a f e d c
    0x37, // H : f e b c g
    0x30, // I : b c (1이랑 같지만 읽기는 쉬움)
    0x38, // J : b c d
    0x07, // K : f e g (대충 K 느낌)
    0x0E, // L : f e d
    0x56, // M : a c e f (타협형)
    0x15, // N : c e g (타협형)
    0x7E, // O : a b c d e f (0과 동일)
    0x67, // P : a b f e g
    0x73, // Q : a b c f g
    0x05, // R : e g
    0x5B, // S : a f g c d (5와 동일)
    0x0F, // T : f e d g
    0x3E, // U : b c d e f
    0x1C, // V : c d e
    0x2A, // W : b d f (타협형)
    0x37, // X : b c e f g (H랑 같지만 교차 느낌)
    0x3B, // Y : b c d f g
    0x6D  // Z : a b d e g (2와 동일)
};

static uint8_t SYMBOLS[] = {
    0x7E, // NUM_0
    0x30, // NUM_1
    0x6D, // NUM_2
    0x79, // NUM_3
    0x33, // NUM_4
    0x5B, // NUM_5
    0x5F, // NUM_6
    0x70, // NUM_7
    0x7F, // NUM_8
    0x7B, // NUM_9
    0x01, // MINUS
    0x4F, // LETTER_E
    0x37, // LETTER_H
    0x0E, // LETTER_L
    0x67, // LETTER_P
    0x00, // BLANK

    // ==== 여기부터 커스텀 ====
    0x5B, // LETTER_S
    0x77, // LETTER_A
    0x0F, // LETTER_T
    0x7B, // LETTER_G
    0x05, // LETTER_R
    0x3D, // LETTER_d
    0x3C, // LETTER_u
	0x63  // SYMBOL_CARET (^)


};

static uint8_t max7219_font_from_ascii(char c)
{
    // 숫자
    if (c >= '0' && c <= '9') {
        return FONT_7SEG_DIGITS[(uint8_t)(c - '0')];
    }

    // 알파벳 (대소문자 공통 처리)
    if (c >= 'A' && c <= 'Z') {
        return FONT_7SEG_ALPHA[(uint8_t)(c - 'A')];
    }
    if (c >= 'a' && c <= 'z') {
        return FONT_7SEG_ALPHA[(uint8_t)(c - 'a')];
    }

    // 기호 몇 개
    if (c == '-') {
        return SYMBOLS[SYMBOL_MINUS];
    }
    if (c == ' ') {
        return 0x00u;      // blank
    }
    if (c == '^') {
        return SYMBOLS[SYMBOL_CARET];   // 위 동그라미 모양
    }


    // 정의 안 된 글자는 blank
    return 0x00u;
}

// 0~7 논리 위치 -> MAX7219 digit 레지스터 (1~8)
// pos 0 = 가장 왼쪽, pos 7 = 가장 오른쪽
static uint8_t max7219_pos_to_digit(uint8_t pos)
{
    if (pos >= NUMBER_OF_DIGITS) {
        pos = NUMBER_OF_DIGITS - 1;
    }
    // NUMBER_OF_DIGITS == 8 이라고 가정
    return (uint8_t)(NUMBER_OF_DIGITS - pos);  // 0 -> 8, 7 -> 1
}

void max7219_WriteCharAt(uint8_t pos, char ch, bool point)
{
    if (pos >= NUMBER_OF_DIGITS) {
        return;
    }

    if (!s_cache_initialized) {
        max7219_cache_clear();
    }

    uint8_t bit = (uint8_t)(1u << pos);

    // 새 dot 마스크 계산
    uint8_t new_mask = s_dot_mask;
    if (point) {
        new_mask |= bit;
    } else {
        new_mask &= (uint8_t)~bit;
    }

    char    prev_ch  = s_digit_cache[pos];
    uint8_t prev_bit = (uint8_t)(s_dot_mask & bit);
    uint8_t new_bit  = (uint8_t)(new_mask    & bit);

    // ★ 캐시와 완전히 동일하면 SPI 전송 스킵
    if (prev_ch == ch && prev_bit == new_bit) {
        s_dot_mask = new_mask;  // 마스크는 항상 최신 값으로 유지
        return;
    }

    // 캐시 업데이트
    s_digit_cache[pos] = ch;
    s_dot_mask         = new_mask;

    // 실제 하드웨어로 전송
    uint8_t digit = max7219_pos_to_digit(pos);  // 0~7 -> 1~8
    max7219_WriteChar((MAX7219_Digits)digit, ch, point);
}





static uint16_t getSymbol(uint8_t number);
static uint32_t lcdPow10(uint8_t n);

void max7219_Init(uint8_t intensivity)
{
    max7219_Turn_On();

    // 디코드 모드 항상 OFF
    decodeMode = 0x00;
    max7219_SendData(REG_DECODE_MODE, decodeMode);

    max7219_SendData(REG_SCAN_LIMIT, NUMBER_OF_DIGITS - 1);
    max7219_SetIntensivity(intensivity);

    // 하드웨어 클리어
    max7219_Clean();

    // ★ 소프트웨어 캐시도 초기화
    max7219_cache_clear();
}



void max7219_SetIntensivity(uint8_t intensivity)
{
	if (intensivity > 0x0F)
	{
		return;
	}

	max7219_SendData(REG_INTENSITY, intensivity);
}

static void max7219_cache_clear(void); // 이미 위에 static 있음

void max7219_Clean(void)
{
    uint8_t clear = 0x00;

    if (decodeMode == 0xFF)
    {
        clear = BLANK;
    }

    for (int i = 0; i < NUMBER_OF_DIGITS; ++i)
    {
        max7219_SendData(i + 1, clear);
    }

    // ★ 하드웨어 클리어할 때 캐시도 같이 초기화
    max7219_cache_clear();
}


void max7219_SendData(uint8_t addr, uint8_t data)
{
	CS_SET();
	HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
	CS_RESET();
}

void max7219_Turn_On(void)
{
	max7219_SendData(REG_SHUTDOWN, 0x01);
}

void max7219_Turn_Off(void)
{
	max7219_SendData(REG_SHUTDOWN, 0x00);
}

void max7219_Decode_On(void)
{
	decodeMode = 0xFF;
	max7219_SendData(REG_DECODE_MODE, decodeMode);
}

void max7219_Decode_Off(void)
{
	decodeMode = 0x00;
	max7219_SendData(REG_DECODE_MODE, decodeMode);
}

void max7219_PrintDigit(MAX7219_Digits position, MAX7219_Numeric numeric, bool point)
{
	if(position > NUMBER_OF_DIGITS)
	{
		return;
	}

	if(point)
	{
		if(decodeMode == 0x00)
		{
			max7219_SendData(position, getSymbol(numeric) | (1 << 7));
		}
		else if(decodeMode == 0xFF)
		{
			max7219_SendData(position, numeric | (1 << 7));
		}
	}
	else
	{
		if(decodeMode == 0x00)
		{
			max7219_SendData(position, getSymbol(numeric) & (~(1 << 7)));
		}
		else if(decodeMode == 0xFF)
		{
			max7219_SendData(position, numeric & (~(1 << 7)));
		}
	}
}

void max7219_WriteChar(MAX7219_Digits position, char ch, bool point)
{
    if (position == 0 || position > NUMBER_OF_DIGITS) {
        return;
    }

    uint8_t seg = max7219_font_from_ascii(ch);

    if (point) {
        seg |= SEG_DP;
    }

    max7219_SendData(position, seg);
}

void max7219_WriteStringRight(const char *s)
{
    // 0~7 전체 범위 안에서 오른쪽 정렬
    max7219_WriteStringInRange(s, 0, NUMBER_OF_DIGITS - 1, true);
}



MAX7219_Digits max7219_PrintItos(MAX7219_Digits position, int value)
{
	max7219_SendData(REG_DECODE_MODE, 0xFF);

	int32_t i;

	if (value < 0)
	{
		if(position > 0)
		{
			max7219_SendData(position, MINUS);
			position--;
		}
		value = -value;
	}

	i = 1;

	while ((value / i) > 9)
	{
		i *= 10;
	}

	if(position > 0)
	{
		max7219_SendData(position, value/i);
		position--;
	}

	i /= 10;

	while (i > 0)
	{
		if(position > 0)
		{
			max7219_SendData(position, (value % (i * 10)) / i);
			position--;
		}

		i /= 10;
	}

	max7219_SendData(REG_DECODE_MODE, decodeMode);

	return position;
}

MAX7219_Digits max7219_PrintNtos(MAX7219_Digits position, uint32_t value, uint8_t n)
{
	max7219_SendData(REG_DECODE_MODE, 0xFF);

	if (n > 0u)
	{
		uint32_t i = lcdPow10(n - 1u);

		while (i > 0u)	/* Display at least one symbol */
		{
			if(position > 0u)
			{
				max7219_SendData(position, (value / i) % 10u);
				position--;
			}

			i /= 10u;
		}
	}

	max7219_SendData(REG_DECODE_MODE, decodeMode);

	return position;
}

MAX7219_Digits max7219_PrintFtos(MAX7219_Digits position, float value, uint8_t n)
{
	if(n > 4)
	{
		n = 4;
	}

	max7219_SendData(REG_DECODE_MODE, 0xFF);

	if (value < 0.0)
	{
		if(position > 0)
		{
			max7219_SendData(position, MINUS);
			position--;
		}

		value = -value;
	}

	position = max7219_PrintItos(position, (int32_t) value);

	if (n > 0u)
	{
		max7219_PrintDigit(position + 1, ((int32_t) value) % 10, true);

		position = max7219_PrintNtos(position, (uint32_t) (value * (float) lcdPow10(n)), n);
	}

	max7219_SendData(REG_DECODE_MODE, decodeMode);

	return position;
}

static uint16_t getSymbol(uint8_t number)
{
	return SYMBOLS[number];
}

static uint32_t lcdPow10(uint8_t n)
{
	uint32_t retval = 1u;

	while (n > 0u)
	{
		retval *= 10u;
		n--;
	}

	return retval;
}





void max7219_WriteStringInRange(const char *s,
                                uint8_t start,
                                uint8_t end,
                                bool align_right)
{
    if (s == NULL) {
        return;
    }

    // 범위 정규화
    if (start > end) {
        uint8_t tmp = start;
        start = end;
        end = tmp;
    }

    if (end >= NUMBER_OF_DIGITS) {
        end = NUMBER_OF_DIGITS - 1;
    }

    uint8_t width = end - start + 1u;
    size_t len = strlen(s);

    if (align_right) {
        // ---- 오른쪽 정렬: 문자열 오른쪽 끝이 end에 붙음 ----
        size_t visible = (len < width) ? len : width;
        uint8_t first_pos  = (uint8_t)(end + 1u - visible); // 출력 시작 슬롯
        size_t first_char  = len - visible;                // 문자열에서 첫 글자 인덱스

        // 왼쪽 패딩 공백
        for (uint8_t pos = start; pos < first_pos; ++pos) {
            max7219_WriteCharAt(pos, ' ', false);
        }

        // 실제 텍스트
        uint8_t pos = first_pos;
        for (size_t i = first_char; i < len && pos <= end; ++i, ++pos) {
            max7219_WriteCharAt(pos, s[i], false);
        }
    } else {
        // ---- 왼쪽 정렬: 문자열 왼쪽이 start에 붙음 ----
        size_t visible = (len < width) ? len : width;
        uint8_t pos = start;
        size_t i = 0;

        for (; i < visible; ++i, ++pos) {
            max7219_WriteCharAt(pos, s[i], false);
        }

        // 나머지 오른쪽 공백
        for (; pos <= end; ++pos) {
            max7219_WriteCharAt(pos, ' ', false);
        }
    }
}


// 왼쪽 4자리: 슬롯 0~3
void max7219_WriteLeft4(const char *s, bool align_right)
{
    max7219_WriteStringInRange(s, 0, 3, align_right);
}

// 오른쪽 4자리: 슬롯 4~7
void max7219_WriteRight4(const char *s, bool align_right)
{
    max7219_WriteStringInRange(s, 4, 7, align_right);
}


void max7219_ScrollText(const char *text,
                        uint8_t start,
                        uint8_t end,
                        MAX7219_ScrollDir dir,
                        uint32_t step_delay_ms,
                        uint32_t hold_ms)
{
    if (text == NULL) {
        return;
    }

    if (start > end) {
        uint8_t tmp = start;
        start = end;
        end = tmp;
    }

    if (end >= NUMBER_OF_DIGITS) {
        end = NUMBER_OF_DIGITS - 1;
    }

    uint8_t width = end - start + 1u;
    size_t len = strlen(text);
    if (len == 0 || width == 0) {
        return;
    }

    if (dir == MAX7219_SCROLL_LEFT) {
        // 일반적인 "왼쪽으로 흘러가기" (새 글자 오른쪽에서 등장)
        for (int offset = -(int)width; offset <= (int)len; ++offset) {
            for (uint8_t pos = 0; pos < width; ++pos) {
                int idx = offset + (int)pos;
                char ch = (idx < 0 || idx >= (int)len) ? ' ' : text[idx];
                max7219_WriteCharAt(start + pos, ch, false);
            }

            // 텍스트가 윈도우에 딱 들어왔을 때 잠깐 멈춤
            if (hold_ms > 0 && offset == 0) {
                HAL_Delay(hold_ms);
            }

            HAL_Delay(step_delay_ms);
        }
    } else {
        // 오른쪽으로 흘러가기 (새 글자 왼쪽에서 등장)
        for (int offset = (int)len; offset >= -(int)width; --offset) {
            for (uint8_t pos = 0; pos < width; ++pos) {
                int idx = offset - 1 - (int)pos;
                char ch = (idx < 0 || idx >= (int)len) ? ' ' : text[idx];
                max7219_WriteCharAt(start + pos, ch, false);
            }

            if (hold_ms > 0 && offset == (int)len) {
                HAL_Delay(hold_ms);
            }

            HAL_Delay(step_delay_ms);
        }
    }
}



// main.c 상단 쪽에 추가 (USER CODE BEGIN PFP 근처에 놔둬도 됨)
 void MAX7219_RunAllTests(void)
{
    char buf[16];

    // 1) 기본 문자열 오른쪽 정렬 테스트
    max7219_Clean();

    max7219_WriteStringRight("HELP");
    HAL_Delay(1000);

    max7219_WriteStringRight("SPd");
    HAL_Delay(1000);

    max7219_WriteStringRight("ALt");
    HAL_Delay(1000);

    snprintf(buf, sizeof(buf), "%4d", 123);
    max7219_WriteStringRight(buf);
    HAL_Delay(1000);

    // 2) 4+4 분할 테스트: 왼쪽은 모드, 오른쪽은 값
    max7219_Clean();

    max7219_WriteLeft4("SPD", true);   // 왼쪽 4칸에서 'SPD' 오른쪽 정렬
    max7219_WriteRight4("0123", true); // 오른쪽 4칸에서 '0123' 오른쪽 정렬
    HAL_Delay(1500);

    // 3) 오른쪽 4자리만 업데이트
    max7219_WriteRight4("9999", true);
    HAL_Delay(1500);

    // 4) 개별 자리 제어 테스트
    max7219_Clean();

    // pos: 0(왼쪽) ~ 7(오른쪽)
    max7219_WriteCharAt(0, 'A', false); // 가장 왼쪽
    max7219_WriteCharAt(1, 'B', false);
    max7219_WriteCharAt(2, 'C', false);
    max7219_WriteCharAt(3, 'D', false);
    max7219_WriteCharAt(4, '1', true);  // 가운데 근처에 '1.' (소수점)
    max7219_WriteCharAt(5, '2', false);
    max7219_WriteCharAt(6, '3', false);
    max7219_WriteCharAt(7, '4', false); // 가장 오른쪽
    HAL_Delay(2000);

    // 5) 전체 8자리 스크롤 테스트 (왼쪽으로 흐르기)
    max7219_Clean();
    max7219_ScrollText("HELLO", 0, 7,
                       MAX7219_SCROLL_LEFT,
                       120,   // step 딜레이(ms)
                       400);  // 중앙에 딱 들어왔을 때 hold(ms)

    HAL_Delay(500);

    // 6) 전체 8자리 스크롤 (오른쪽으로 흐르기)
    max7219_Clean();
    max7219_ScrollText("SPD=123", 0, 7,
                       MAX7219_SCROLL_RIGHT,
                       120,
                       400);

    HAL_Delay(500);

    // 7) 4+4 조합 + 오른쪽 4자리만 스크롤
    max7219_Clean();
    max7219_WriteLeft4("SPD", true); // 왼쪽 4칸에 SPD
    // 오른쪽 4칸에서만 숫자 흐르게
    max7219_ScrollText("0123", 4, 7,
                       MAX7219_SCROLL_LEFT,
                       120,
                       300);

    // 끝나면 마지막 상태 그대로 화면에 남겨둠
}


