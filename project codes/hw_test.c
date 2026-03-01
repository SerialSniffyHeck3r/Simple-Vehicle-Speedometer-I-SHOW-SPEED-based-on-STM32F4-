#include "hw_test.h"

#include "main.h"
#include "max7219.h"
#include "buzzer.h"

/*
 * FLASH 테스트는 STM32F411CEU6의 Sector 4 (0x08010000 ~ 0x0801FFFF)를 사용한다고 가정.
 * 이 섹터는 코드/데이터/설정에 사용하지 않도록 링크 스크립트에서 제외해야 한다.
 */
#define HW_FLASH_TEST_SECTOR       FLASH_SECTOR_4
#define HW_FLASH_TEST_BASE         (0x08010000u)
#define HW_FLASH_TEST_BYTES        (16u * 1024u)    /* 이만큼만 반복 테스트 (섹터 전체 erase는 필수) */

#define HW_FLASH_TEST_WORDS        (HW_FLASH_TEST_BYTES / 4u)

/* RAM 테스트는 전용 버퍼를 사용해서 "미사용 영역"을 대신한다.
 * 이 버퍼는 다른 코드에서 접근하지 않으므로, 이 버퍼가 곧 RAM 테스트 영역이다.
 */
#define HW_RAM_TEST_WORDS          (1024u)          /* 4KB 정도 */

static uint32_t s_ram_test_buffer[HW_RAM_TEST_WORDS];

/* 공통 유틸: 8글자 텍스트를 MAX7219에 그대로 찍기 */
static void HW_DisplayText8(const char text[8])
{
    for (uint8_t i = 0u; i < 8u; ++i) {
        char ch = text[i];
        if (ch == '\0') {
            ch = ' ';
        }
        max7219_WriteCharAt(i, ch, false);
    }
}

/* 공통 유틸: "XY   NNN" 형태 퍼센트 표시
 *  - label0,label1: 'F','L' or 'r','A'
 *  - percent: 0~100
 */
static void HW_DisplayPercent2(const char label0,
                               const char label1,
                               uint8_t percent)
{
    if (percent > 100u) {
        percent = 100u;
    }

    max7219_WriteCharAt(0, label0, false);
    max7219_WriteCharAt(1, label1, false);
    max7219_WriteCharAt(2, ' ',  false);
    max7219_WriteCharAt(3, ' ',  false);
    max7219_WriteCharAt(4, ' ',  false);

    char d2 = ' ';
    char d1 = ' ';
    char d0 = '0';

    if (percent == 100u) {
        d2 = '1';
        d1 = '0';
        d0 = '0';
    } else {
        uint8_t t = percent / 10u;
        uint8_t o = percent % 10u;

        if (t > 0u) {
            d1 = (char)('0' + t);
        } else {
            d1 = ' ';
        }
        d0 = (char)('0' + o);
    }

    max7219_WriteCharAt(5, d2, false);
    max7219_WriteCharAt(6, d1, false);
    max7219_WriteCharAt(7, d0, false);
}

/* FLASH 테스트: true = OK, false = NG */
static bool HW_FlashTest_Run(void)
{
    /* 엔트리: "FL tESt " 2초 */
    HW_DisplayText8("FL tESt ");
    HAL_Delay(2000);

    HAL_FLASH_Unlock();

    /* 테스트 패턴들 */
    const uint32_t patterns[] = {
        0x00000000u,
        0xFFFFFFFFu,
        0x55555555u,
        0xAAAAAAAAu
    };
    const uint32_t num_patterns = (uint32_t)(sizeof(patterns) / sizeof(patterns[0]));

    const uint32_t total_words = HW_FLASH_TEST_WORDS;
    const uint32_t total_ops   = num_patterns * total_words;
    uint32_t       done_ops    = 0u;
    uint8_t        last_percent = 255u;

    for (uint32_t pi = 0u; pi < num_patterns; ++pi) {
        uint32_t pat = patterns[pi];

        /* 🔥 패턴마다 섹터 한 번씩 erase */
        FLASH_EraseInitTypeDef erase;
        uint32_t sector_error = 0u;

        erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
        erase.Sector       = HW_FLASH_TEST_SECTOR;
        erase.NbSectors    = 1u;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
            HAL_FLASH_Lock();
            HW_DisplayText8("FLASH ng");
            HAL_Delay(2000);
            return false;
        }

        /* 실제 쓰기/읽기 테스트: "FL   ---" 형태로 진행 표시 */
        for (uint32_t i = 0u; i < total_words; ++i) {
            uint32_t addr = HW_FLASH_TEST_BASE + (i * 4u);

            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, pat) != HAL_OK) {
                HAL_FLASH_Lock();
                HW_DisplayText8("FLASH ng");
                HAL_Delay(2000);
                return false;
            }

            uint32_t read_back = *(__IO uint32_t *)addr;
            if (read_back != pat) {
                HAL_FLASH_Lock();
                HW_DisplayText8("FLASH ng");
                HAL_Delay(2000);
                return false;
            }

            ++done_ops;
            uint8_t percent = (uint8_t)((done_ops * 100u) / total_ops);
            if (percent != last_percent) {
                last_percent = percent;
                HW_DisplayPercent2('F', 'L', percent);
            }
        }
    }

    HAL_FLASH_Lock();

    HW_DisplayText8("good    ");
    HAL_Delay(2000);

    return true;
}


/* RAM 테스트: true = OK, false = NG */
static bool HW_RamTest_Run(void)
{
    /* 엔트리: "rAn tEST" 2초 */
    HW_DisplayText8("rAn tEST");
    HAL_Delay(2000);

    const uint32_t patterns[] = {
        0x00000000u,
        0xFFFFFFFFu,
        0x55555555u,
        0xAAAAAAAAu
    };
    const uint32_t num_patterns = (uint32_t)(sizeof(patterns) / sizeof(patterns[0]));

    const uint32_t total_words = HW_RAM_TEST_WORDS;
    const uint32_t total_ops   = num_patterns * total_words * 2u; /* write + verify */
    uint32_t       done_ops    = 0u;
    uint8_t        last_percent = 255u;

    bool ok = true;

    for (uint32_t pi = 0u; pi < num_patterns; ++pi) {
        uint32_t pat = patterns[pi];

        /* 쓰기 */
        for (uint32_t i = 0u; i < total_words; ++i) {
            s_ram_test_buffer[i] = pat;

            ++done_ops;
            uint8_t percent = (uint8_t)((done_ops * 100u) / total_ops);
            if (percent != last_percent) {
                last_percent = percent;
                HW_DisplayPercent2('r', 'A', percent);
            }
        }

        /* 검증 */
        for (uint32_t i = 0u; i < total_words; ++i) {
            if (s_ram_test_buffer[i] != pat) {
                ok = false;
            }

            ++done_ops;
            uint8_t percent = (uint8_t)((done_ops * 100u) / total_ops);
            if (percent != last_percent) {
                last_percent = percent;
                HW_DisplayPercent2('r', 'A', percent);
            }
        }
    }

    if (ok) {
        HW_DisplayText8("rAn good");
    } else {
        HW_DisplayText8("rAn ng  ");
    }
    HAL_Delay(2000);

    return ok;
}

void HW_RunSelfTest(void)
{
    /* FLASH → RAM 순서로 테스트 (1회 블로킹 실행) */
    (void)HW_FlashTest_Run();
    (void)HW_RamTest_Run();
}
