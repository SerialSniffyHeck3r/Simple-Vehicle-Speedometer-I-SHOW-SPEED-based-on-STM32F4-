/* settings_storage.c */

#include "settings_storage.h"
#include "main.h"
#include <string.h>
#include <stddef.h>

/* 아무 의미 없는 매직 값 (레코드 식별용) */
#define SETTINGS_MAGIC               (0x43504647u)  /* 'G','F','P','C' 같은 느낌 */

/* STM32F411CEU6 (512KB Flash)
 * Sector 0: 0x08000000, 16KB
 * ...
 * Sector 4: 0x08010000, 64KB
 * Sector 5: 0x08020000, 128KB
 * Sector 6: 0x08040000, 128KB
 * Sector 7: 0x08060000, 128KB
 *
 * => 끝부분 3섹터 = 5,6,7 사용 (총 384KB)
 */
#define SETTINGS_FLASH_BASE          (0x08020000u)  /* Sector 5 시작 */
#define SETTINGS_FLASH_END           (0x08080000u)  /* Flash 끝 주소 (exclusive) */

#define SETTINGS_FLASH_SECTOR_FIRST  FLASH_SECTOR_5
#define SETTINGS_FLASH_SECTOR_COUNT  (3u)           /* 5,6,7 섹터 사용 */

typedef struct {
    uint32_t       magic;
    uint32_t       seq;
    app_settings_t payload;
    uint32_t       crc32;
} settings_record_t;

#define SETTINGS_RECORD_SIZE   (sizeof(settings_record_t))

/* CRC32 (Ethernet 폴리노미얼) */
static uint32_t crc32_calc(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint32_t)data[i];
        for (uint32_t bit = 0; bit < 8u; ++bit) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

/* 플래시 영역을 스캔해서
 *  - 마지막으로 유효한 레코드의 seq, 주소
 *  - 다음에 쓸 수 있는 빈 슬롯 주소
 * 를 찾는다. (웨어 레벨링용 로그 스캔)
 */
static bool settings_scan(uint32_t *out_last_seq,
                          uint32_t *out_last_addr,
                          uint32_t *out_next_free_addr)
{
    uint32_t last_seq      = 0u;
    uint32_t last_addr     = 0u;
    bool     have_last     = false;

    uint32_t addr = SETTINGS_FLASH_BASE;
    const uint32_t end = SETTINGS_FLASH_END;

    while ((addr + SETTINGS_RECORD_SIZE) <= end) {
        const settings_record_t *rec = (const settings_record_t *)addr;

        uint32_t magic = rec->magic;

        /* 플래시 erase 상태: 0xFFFFFFFF...
         * 첫 번째 지워진 슬롯을 만나면 그 뒤는 전부 빈 영역이라고 가정
         */
        if (magic == 0xFFFFFFFFu) {
            break;
        }

        if (magic != SETTINGS_MAGIC) {
            addr += SETTINGS_RECORD_SIZE;
            continue;
        }

        uint32_t calc_crc = crc32_calc((const uint8_t *)&rec->magic,
                                       sizeof(settings_record_t) - sizeof(uint32_t));
        if (calc_crc != rec->crc32) {
            addr += SETTINGS_RECORD_SIZE;
            continue;
        }

        if (!have_last || (rec->seq > last_seq)) {
            have_last = true;
            last_seq  = rec->seq;
            last_addr = addr;
        }

        addr += SETTINGS_RECORD_SIZE;
    }

    if (out_last_seq) {
        *out_last_seq = last_seq;
    }
    if (out_last_addr) {
        *out_last_addr = last_addr;
    }

    if (out_next_free_addr) {
        if ((addr + SETTINGS_RECORD_SIZE) <= end) {
            *out_next_free_addr = addr;
        } else {
            *out_next_free_addr = 0u; /* 더 쓸 곳 없음 */
        }
    }

    return have_last;
}

/* 플래시에서 마지막 유효 설정을 읽기 */
bool Settings_Load(app_settings_t *out)
{
    if (!out) {
        return false;
    }

    uint32_t last_seq  = 0u;
    uint32_t last_addr = 0u;
    uint32_t next_addr = 0u;

    (void)next_addr;

    bool have_last = settings_scan(&last_seq, &last_addr, &next_addr);
    if (!have_last) {
        return false;
    }

    const settings_record_t *rec = (const settings_record_t *)last_addr;
    *out = rec->payload;
    return true;
}

/* 현재 설정을 플래시에 1레코드로 append (웨어 레벨링) */
bool Settings_Save(const app_settings_t *cfg)
{
    if (!cfg) {
        return false;
    }

    uint32_t last_seq  = 0u;
    uint32_t last_addr = 0u;
    uint32_t next_addr = 0u;

    (void)last_addr;
    bool have_last = settings_scan(&last_seq, &last_addr, &next_addr);

    uint32_t new_seq = have_last ? (last_seq + 1u) : 1u;

    HAL_FLASH_Unlock();

    /* 더 쓸 수 있는 슬롯이 없으면 3개 섹터 전체를 지우고 처음부터 다시 시작 */
    if (next_addr == 0u) {
        FLASH_EraseInitTypeDef erase;
        uint32_t sector_error = 0u;

        erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
        erase.Sector       = SETTINGS_FLASH_SECTOR_FIRST;
        erase.NbSectors    = SETTINGS_FLASH_SECTOR_COUNT;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }

        next_addr = SETTINGS_FLASH_BASE;
    }

    /* RAM에서 새 레코드 구성 */
    /* RAM에서 새 레코드 구성 */
    settings_record_t rec;
    memset(&rec, 0xFF, sizeof(rec));  /* 플래시에 맞춰 기본 all-1 (필수는 아님) */
    rec.magic   = SETTINGS_MAGIC;
    rec.seq     = new_seq;
    rec.payload = *cfg;
    rec.crc32   = crc32_calc((const uint8_t *)&rec.magic,
                             sizeof(settings_record_t) - sizeof(uint32_t));

    /* 32비트 워드 단위로 프로그래밍 (F4 시리즈) */
    uint32_t *p_word = (uint32_t *)&rec;
    uint32_t addr    = next_addr;
    for (size_t i = 0u; i < (sizeof(settings_record_t) / 4u); ++i) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, p_word[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        addr += 4u;
    }


    HAL_FLASH_Lock();
    return true;
}
