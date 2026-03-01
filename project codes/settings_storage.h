/* settings_storage.h */

#ifndef INC_SETTINGS_STORAGE_H_
#define INC_SETTINGS_STORAGE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int8_t  timezone_hours;  /* GMT offset, 예: +9 */
    uint8_t brightness;      /* 1~3 */
    uint8_t auto_mode;       /* 0=off, 1=on */
    // Beep volume: 0~4 (0=mute, 4=max)
    uint8_t beep_volume;
    uint8_t reserved;        /* 정렬/확장용 */

} app_settings_t;

/* 플래시에서 설정을 읽어온다.
 *  - 성공: true, out에 값 채움
 *  - 실패: false (유효한 레코드 없음, CRC 에러 등)
 */
bool Settings_Load(app_settings_t *out);

/* 현재 설정을 플래시에 저장한다 (웨어 레벨링 적용). */
bool Settings_Save(const app_settings_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* INC_SETTINGS_STORAGE_H_ */
