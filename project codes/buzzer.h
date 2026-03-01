#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BEEP_SEQ_START = 0,
    BEEP_SEQ_SAT_OK,
    BEEP_SEQ_WARNING1,
    BEEP_SEQ_WARNING2,
    BEEP_SEQ_WARNING3,
    BEEP_SEQ_USER1,
    BEEP_SEQ_USER2,
    BEEP_SEQ_USER3,
    BEEP_SEQ_USER4,
    BEEP_SEQ_USER5,
    BEEP_SEQ_USER6,
    BEEP_SEQ_USER7,
    BEEP_SEQ_USER8,
    BEEP_SEQ_USER9,
    BEEP_SEQ_COUNT
} beep_sequence_id_t;

void Buzzer_Init(void);
void Buzzer_Tick_100ms(void);
void Buzzer_PlaySequence(beep_sequence_id_t seq);
bool Buzzer_IsPlaying(void);

void Buzzer_SetVolume(uint8_t volume_0_to_4);

#ifdef __cplusplus
}
#endif

#endif // BUZZER_H
