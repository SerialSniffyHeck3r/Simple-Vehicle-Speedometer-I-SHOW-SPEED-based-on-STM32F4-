#include "buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;   // CubeMX에서 생성되는 TIM4 핸들

#define BUZZER_TIMER          htim4
#define BUZZER_TIMER_CHANNEL  TIM_CHANNEL_1
#define BUZZER_TIMER_CLOCK_HZ 100000u   // PSC=999 기준 100MHz / 1000

typedef struct {
    uint16_t freq_hz;      // 0이면 쉼표
    uint16_t duration_ms;  // note 길이
} buzzer_note_t;

typedef struct {
    const buzzer_note_t *notes;
    uint8_t              length;
} buzzer_sequence_t;

typedef struct {
    const buzzer_sequence_t *seq;
    uint8_t   index;
    uint16_t  ticks_left;   // 100ms tick 단위
    uint8_t   active;
    uint8_t   priority;     // ★ 추가: 현재 재생 중인 시퀀스 우선순위
} buzzer_state_t;

static buzzer_state_t s_buzzer;
static uint8_t s_buzzer_volume = 4u;  // 0~4 (0=mute, 4=max)

// ★ 새로 추가: 다음 tick에서 시작할 pending 시퀀스
//  -1이면 "예약 없음"
static volatile int8_t s_pending_seq_id = -1;



// 볼륨 → duty 스케일 (강한 로그 느낌)
// 인덱스: 0,1,2,3,4  →  0, 1, 2, 4, 8
// 최종 duty = period * scale / 16  → 최대 duty는 50%
static const uint8_t s_volume_scale[5] = {
    0u,  // 0: mute
    1u,  // 1: 아주 작게
    2u,  // 2: 작게
    4u,  // 3: 중간
    8u   // 4: 최대
};


// ----- 음계 정의 (대략 도레미파솔라시도) -----
// ----- 음계 정의: 2옥타브 ~ 7옥타브 (12-TET, A4=440Hz) -----
#define NOTE_REST   0u

// 2옥타브
#define NOTE_C2     65u
#define NOTE_CS2    69u
#define NOTE_D2     73u
#define NOTE_DS2    78u
#define NOTE_E2     82u
#define NOTE_F2     87u
#define NOTE_FS2    92u
#define NOTE_G2     98u
#define NOTE_GS2   104u
#define NOTE_A2    110u
#define NOTE_AS2   117u
#define NOTE_B2    123u

// 3옥타브
#define NOTE_C3    131u
#define NOTE_CS3   139u
#define NOTE_D3    147u
#define NOTE_DS3   156u
#define NOTE_E3    165u
#define NOTE_F3    175u
#define NOTE_FS3   185u
#define NOTE_G3    196u
#define NOTE_GS3   208u
#define NOTE_A3    220u
#define NOTE_AS3   233u
#define NOTE_B3    247u

// 4옥타브
#define NOTE_C4    262u
#define NOTE_CS4   277u
#define NOTE_D4    294u
#define NOTE_DS4   311u
#define NOTE_E4    330u
#define NOTE_F4    349u
#define NOTE_FS4   370u
#define NOTE_G4    392u
#define NOTE_GS4   415u
#define NOTE_A4    440u
#define NOTE_AS4   466u
#define NOTE_B4    494u

// 5옥타브
#define NOTE_C5    523u
#define NOTE_CS5   554u
#define NOTE_D5    587u
#define NOTE_DS5   622u
#define NOTE_E5    659u
#define NOTE_F5    698u
#define NOTE_FS5   740u
#define NOTE_G5    784u
#define NOTE_GS5   831u
#define NOTE_A5    880u
#define NOTE_AS5   932u
#define NOTE_B5    988u

// 6옥타브
#define NOTE_C6   1047u
#define NOTE_CS6  1109u
#define NOTE_D6   1175u
#define NOTE_DS6  1245u
#define NOTE_E6   1319u
#define NOTE_F6   1397u
#define NOTE_FS6  1480u
#define NOTE_G6   1568u
#define NOTE_GS6  1661u
#define NOTE_A6   1760u
#define NOTE_AS6  1865u
#define NOTE_B6   1976u

// 7옥타브
#define NOTE_C7   2093u
#define NOTE_CS7  2217u
#define NOTE_D7   2349u
#define NOTE_DS7  2489u
#define NOTE_E7   2637u
#define NOTE_F7   2794u
#define NOTE_FS7  2960u
#define NOTE_G7   3136u
#define NOTE_GS7  3322u
#define NOTE_A7   3520u
#define NOTE_AS7  3729u
#define NOTE_B7   3951u


// ----- 시퀀스 정의 -----

// Start: 도 - 미 - 솔 - 높은 도
static const buzzer_note_t SEQ_START[] = {
    { NOTE_AS3, 200 }, { NOTE_D4, 200 },
    { NOTE_F4, 200 }, { NOTE_AS4, 600 },
};

// Sat OK: 솔 - 도(높은)
static const buzzer_note_t SEQ_SAT_OK[] = {
    { NOTE_F4, 150 }, { NOTE_AS4, 250 },
};

// Warning 1: 낮은 경고 한 번
static const buzzer_note_t SEQ_WARN1[] = {
    { NOTE_G4, 300 },
};

// Warning 2: 삐-삐
static const buzzer_note_t SEQ_WARN2[] = {
    { NOTE_A5, 200 }, { NOTE_REST, 100 }, { NOTE_A5, 200 },
};

// Warning 3: 삐삐삐 급박
static const buzzer_note_t SEQ_WARN3[] = {
    { NOTE_G5, 180 }, { NOTE_REST, 80 },
    { NOTE_G5, 180 }, { NOTE_REST, 80 },
    { NOTE_G5, 220 },
};

// User 계열은 간단한 단음으로만 정의
static const buzzer_note_t SEQ_USER1[] = { { NOTE_C5, 150 } };
static const buzzer_note_t SEQ_USER2[] = { { NOTE_D5, 150 } };
static const buzzer_note_t SEQ_USER3[] = { { NOTE_E5, 150 } };
static const buzzer_note_t SEQ_USER4[] = { { NOTE_F5, 150 } };
static const buzzer_note_t SEQ_USER5[] = { { NOTE_G5, 150 } };
static const buzzer_note_t SEQ_USER6[] = { { NOTE_A5, 150 } };
static const buzzer_note_t SEQ_USER7[] = { { NOTE_B5, 150 } };
static const buzzer_note_t SEQ_USER8[] = { { NOTE_AS6, 50 } };
static const buzzer_note_t SEQ_USER9[] = {
    { NOTE_G5, 75 }, { NOTE_D6, 125 },
};

static const buzzer_sequence_t s_sequences[BEEP_SEQ_COUNT] = {
    [BEEP_SEQ_START]    = { SEQ_START,   sizeof(SEQ_START)   / sizeof(SEQ_START[0]) },
    [BEEP_SEQ_SAT_OK]   = { SEQ_SAT_OK,  sizeof(SEQ_SAT_OK)  / sizeof(SEQ_SAT_OK[0]) },
    [BEEP_SEQ_WARNING1] = { SEQ_WARN1,   sizeof(SEQ_WARN1)   / sizeof(SEQ_WARN1[0]) },
    [BEEP_SEQ_WARNING2] = { SEQ_WARN2,   sizeof(SEQ_WARN2)   / sizeof(SEQ_WARN2[0]) },
    [BEEP_SEQ_WARNING3] = { SEQ_WARN3,   sizeof(SEQ_WARN3)   / sizeof(SEQ_WARN3[0]) },
    [BEEP_SEQ_USER1]    = { SEQ_USER1,   sizeof(SEQ_USER1)   / sizeof(SEQ_USER1[0]) },
    [BEEP_SEQ_USER2]    = { SEQ_USER2,   sizeof(SEQ_USER2)   / sizeof(SEQ_USER2[0]) },

    // ★ 여기 원래 코드가 잘못 돼 있었을 가능성 큼
    [BEEP_SEQ_USER3]    = { SEQ_USER3,   sizeof(SEQ_USER3)   / sizeof(SEQ_USER3[0]) },

    [BEEP_SEQ_USER4]    = { SEQ_USER4,   sizeof(SEQ_USER4)   / sizeof(SEQ_USER4[0]) },
    [BEEP_SEQ_USER5]    = { SEQ_USER5,   sizeof(SEQ_USER5)   / sizeof(SEQ_USER5[0]) },
    [BEEP_SEQ_USER6]    = { SEQ_USER6,   sizeof(SEQ_USER6)   / sizeof(SEQ_USER6[0]) },
    [BEEP_SEQ_USER7]    = { SEQ_USER7,   sizeof(SEQ_USER7)   / sizeof(SEQ_USER7[0]) },
    [BEEP_SEQ_USER8]    = { SEQ_USER8,   sizeof(SEQ_USER8)   / sizeof(SEQ_USER8[0]) },
    [BEEP_SEQ_USER9]    = { SEQ_USER9,   sizeof(SEQ_USER9)   / sizeof(SEQ_USER9[0]) },
};


// 우선순위: 0 = 가장 낮음, 숫자 클수록 더 중요
static const uint8_t s_seq_priority[BEEP_SEQ_COUNT] =
{
    [BEEP_SEQ_START]    = 2,  // 부팅 멜로디
    [BEEP_SEQ_SAT_OK]   = 0,  // SAT OK는 그냥 알림
    [BEEP_SEQ_WARNING1] = 2,  // 속도 경고는 중요
    [BEEP_SEQ_WARNING2] = 2,
    [BEEP_SEQ_WARNING3] = 2,
    [BEEP_SEQ_USER1]    = 0,
    [BEEP_SEQ_USER2]    = 0,  // 0→100 카운트다운
    [BEEP_SEQ_USER3]    = 0,
    [BEEP_SEQ_USER4]    = 0,
    [BEEP_SEQ_USER5]    = 0,
    [BEEP_SEQ_USER6]    = 0,
    [BEEP_SEQ_USER7]    = 0,
    [BEEP_SEQ_USER8]    = 1,  // GO! 같은 강조 비프
    [BEEP_SEQ_USER9]    = 1,
};


// ----- 내부 함수 -----

// 수정 후
static void buzzer_set_frequency(uint16_t freq_hz)
{
    // freq_hz == 0 이거나 볼륨이 0이면 완전 mute
    if (freq_hz == 0u || s_buzzer_volume == 0u) {
        __HAL_TIM_SET_COMPARE(&BUZZER_TIMER, BUZZER_TIMER_CHANNEL, 0u);
        return;
    }

    uint32_t period = BUZZER_TIMER_CLOCK_HZ / freq_hz;
    if (period < 2u) {
        period = 2u;
    }
    if (period > 0xFFFFu) {
        period = 0xFFFFu;
    }

    __HAL_TIM_SET_AUTORELOAD(&BUZZER_TIMER, (uint16_t)(period - 1u));

    // ----- 여기부터 로그 스케일 duty 계산 -----
    uint8_t vol = s_buzzer_volume;
    if (vol > 4u) {
        vol = 4u;
    }

    // 최대 duty = period * 8 / 16 = period / 2 (50%)
    uint32_t duty = (period * (uint32_t)s_volume_scale[vol]) >> 4;  // /16

    // 볼륨이 1 이상인데 계산상 0 나오면 최소 1로
    if (duty == 0u && vol > 0u) {
        duty = 1u;
    }

    __HAL_TIM_SET_COMPARE(&BUZZER_TIMER, BUZZER_TIMER_CHANNEL, (uint16_t)duty);
}

// ----- 외부 API -----

// ----- 외부 API -----

void Buzzer_SetVolume(uint8_t volume_0_to_4)
{
    if (volume_0_to_4 > 4u) {
        volume_0_to_4 = 4u;
    }
    s_buzzer_volume = volume_0_to_4;
}

void Buzzer_Init(void)
{
    memset(&s_buzzer, 0, sizeof(s_buzzer));
    s_pending_seq_id = -1;   // 예약 없도록 명시

    buzzer_set_frequency(0u);
    HAL_TIM_PWM_Start(&BUZZER_TIMER, BUZZER_TIMER_CHANNEL);
}


// 이전 Buzzer_PlaySequence 몸통에서 뽑아낸 내부 함수
static void buzzer_start_sequence_internal(beep_sequence_id_t seq_id)
{
    if (seq_id >= BEEP_SEQ_COUNT) {
        return;
    }

    const buzzer_sequence_t *seq = &s_sequences[seq_id];
    s_buzzer.seq   = seq;
    s_buzzer.index = 0u;

    if (!seq || seq->length == 0u) {
        s_buzzer.ticks_left = 0u;
        s_buzzer.active     = 0u;
        buzzer_set_frequency(0u);
        return;
    }

    const buzzer_note_t *note = &seq->notes[0];
    uint16_t ms    = note->duration_ms;
    uint16_t ticks = (uint16_t)((ms + 99u) / 100u);  // 100ms 단위 반올림
    if (ticks == 0u) {
        ticks = 1u;
    }

    s_buzzer.ticks_left = ticks;
    s_buzzer.active     = 1u;
    buzzer_set_frequency(note->freq_hz);
}


void Buzzer_PlaySequence(beep_sequence_id_t seq_id)
{
    if (seq_id >= BEEP_SEQ_COUNT) {
        return;
    }

    // 다음 TIM3 100ms tick에서 시작하도록 예약만 한다.
    __disable_irq();
    s_pending_seq_id = (int8_t)seq_id;
    __enable_irq();
}



void Buzzer_Tick_100ms(void)
{
    // 1) 새 시퀀스 예약이 있으면 여기서만 실제 시작
    if (s_pending_seq_id >= 0) {
        beep_sequence_id_t seq_id = (beep_sequence_id_t)s_pending_seq_id;
        s_pending_seq_id = -1;

        buzzer_start_sequence_internal(seq_id);

        // 방금 시작한 tick에서는 duration을 깎지 않는다.
        // → 다음 tick부터 100ms씩 깎게 하려고 여기서 return.
        return;
    }

    // 2) 재생 중이 아니면 아무것도 안 함
    if (!s_buzzer.active || !s_buzzer.seq) {
        return;
    }

    // 3) 남은 tick 감소 및 다음 note로 진행
    if (s_buzzer.ticks_left > 0u) {
        s_buzzer.ticks_left--;
        if (s_buzzer.ticks_left > 0u) {
            return;
        }
    }

    // 다음 note로 이동
    s_buzzer.index++;
    if (s_buzzer.index >= s_buzzer.seq->length) {
        // 시퀀스 종료
        s_buzzer.active = 0u;
        buzzer_set_frequency(0u);
        return;
    }

    const buzzer_note_t *note = &s_buzzer.seq->notes[s_buzzer.index];
    uint16_t ms    = note->duration_ms;
    uint16_t ticks = (uint16_t)((ms + 99u) / 100u);
    if (ticks == 0u) ticks = 1u;

    s_buzzer.ticks_left = ticks;
    buzzer_set_frequency(note->freq_hz);
}



bool Buzzer_IsPlaying(void)
{
    return (s_buzzer.active != 0u);
}
