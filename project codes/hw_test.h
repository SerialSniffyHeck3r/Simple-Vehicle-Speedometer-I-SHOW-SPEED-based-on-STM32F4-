#ifndef INC_HW_TEST_H_
#define INC_HW_TEST_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* dEu TEST: FLASH + RAM 일회성 블로킹 하드웨어 테스트 */
void HW_RunSelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_HW_TEST_H_ */
