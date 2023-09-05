#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "struct_typedef.h"
extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);
extern void buzzer_init(void);
extern void buzzer_high(void);
extern void buzzer_mid(void);
extern void buzzer_low(void);
#endif
