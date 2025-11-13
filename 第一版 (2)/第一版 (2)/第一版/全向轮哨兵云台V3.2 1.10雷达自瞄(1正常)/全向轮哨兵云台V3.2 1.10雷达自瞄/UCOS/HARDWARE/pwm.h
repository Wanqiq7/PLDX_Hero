#ifndef __PWM_H__
#define __PWM_H__

void PWM_Configuration(void);

#define PWM1  TIM12->CCR2
#define PWM2  TIM12->CCR1
#define LASER_ON()  GPIO_SetBits(GPIOG, GPIO_Pin_13)
#define LASER_OFF()  GPIO_ResetBits(GPIOG, GPIO_Pin_13)

void PWM_Configuration(void);

#endif
