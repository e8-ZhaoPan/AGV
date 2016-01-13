#ifndef __AGV_PWM_H__
#define __AGV_PWM_H__

#include "stm32f10x.h"


void PWM_RCC_Configuration(void);
void PWM_GPIO_Configuration(void);
//void EXTI_Configuration(void);
//void NVIC_Configuration(void);
//void EXTI2_IRQHandler(void);
void TIM2_Configuration(void);
// void TIM3_Configuration(void);
void motor_control(void);
//void TIM3_IRQHandler(void);
void motorQZ_control(TIM_TypeDef* TMM , uint16_t PWMPulse, u8 Channel, u8 Cmd);
void motorQY_control(TIM_TypeDef* TMM , uint16_t PWMPulse, u8 Channel, u8 Cmd);
void motorHZ_control(TIM_TypeDef* TMM , uint16_t PWMPulse, u8 Channel, u8 Cmd);
void motorHY_control(TIM_TypeDef* TMM , uint16_t PWMPulse, u8 Channel, u8 Cmd);


#endif
