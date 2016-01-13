#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"   
#include "stm32f10x_tim.h"


/* the macro definition to trigger the led on or off 
 * 1 - off
 - 0 - on
 */
#define ON  0
#define OFF 1

#define LED1(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_8);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_8)

#define LED2(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_7);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_7)



void LED_GPIO_Config(void);
void ControlDI_AGV_GPIO_Config(void);
void ControlDO_AGV_GPIO_Config(void);
void ControlDOC_AGV_GPIO_Config(void);		
void BOOT1_ReleaseToGPIO(void);					

#endif /* __LED_H */
