/**PWM控制**/
/**PWM控制:注意，此文件中要包含<stm32f10x.tim.h>**/
/**PWM控制:注意，此文件中要包含<stm32f10x.tim.h>,或者在STM32F10X.CONF.H中放开#include "stm32f10x_tim.h" **/
#include "stm32f10x.h"
#include "AGV_pwm.h"    
#include "stm32f10x_tim.h"



/**前左电机模式定义**/
#define   MotorQZ_IN1_Set     GPIO_SetBits(GPIOA, GPIO_Pin_6)
#define   MotorQZ_IN1_Reset   GPIO_ResetBits(GPIOA, GPIO_Pin_6)
#define   MotorQZ_IN2_Set     GPIO_SetBits(GPIOA, GPIO_Pin_7)
#define   MotorQZ_IN2_Reset   GPIO_ResetBits(GPIOA, GPIO_Pin_7)			

/**前右电机模式定义**/
#define   MotorQY_IN1_Set     GPIO_SetBits(GPIOB, GPIO_Pin_0)
#define   MotorQY_IN1_Reset   GPIO_ResetBits(GPIOB, GPIO_Pin_0)
#define   MotorQY_IN2_Set     GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define   MotorQY_IN2_Reset   GPIO_ResetBits(GPIOB, GPIO_Pin_1)	

/**后左电机模式定义**/
#define   MotorHZ_IN1_Set     GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define   MotorHZ_IN1_Reset   GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define   MotorHZ_IN2_Set     GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define   MotorHZ_IN2_Reset   GPIO_ResetBits(GPIOC, GPIO_Pin_13)		

/**后右电机模式定义**/
#define   MotorHY_IN1_Set     GPIO_SetBits(GPIOC, GPIO_Pin_14)
#define   MotorHY_IN1_Reset   GPIO_ResetBits(GPIOC, GPIO_Pin_14)
#define   MotorHY_IN2_Set     GPIO_SetBits(GPIOC, GPIO_Pin_15)
#define   MotorHY_IN2_Reset   GPIO_ResetBits(GPIOC, GPIO_Pin_15)	


void PWM_RCC_Configuration(void)//SYSTEN CLOCK INITIALIZATION
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE); 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE); 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 

}


/***************  配置PWM电机控制用到的I/O口 *******************/
/**TIM2的4路Channel_PA0左前轮转速__PA1右前轮转速__PA2左后轮转速__PA3右后轮转速**/
/**TIM3的4路Channel__PA6备用__PA7备用__PB0备用__PB1备用_**/
void PWM_GPIO_Configuration(void)	//PORT INITIALIZATION
{
	GPIO_InitTypeDef    GPIO_InitStructure;
	 	
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

/**TIM2选用没有重影像方式**/
void TIM2_Configuration(void)//TIMER INITIALIZATION
{	
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure; 
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
//	GPIO_PinRemapConfig(_,_);//没有重映像
//	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	
	TIM_TimeBaseStructure.TIM_Period = 100-1; 
	TIM_TimeBaseStructure.TIM_Prescaler =36-1;   
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);   
//	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);//定时器中断

	//PWM初始化
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);	
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);//使能预装载寄存器
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);	
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);//使能预装载寄存器
	TIM_OC3Init(TIM2,&TIM_OCInitStructure);	
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);//使能预装载寄存器
	TIM_OC4Init(TIM2,&TIM_OCInitStructure);	
	TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);//使能预装载寄存器
	
	TIM_Cmd(TIM2,ENABLE);
}



// /**TIM3选用没有重影像方式**/
// void TIM3_Configuration(void)//TIMER INITIALIZATION
// {	
// 	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure; 
// 	TIM_OCInitTypeDef TIM_OCInitStructure;
// 	
// //	GPIO_PinRemapConfig(_,_);//没有重映像
// //	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
// 	
// 	TIM_TimeBaseStructure.TIM_Period = 100-1; 
// 	TIM_TimeBaseStructure.TIM_Prescaler =36-1;   
// 	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
// 	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
// 	
// 	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);   
// //	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);//定时器中断

// 	//PWM初始化
// 	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
// 	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
// 	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
// 	
// 	TIM_OC1Init(TIM3,&TIM_OCInitStructure);	
// 	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);//使能预装载寄存器
// 	TIM_OC2Init(TIM3,&TIM_OCInitStructure);	
// 	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);//使能预装载寄存器
// 	TIM_OC3Init(TIM3,&TIM_OCInitStructure);	
// 	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);//使能预装载寄存器
// 	TIM_OC3Init(TIM3,&TIM_OCInitStructure);	
// 	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);//使能预装载寄存器
// 	
// 	TIM_Cmd(TIM3,ENABLE);
// }



//电机控制函数，控制输出PWM的占空比。
void motor_control(void)  //初始化电机控制
{
		TIM_SetCompare1(TIM2, 0);
		TIM_SetCompare2(TIM2, 0);
		TIM_SetCompare3(TIM2, 0);
		TIM_SetCompare4(TIM2, 0);
// 		TIM_SetCompare1(TIM3, 20);
// 		TIM_SetCompare2(TIM3, 40);
// 		TIM_SetCompare3(TIM3, 60);
// 		TIM_SetCompare4(TIM3, 80);
	
}
//前左电机控制
void motorQZ_control(TIM_TypeDef* TMM , uint16_t PWMPulse, u8 Channel, u8 Cmd)
{
		//PWM控制
		if(Channel==1)
			TIM_SetCompare1(TMM, PWMPulse);
		else if(Channel==2)
			TIM_SetCompare2(TMM, PWMPulse);
		else if(Channel==3)
			TIM_SetCompare3(TMM, PWMPulse);
		else if(Channel==4)
			TIM_SetCompare4(TMM, PWMPulse);
		
		//模式控制
				if(Cmd==1)  //模式0，停止
		{
			MotorQZ_IN1_Reset;
			MotorQZ_IN2_Reset;
		}
				if(Cmd==2)  //模式1，正转
		{
			MotorQZ_IN1_Set;
			MotorQZ_IN2_Reset;
		}
				if(Cmd==3)  //模式2，反转
		{
			MotorQZ_IN1_Reset;
			MotorQZ_IN2_Set;
		}
				if(Cmd==4)  //模式3，刹车
		{
			MotorQZ_IN1_Set;
			MotorQZ_IN2_Set;
		}
		
}

//前右电机控制
void motorQY_control(TIM_TypeDef* TMM , uint16_t PWMPulse, u8 Channel, u8 Cmd)
{
		//PWM控制
		if(Channel==1)
			TIM_SetCompare1(TMM, PWMPulse);
		else if(Channel==2)
			TIM_SetCompare2(TMM, PWMPulse);
		else if(Channel==3)
			TIM_SetCompare3(TMM, PWMPulse);
		else if(Channel==4)
			TIM_SetCompare4(TMM, PWMPulse);
		
		//模式控制
				if(Cmd==1)  //模式0，停止
		{
			MotorQY_IN1_Reset;
			MotorQY_IN2_Reset;
		}
				if(Cmd==2)  //模式1，正转
		{
			MotorQY_IN1_Set;
			MotorQY_IN2_Reset;
		}
				if(Cmd==3)  //模式2，反转
		{
			MotorQY_IN1_Reset;
			MotorQY_IN2_Set;
		}
				if(Cmd==4)  //模式3，刹车
		{
			MotorQY_IN1_Set;
			MotorQY_IN2_Set;
		}
		
}

//后左电机控制
void motorHZ_control(TIM_TypeDef* TMM , uint16_t PWMPulse, u8 Channel, u8 Cmd)
{
		//PWM控制
		if(Channel==1)
			TIM_SetCompare1(TMM, PWMPulse);
		else if(Channel==2)
			TIM_SetCompare2(TMM, PWMPulse);
		else if(Channel==3)
			TIM_SetCompare3(TMM, PWMPulse);
		else if(Channel==4)
			TIM_SetCompare4(TMM, PWMPulse);
		
		//模式控制
				if(Cmd==1)  //模式0，停止
		{
			MotorHZ_IN1_Reset;
			MotorHZ_IN2_Reset;
		}
				if(Cmd==2)  //模式1，正转
		{
			MotorHZ_IN1_Set;
			MotorHZ_IN2_Reset;
		}
				if(Cmd==3)  //模式2，反转
		{
			MotorHZ_IN1_Reset;
			MotorHZ_IN2_Set;
		}
				if(Cmd==4)  //模式3，刹车
		{
			MotorHZ_IN1_Set;
			MotorHZ_IN2_Set;
		}
		
}

//后右电机控制
void motorHY_control(TIM_TypeDef* TMM , uint16_t PWMPulse, u8 Channel, u8 Cmd)
{
		//PWM控制
		if(Channel==1)
			TIM_SetCompare1(TMM, PWMPulse);
		else if(Channel==2)
			TIM_SetCompare2(TMM, PWMPulse);
		else if(Channel==3)
			TIM_SetCompare3(TMM, PWMPulse);
		else if(Channel==4)
			TIM_SetCompare4(TMM, PWMPulse);
		
		//模式控制
				if(Cmd==1)  //模式0，停止
		{
			MotorHY_IN1_Reset;
			MotorHY_IN2_Reset;
		}
				if(Cmd==2)  //模式1，正转
		{
			MotorHY_IN1_Set;
			MotorHY_IN2_Reset;
		}
				if(Cmd==3)  //模式2，反转
		{
			MotorHY_IN1_Reset;
			MotorHY_IN2_Set;
		}
				if(Cmd==4)  //模式3，刹车
		{
			MotorHY_IN1_Set;
			MotorHY_IN2_Set;
		}
		
}






