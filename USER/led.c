/******************** 鑫盛电子工作室 ********************
 * 文件名  ：led.c
 * 描述    ：led 应用函数库
 *          
 * 实验平台：MINI STM32开发板 基于STM32F103C8T6
 * 硬件连接：-----------------
 *          |   PB15 - LED1   |
 *          |   PB14 - LED2   |
 *          |                 |
 *           ----------------- 
 * 库版本  ：ST3.0.0  																										  
 * 淘宝店：http://shop66177872.taobao.com
*********************************************************/
#include "stm32f10x.h"   
#include "led.h"


 /***************  配置LED用到的I/O口 *******************/
void LED_GPIO_Config(void)	
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); // 使能PB端口时钟  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  //初始化PB端口
  GPIO_SetBits(GPIOB,  GPIO_Pin_8 );	 // 关闭所有LED
}

/***************  配置LED用到的I/O口 *******************/
void ControlDI_AGV_GPIO_Config(void)	
{
		GPIO_InitTypeDef  GPIO_InitStructure;	
		/* Enable the GPIO_LED Clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

		/* Configure the GPIO pin */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 ;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  //下拉输入模式
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);	//输入点
		
		/* Enable the GPIO_LED Clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		/* Configure the GPIO pin */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_14 | GPIO_Pin_4 ;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  //下拉输入模式
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);	//输入点
  
}

/***************  配置LED用到的I/O口 *******************/
void ControlDO_AGV_GPIO_Config(void)	
{
		GPIO_InitTypeDef  GPIO_InitStructure;	
		
		/* Configure the GPIO pin */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_2  ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);	   //控制皮带电机转向
	
			/* Configure the GPIO pin */
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15 ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);	  //控制皮带电机转向
		
		
		/* Configure the GPIO pin */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5 ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);	   //控制电机转向
	
		/* Configure the GPIO pin */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_15 ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);	   //控制电机转向
	
		/* Configure the GPIO pin */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_Init(GPIOC, &GPIO_InitStructure);	   //控制电机转向
		/* Enable the GPIO_ Clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  
		GPIO_SetBits(GPIOB, GPIO_Pin_11| GPIO_Pin_10 |GPIO_Pin_2);    //	皮带电机初始化
		GPIO_SetBits(GPIOA, GPIO_Pin_15);    //	皮带电机初始化
}

void BOOT1_ReleaseToGPIO(void)
{ 
	//注释：BOOT0外部引脚设置为BOOT0=1的情况下，BOOT1即可释放出来做普通GPIO来使用。
	GPIO_InitTypeDef  GPIO_InitStructure;	
		
		/* Configure the GPIO pin */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;  //BOOT1的外部引脚
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);	   //控制皮带电机转向
}
	

/***************  配置用到的PC口 *******************/
void ControlDOC_AGV_GPIO_Config(void)	
{
		GPIO_InitTypeDef  GPIO_InitStructure;	
		/* Configure the GPIO pin */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_Init(GPIOC, &GPIO_InitStructure);	   //控制电机转向
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

}


