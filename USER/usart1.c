/******************** 鑫盛电子工作室 ********************
 * 文件名  ：usart1.c
 * 描述    ：将printf函数重定向到USART1。这样就可以用printf函数将单片机的数据
 *           打印到PC上的超级终端或串口调试助手。         
 * 实验平台：MINI STM32开发板 基于STM32F103C8T6
 * 硬件连接：------------------------
 *          | PA9  - USART1(Tx)      |
 *          | PA10 - USART1(Rx)      |
 *           ------------------------
 * 库版本  ：ST3.0.0  *
 * 淘宝店：http://shop66177872.taobao.com
**********************************************************************************/

#include "usart1.h"
#include <stdarg.h>
#include "led.h"
extern unsigned char GetData ;
extern u8 USFlag;

void USART1_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;

		/* 使能 USART1 时钟*/
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE); 

		/* USART1 使用IO端口配置 */    
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);    
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入
		GPIO_Init(GPIOA, &GPIO_InitStructure);   //初始化GPIOA
			
		/* USART1 工作模式配置 */
		//USART_InitStructure.USART_BaudRate = 9600;	//波特率设置：9600
		USART_InitStructure.USART_BaudRate = 115200;	//波特率设置：115200
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//数据位数设置：8位
		USART_InitStructure.USART_StopBits = USART_StopBits_1; 	//停止位设置：1位
		USART_InitStructure.USART_Parity = USART_Parity_No ;  //是否奇偶校验：无
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制模式设置：没有使能
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//接收与发送都使能
		USART_Init(USART1, &USART_InitStructure);  //初始化USART1
		USART_Cmd(USART1, ENABLE);// USART1使能
}


 /* 描述  ：重定向c库函数printf到USART1*/ 
int fputc(int ch, FILE *f)
{
/* 将Printf内容发往串口 */
  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
 
  return (ch);
}

unsigned char UART1GetByte(void)
{   	   
        if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
        {  GetData = USART_ReceiveData(USART1); 
					 return 1;//收到数据 
					
		}
		    else 
					 return 0;//没有收到数据 
       
}


//系统中断管理
void NVIC_Configuration(void)
	{ 
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	//设置优先级分组：先占优先级0位,从优先级4位
	
	//设置向量表的位置和偏移
	#ifdef  VECT_TAB_RAM  
		/* Set the Vector Table base location at 0x20000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 		//向量表位于RAM
	#else  /* VECT_TAB_FLASH  */
		/* Set the Vector Table base location at 0x08000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   //向量表位于FLASH
	#endif

	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//USART1中断
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
	}


