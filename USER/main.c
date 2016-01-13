/******************** 鑫盛电子工作室 ********************
 * 文件名  ：main.c
 * 描述    ：串口1(USART1)向电脑的超级终端以1s为间隔打印当前ADC1的转换电压值         
 * 实验平台：MINI STM32开发板 基于STM32F103C8T6
 * 库版本  ：ST3.0.0
 * 淘宝店：http://shop66177872.taobao.com
**********************************************************************************/

#include "stm32f10x.h"
#include "usart1.h"
#include "adc.h" 
#include "led.h"
#include "AGV_pwm.h" 
#include "rc522.h"
#include "stm32f10x_it.h"
#include "Time_test.h"
#include "SysTick.h"

/**定义参数**/

#define  PWMPulseLow       80	
#define  PWMPulseMid1      80
#define  PWMPulseMid2      80
#define  timeeeer1          45
#define  timeeeer2          70
#define  timeeeer3          150
//////////////////////////////////////////////////////////
//M1卡分為16個扇區，每個扇區由4塊（塊0、塊1、塊2、塊3）組成
//我們也將16個扇區的64個塊按絕對地址編號0~63
//第0扇區的塊0（即絕對地址0塊），他用於存放廠商代碼，已經固化，不可更改
//每個扇區的塊0、塊1、塊2為數據塊，可用於存放數據
//每個扇區的塊3為控制塊（絕對地址塊3、7、11....），包括了密碼A，存取控制、密碼B。

/*******************************
PA4、5、6、7：SPI1_NSS、SPI1_SCK、SPI1_MISO、SPI1_MOSI
*连线说明：
*1--SS（sda） <----->PF0  pa2   从机选择  
*2--SCK <----->PB13   时钟输出
*3--MOSI<----->PB15   射频数据输出
*4--MISO<----->PB14   射频模块输入
*5--悬空
*6--GND <----->GND
*7--RST <----->PF1   pa3  休眠控制
*8--VCC <----->VCC   
************************************/
/*全局变量*/
unsigned char CT[20];//卡类型
unsigned char SN[4]; //卡号
unsigned char RFID[16];			//存放RFID 
unsigned char RFIDWrite[16]={0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x07,0x80,0x29,0xff,0xff,0xff,0xff,0xff,0xff};			//RFID writedata 
unsigned char lxl_bit=0;
unsigned char card1_bit=0;
unsigned char card2_bit=0;
unsigned char card3_bit=0;
unsigned char card4_bit=0;
unsigned char total=0;
unsigned char lxl[4]={6,109,250,186};
unsigned char card_1[4]={66,193,88,0};
unsigned char card_2[4]={66,191,104,0};
unsigned char card_3[4]={62,84,28,11};
unsigned char card_4[4]={126,252,248,12};
u8 KEY[6]={0xff,0xff,0xff,0xff,0xff,0xff};
unsigned char RFID1[16]={0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x07,0x80,0x29,0xff,0xff,0xff,0xff,0xff,0xff};
unsigned char senddata1[]={0x41 ,0x54 ,0x2B ,0x43 ,0x49 ,0x50 ,0x53 ,0x54 ,0x41 ,0x52 ,0x54 ,0x3D ,0x22 ,0x54 ,0x43 ,0x50 ,0x22 ,0x2C ,0x22 ,0x31 ,0x30 ,0x2E ,0x30 ,0x2E ,0x31 ,0x2E ,0x35,0X35 ,0x22 ,0x2C ,0x38 ,0x30 ,0x0D ,0x0A };
unsigned char senddata2[]={0x41 ,0x54 ,0x2B ,0x43 ,0x49 ,0x50 ,0x4D ,0x4F ,0x44 ,0x45 ,0x3D ,0x31 ,0x0D ,0x0A };
unsigned char senddata3[]={0x41 ,0x54 ,0x2B ,0x43 ,0x49 ,0x50  ,0x53 ,0x45 ,0x4E ,0x44, 0x0D ,0x0A };
u8 USFlag=0;
volatile u32 T3time=0; // ms 计时变量

/*变量定义区*/
	u8 USART_RX_BUF[64];     //接收缓冲,最大64个字节.
	u8 USART_RX_BUF_LINK[64];     //LinkWifi时候接收缓冲,最大64个字节.
	u8 USART_RX_STA=0;       //接收状态标记
	u8 tempNumInc=0;//接收计数
	uint8_t T1,T2,T3,T4,T5,clp,distance;  //输入DI变量
	u8 counter_BUF=0;
	u8 counter_BUF_LINK=0;
	u8 testt=0;
	u8 espFlag=0;
  u8 Wifi_Flag=0;
	u8 Wifi_Touchuan=0; //透传标志
  u8 WifiStartR=0;
//bool T1,T2,T3,T4,T5,clp;  //输入DI变量
u8 Track,FLAG,testu8; 	 //行动路线
u8 qiaoshuo=0;
u8 zhaopan=0;
u8 timeout=0;//用于接收超时判断
u32 SysTickCountFlag =0;  //SysTick中断计数
u32 SysTickCountFlag_1_0ms =(1000*100);  //SysTick中断1ms计数
u32 SysTickCountFlag_1_5ms =(1500*100);  //SysTick中断1.5ms计数
u32 SysTickCountFlag_2_0ms =(2000*100);  //SysTick中断2ms计数
u32 SysTickCountFlag_Max =100000;  //SysTick中断计数到
u32 HighSysTick =0;  //SysTick 舵机控制：20ms周期，0.5ms~2.5ms高电平占空比-------0度~180度(高电平时钟控制)HighSysTick 取值50~250
u32 LowSysTick = 0;  //SysTick 舵机控制：20ms周期，0.5ms~2.5ms高电平占空比-------0度~180度(低电平时钟控制)LowSysTick = 2000-HighSysTick；
u8  High_Low = 0;   // 在高电平输出时刻还是低电平输出时刻(0-高电平时刻，1-低电平时刻)
//u32 SysTick_TimeSet=100000;     //SysTick的时间基数

u16  PWMPulseHigh=80;
unsigned char GetData;
extern __IO u16 ADC_ConvertedValue;	 
// 软件延时
void Delay(unsigned long time)
{unsigned long i,j;
  
	for(j=0; j<time; j++)
	{
	   for(i=0;i<12000;i++);
	}
}

// 解放PA11和PA12，使得其可以做普通GPIO使用
void USBCAN_GPIO_NoRemap(void)
{
		 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能对应GPIO时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   //使能AFIO重映射时钟
		AFIO->MAPR |= 0x2000;				//进行重映射配置
		AFIO->MAPR &=~0x4000;
}	

//解放PA15和PB3和PB4，使得其可以做普通GPIO使用
void JTAGDisable_GPIO_NoRemap(void)
{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能对应GPIO时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //使能对应GPIO时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   //使能AFIO重映射时钟
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //库函数实现重映射
		
// 		AFIO->MAPR &=~0x01000000;										//直接操作寄存器实现								
// 		AFIO->MAPR |= 0x02000000;
// 		AFIO->MAPR &=~0x04000000;
}
	


//函数声明区域
void LED_GPIO_Config(void);
void ControlDI_AGV_GPIO_Config(void);
void ControlDO_AGV_GPIO_Config(void);
void RFIDReader(void);
void MotoBelt(u16 direction);
void MotoBeltControl(void);//皮带电机通信控制
u8  InfraredDetection(void);
void USART1_IRQHandler(void);
unsigned char UART1GetByte(void);
void NVIC_Configuration(void);
void  Wifi_Connect(void);
void TIM3_NVIC_Configuration(void);
void TIM3_Configuration(void);
void BOOT1_ReleaseToGPIO(void);
void RFID_SN_Control(void);


int main(void)
{ u32 LEDcounter=0,LEDFlag=0;
	
	
	
//  u32 AD_value;	  
//	GPIO_InitTypeDef  GPIO_InitStructurer;	
	
 	USBCAN_GPIO_NoRemap();   // 解放PA11和PA12
 	JTAGDisable_GPIO_NoRemap(); 	//解放PA15和PB3和PB4
	BOOT1_ReleaseToGPIO();

  /* 配置系统时钟为72M */      
  SystemInit();	
  /* 配置串口 */
  USART1_Config();
  /* 初始化ADC1 */
//  ADC1_Init();
	/*初始化状态灯*/
	LED_GPIO_Config();
	/*初始化输入*/
	ControlDI_AGV_GPIO_Config();
	/*初始化输出*/
	ControlDO_AGV_GPIO_Config();
	
	/*初始化PWM控制*/
	PWM_RCC_Configuration();
	PWM_GPIO_Configuration();
	TIM2_Configuration();
	//TIM3_Configuration();
	motor_control();
	//PC口初始化
	ControlDOC_AGV_GPIO_Config();
	NVIC_Configuration();
	/*舵机初始化设置*/
		HighSysTick =150;  //SysTick 舵机控制：20ms周期，0.5ms~2.5ms高电平占空比-------0度~180度(高电平时钟控制)HighSysTick 取值50~250
    LowSysTick =  2000-HighSysTick;  //SysTick 舵机控制：20ms周期，0.5ms~2.5ms高电平占空比-------0度~180度(低电平时钟控制)LowSysTick = 2000-HighSysTick；
    High_Low = 0; 
			/* 配置SysTick 为10us中断一次 */
		SysTick_Init();
	
	GPIO_SetBits(GPIOB, GPIO_Pin_8);//led off
	MotoBelt(0);//0=停止，1=正转，2=反转
	USART_ITConfig(USART1, USART_IT_RXNE , ENABLE);		//USART1接收中断使能
		TIM3_NVIC_Configuration(); /* TIM3(中断优先级) 定时配置 */
    TIM3_Configuration(); 	 /* TIM3(初始化) 定时配置 */
	
//  	printf("Uart init OK            \n");	
	InitRc522();				//初始化射频卡模块
// 	printf("Rc522 init OK           \n");
// 	
//   printf("\r\n --------This is a ADC testing-----\r\n");
//   printf("\r\n ------The ADC Pin is PA0 -----\r\n");
// 	


// // TEST MOTOR CONVEYOR
// while(1)
// {
// // 																		 MotoBelt(1);

// // 																		 Delay(2000);	
// // 																		 MotoBelt(2);
// // 																		 Delay(2000);
// // 																		 MotoBelt(0);
// 	GPIO_ResetBits(GPIOA, GPIO_Pin_15);  //3
// 	Delay(2000);
// 	GPIO_SetBits(GPIOA, GPIO_Pin_15);  //3
// 	Delay(2000);
// }


// // // // // // 	//*************定时器测试*********************
// // // // // // 	START_TIME;	 /* TIM3 开始计时 */
// // // // // // while(1)
// // // // // // {

// // // // // // 		
// // // // // // 	 if ( T3time >= 500 ) /* 500ms 时间到 */
// // // // // //     {
// // // // // // 			STOP_TIME;   /* TIM3 停止计时 */
// // // // // //       T3time = 0;				     
// // // // // // 			printf("\r\n --------This is a Timer testing-----\r\n");
// // // // // // 			START_TIME;	 /* TIM3 开始计时 */
// // // // // //     } 

// // // // // // }
// // // // // // 	 //************定时器测试**********************


// //***********SysTick Testing*********************

// while(1)
// 	{
// 	if(SysTickCountFlag==SysTickCountFlag_Max)
// 	{
// 		GPIO_SetBits(GPIOB,GPIO_Pin_8);
// 	}
// 	
// 	if(SysTickCountFlag==SysTickCountFlag_Max *2)
// 	{
// 		GPIO_ResetBits(GPIOB,GPIO_Pin_8);
// 		SysTickCountFlag=0;
// 		
// 	}
// 	
// 	
// 	}

// //***********SysTick Testing*********************
	
	
	Delay(5000);
	
	while(1)
 { 
		if(espFlag==0) //在未曾连上WIFI时候，才进行WIFI链接操作，链接上后将不再执行此操作。
		{ 
			Wifi_Connect();
		}
	 				
		
    while (espFlag)
  {
 /***************  模拟量采集程序，暂时不用，屏蔽掉。  
// 	   AD_value  = 3300000/4096*ADC_ConvertedValue/1000;
// 	   
// 	   printf("AD value = %d mV  \r\n", AD_value);
// 		Delay(50);
// 		 printf("AD value = %d mV  \r\n", (u32)ADC_ConvertedValue); 
// 		Delay(50);
		*************/
		
		
		UART1GetByte();
		RFIDReader();		

		
/*********以下是AGV Control*************/
	
// 		if(LEDFlag==0)
// 		{
// 			GPIO_SetBits(GPIOB,GPIO_Pin_8);
// 			LEDcounter++;
// 			if(LEDcounter>=150) 
// 			{
// 				LEDcounter=0;
// 				LEDFlag=1;
// 			}
// 		}

// 		
// 		if(LEDFlag==1)
// 		{
// 			GPIO_ResetBits(GPIOB,GPIO_Pin_8);
// 			LEDcounter++;
// 			if(LEDcounter>=150) 
// 			{
// 				LEDcounter=0;
// 				LEDFlag=0;
// 			}
// 		}
// 		
		

		
			MotoBeltControl();
		testu8=InfraredDetection();//红外检测板循迹检测
			
		
// 						 motorQZ_control(TIM2,60,1,2);
// 						 motorQY_control(TIM2,50,2,2);
// 						 motorHZ_control(TIM2,50,3,2);
// 						 motorHY_control(TIM2,60,4,2);
		
			
				
					if(FLAG==0||testu8==0)
					{ //printf("stop all 0(0),Track= %d %d \n",Track,testu8);
						
							 motorQZ_control(TIM2,0,1,1);
							 motorQY_control(TIM2,0,2,1);
							 motorHZ_control(TIM2,0,3,1);
							 motorHY_control(TIM2,0,4,1);
					}
					if(FLAG==1||testu8==1)
				 { //printf("Rollback 1(1) ,Track= %d %d \n",Track,testu8);
							 motorQZ_control(TIM2,PWMPulseHigh,1,3);
						   motorQY_control(TIM2,PWMPulseHigh,2,3);
						   motorHZ_control(TIM2,PWMPulseHigh,3,3);
						   motorHY_control(TIM2,PWMPulseHigh,4,3);
				 }
				
					if(FLAG==2||testu8==2)
					{// printf("Go along(2) ,Track= %d %d \n",Track,testu8);
						 motorQZ_control(TIM2,PWMPulseHigh,1,2);
						 motorQY_control(TIM2,PWMPulseHigh,2,2);
						 motorHZ_control(TIM2,PWMPulseHigh,3,2);
						 motorHY_control(TIM2,PWMPulseHigh,4,2);
					}
						 
						
				if(FLAG==3||testu8==3)
				{ //printf("Turn Left 1(3) ,Track= %d %d \n",Track,testu8);
						motorQZ_control(TIM2,PWMPulseLow,1,2);
						 motorQY_control(TIM2,PWMPulseLow,2,3);
						 motorHZ_control(TIM2,PWMPulseLow,3,2);
						 motorHY_control(TIM2,PWMPulseLow,4,3);	
// 						Delay(timeeeer1);
// 						motorQZ_control(TIM2,0,1,4);
// 						 motorQY_control(TIM2,0,2,4);
// 						 motorHZ_control(TIM2,0,3,4);
// 						 motorHY_control(TIM2,0,4,4);
				}
				if(FLAG==4||testu8==4)
				{	//printf("Turn Left 2(4) ,Track= %d %d \n",Track,testu8);	
						  motorQZ_control(TIM2,PWMPulseMid1,1,2);
						 motorQY_control(TIM2,PWMPulseMid1,2,3);
						 motorHZ_control(TIM2,PWMPulseMid1,3,2);
						 motorHY_control(TIM2,PWMPulseMid1,4,3);
// 					Delay(timeeeer2);
// 						motorQZ_control(TIM2,0,1,4);
// 						 motorQY_control(TIM2,0,2,4);
// 						 motorHZ_control(TIM2,0,3,4);
// 						 motorHY_control(TIM2,0,4,4);
				}
				if(FLAG==5||testu8==5)		 
				{	//printf("Turn Left 3(5) ,Track= %d %d \n",Track,testu8);
						motorQZ_control(TIM2,PWMPulseMid2,1,2);
						 motorQY_control(TIM2,PWMPulseMid2,2,3);
						 motorHZ_control(TIM2,PWMPulseMid2,3,2);
						 motorHY_control(TIM2,PWMPulseMid2,4,3);
// 					Delay(timeeeer3);
// 					motorQZ_control(TIM2,0,1,4);
// 						 motorQY_control(TIM2,0,2,4);
// 						 motorHZ_control(TIM2,0,3,4);
// 						 motorHY_control(TIM2,0,4,4);
				}
				if(FLAG==6||testu8==6)
				{	//printf("Turn Right 1(6) ,Track= %d %d \n",Track,testu8);
							motorQZ_control(TIM2,PWMPulseLow,1,3);
						 motorQY_control(TIM2,PWMPulseLow,2,2);
						 motorHZ_control(TIM2,PWMPulseLow,3,3);
						 motorHY_control(TIM2,PWMPulseLow,4,2);
// 					Delay(timeeeer1);
// 			motorQZ_control(TIM2,0,1,4);
// 						 motorQY_control(TIM2,0,2,4);
// 						 motorHZ_control(TIM2,0,3,4);
// 						 motorHY_control(TIM2,0,4,4);
					
				
				}
				if(FLAG==7||testu8==7)
				{	//printf("Turn Right 2(7) ,Track= %d %d \n",Track,testu8);	
					
					motorQZ_control(TIM2,PWMPulseMid1,1,3);
						 motorQY_control(TIM2,PWMPulseMid1,2,2);
						 motorHZ_control(TIM2,PWMPulseMid1,3,3);
						 motorHY_control(TIM2,PWMPulseMid1,4,2);
// 					Delay(timeeeer2);
// 					motorQZ_control(TIM2,0,1,4);
// 						 motorQY_control(TIM2,0,2,4);
// 						 motorHZ_control(TIM2,0,3,4);
// 						 motorHY_control(TIM2,0,4,4);
						
				}
				if(FLAG==8||testu8==8)
				{	//printf("Turn Right 3(8) ,Track= %d %d \n",Track,testu8); 
						
					 motorQZ_control(TIM2,PWMPulseMid2,1,3);
						 motorQY_control(TIM2,PWMPulseMid2,2,2);
						 motorHZ_control(TIM2,PWMPulseMid2,3,3);
						 motorHY_control(TIM2,PWMPulseMid2,4,2);
// 						Delay(timeeeer3);
// 				motorQZ_control(TIM2,0,1,4);
// 						 motorQY_control(TIM2,0,2,4);
// 						 motorHZ_control(TIM2,0,3,4);
// 						 motorHY_control(TIM2,0,4,4);
					
				}
				if(FLAG==9||testu8==9)
				{//	printf("closely(9) ,Track= %d %d \n",Track,testu8);
						 motorQZ_control(TIM2,0,1,4);
						 motorQY_control(TIM2,0,2,4);
						 motorHZ_control(TIM2,0,3,4);
						 motorHY_control(TIM2,0,4,4);
				}

				if(FLAG==10||testu8==10)
				{ //printf("Estop(10) ,Track= %d %d \n",Track,testu8);
						 motorQZ_control(TIM2,0,1,4);
						 motorQY_control(TIM2,0,2,4);
						 motorHZ_control(TIM2,0,3,4);
						 motorHY_control(TIM2,0,4,4);
				}

				
				
	
/*********以下是AGV Control*************/

}
}
}


//小车皮带正反转控制
void MotoBelt(u16 direction)	//0=停止，1=正转，2=反转
	{
		if(direction==1)  //正转
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_10);  //1
			GPIO_ResetBits(GPIOB, GPIO_Pin_2);   //4	
      GPIO_SetBits(GPIOB, GPIO_Pin_11);    //2	
			GPIO_SetBits(GPIOA, GPIO_Pin_15);    //3			
		}
		if(direction==2)  //反转
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_11);  //2
			GPIO_ResetBits(GPIOA, GPIO_Pin_15);  //3
      GPIO_SetBits(GPIOB, GPIO_Pin_10);    //1	
      GPIO_SetBits(GPIOB, GPIO_Pin_2);     //4				
		}
		if(direction==0)  //停止
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_10);  //1
			GPIO_SetBits(GPIOB, GPIO_Pin_11);  //2	
			GPIO_SetBits(GPIOA, GPIO_Pin_15);  //3
			GPIO_SetBits(GPIOB, GPIO_Pin_2);   //4		
		}
	
	}
	
		
void MotoBeltControl(void)//皮带电机通信控制
{
	if(USART_RX_STA==2)//接收正确
			{			
				
				GPIO_ResetBits(GPIOB, GPIO_Pin_8);//LED灭
				MotoBelt(1);
				printf("\n Encompass21  \n");
				USART_RX_STA=0;

			}		
		else if(USART_RX_STA==3)
			{  
				MotoBelt(2);
				printf("\n Encompass22  \n");
				USART_RX_STA=0;
			}
		else if(USART_RX_STA==4)
			{  
				MotoBelt(0);
				printf("\n Encompass24  \n");
				USART_RX_STA=0;
			}
// 			Delay(50);
}	


u8 InfraredDetection(void)
{
	u8 HongWaiStatus=2;
	
	T1 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);
	//	printf("\n T1 = %d mV  \r\n",T1);
// 		Delay(50);
	
	T2 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
	//printf("\n T2 = %d mV  \r\n",T2);
// 		Delay(50);
	T3 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8);
	//printf("\n T3 = %d mV  \r\n",T3);
// 		Delay(50);
	T4 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9);
//	printf("\n T4 = %d mV  \r\n",T4);
// 		Delay(50);
	T5 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
	//printf("\n T5 = %d mV  \r\n",T5);
// 		Delay(50);
	clp = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
	//printf("\n clp = %d mV  \r\n",clp);
// 		Delay(50);
	distance = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);
	//printf("\n distance = %d mV  \r\n",distance);
// 		Delay(50);
		
	if( (!T1) && (!T2) && (!T3)&& (!T4)&& (!T5))//全部被遮挡时,认为进入减速带，准备RFID检测
	{

				PWMPulseHigh=60;
	}
	if( (T1) && (T2) && (T3)&& (T4)&& (T5))
	{

		HongWaiStatus=1;	//stop all 1(1)
	}
	if( (T1) && (T2) && (!T3)&& (T4)&& (T5))
	{

		HongWaiStatus=2;	//go along (2)
		
	}
	
	if( (T1) && (!T2) && (!T3)&& (!T4)&& (T5))
	{

		HongWaiStatus=2;	//go along (2)
	}
	
	
	if( (T1) && (!T2) && (!T3)&& (T4)&& (T5))
	{

		HongWaiStatus=2;  //turn left 1(3)
	}
	
	if( (!T1) && (!T2) && (T3)&& (T4)&& (T5))
	{

		HongWaiStatus=4;  //turn left 2(4)
	}
	if( (T1) && (!T2) && (T3)&& (T4)&& (T5))
	{

		HongWaiStatus=4;  //turn left 2(4)
	}
	if( (!T1) && (T2) && (T3)&& (T4)&& (T5))
	{

		HongWaiStatus=5;  //turn left 3(5)
	}
	if( (!T1) && (!T2) && (!T3)&& (T4)&& (T5))
	{

		HongWaiStatus=5;  //turn left 3(5)
	}
	if( (T1) && (T2) && (!T3)&& (!T4)&& (T5))
	{
		
		HongWaiStatus=2; //turn right 1(6)
	}
	
	if( (T1) && (T2) && (T3)&& (!T4)&& (!T5))
	{

		HongWaiStatus=7; //turn right 2(7)
	}
	if( (T1) && (T2) && (T3)&& (!T4)&& (T5))
	{

		HongWaiStatus=7; //turn right 2(7)
	}
	
	if( (T1) && (T2) && (T3)&& (T4)&& (!T5))
	{

		HongWaiStatus=8; //turn right 3(8)
	}
	if( (T1) && (T2) && (!T3)&& (!T4)&& (!T5))
	{

		HongWaiStatus=8; //turn right 3(8)
	}
	if(distance)
	{
		HongWaiStatus=9; 		 //closely (9)
	}

  if(clp)
	{
		HongWaiStatus=10; 		//Estop (10)
	}
	
	
	return HongWaiStatus;
	
}

/******以下为Wifi链接操作*********/

void  Wifi_Connect(void)
{
	u8 ifor;
 		
	if(Wifi_Flag==0)
		{
			for(ifor=0;ifor<34;ifor++)
		{USART_SendData(USART1, senddata1[ifor] );
  while (!(USART1->SR & USART_FLAG_TXE));
		}
		Wifi_Flag=1;
		Delay(1000);
		}
				
 		
	if(Wifi_Flag==2)
	{
		for(ifor=0;ifor<14;ifor++)
		{USART_SendData(USART1, senddata2[ifor] );
  while (!(USART1->SR & USART_FLAG_TXE));
		}
		Wifi_Flag=3;
		Delay(1000);
	}	
		
 		
	if(Wifi_Flag==4)
		{
			for(ifor=0;ifor<12;ifor++)
		{USART_SendData(USART1, senddata3[ifor] );
		while (!(USART1->SR & USART_FLAG_TXE));
		}
		Wifi_Flag=5;
		Delay(1000);
		}
 		
		if(Wifi_Flag==6)
		{
		Wifi_Flag=7;
		Delay(1000);
		}
		if(Wifi_Flag==7)
		{
			Wifi_Flag=8;  //WIFI连接OK标志(Wifi_Flag=8;)
			espFlag=1;
			Wifi_Touchuan=1;
		}
	}
		
/*****以上为Wifi链接操作*********/	


	
void RFIDReader(void)
{
	unsigned char status;
	unsigned char RFID_status = RFID_NO ;
	unsigned char s=0x08;
// /*>>>>>>>>>>>RFID>>>>>>>>>>>*/				
	switch (RFID_status)	{
		case RFID_NO :    status = PcdRequest(PICC_REQALL,CT);/*尋卡*/
											if (status==MI_OK)
											{
														 
												RFID_status = RFID_XunKa_OK;
												status=MI_ERR;
												status = PcdAnticoll(SN);/*防冲撞*/	
											}
		case RFID_XunKa_OK: 			
												if (status==MI_OK)//防衝撞
												{      
                             													
													RFID_status = RFID_FangChongZhuang_OK ;
													status=MI_ERR;
													status =PcdSelect(SN);//選卡	
														
												}									
	case RFID_FangChongZhuang_OK: 
																if (status==MI_OK)
																{  
                            																
																	RFID_status = RFID_XuanKa_OK  ;
																	status=MI_ERR;
																	status =PcdAuthState(0x60,0x09,KEY,SN);//驗證
																}
												
	case 	RFID_XuanKa_OK:		
													if (status==MI_OK)
													{         
														 	
														RFID_status = RFID_YanZheng_OK   ;
														status=MI_ERR;
														status=PcdRead(s,RFID); //读卡
													}					
		
	case RFID_YanZheng_OK:				
																if (status==MI_OK)
																{  
																	if(USFlag !=0)
																	{
																		 printf("READ_MI_OK the %d area data is  %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x     \n",s,RFID[0],RFID[1],RFID[2],RFID[3],RFID[4],RFID[5],RFID[6],RFID[7],RFID[8],RFID[9],RFID[10],RFID[11],RFID[12],RFID[13],RFID[14],RFID[15]);
																		 motorQZ_control(TIM2,0,1,4);
																		 motorQY_control(TIM2,0,2,4);
																		 motorHZ_control(TIM2,0,3,4);
																		 motorHY_control(TIM2,0,4,4);	
																		 RFID_SN_Control();
																		
																		
																		
																	}
														 
                                 PWMPulseHigh=80;																	
																	RFID_status = RFID_DuKa_OK   ;																	
																	status=MI_ERR;
																
																}										
	case 	RFID_DuKa_OK :			
                             
				                     RFID_status = RFID_NO;							
	default: RFID_status = RFID_NO;
	return;
	

// /*<<<<<<<<<<<<<<<<RFID<<<<<<<<<<<<<<*/	
/**********************以下为RFID-RC522操作********************/
// 	 GPIO_ResetBits(GPIOB,GPIO_Pin_8);
// 		 // Delay(500);
//       GPIO_SetBits(GPIOB,GPIO_Pin_8);
// 		 // Delay(500);
// 		
// 			status = PcdRequest(PICC_REQALL,CT);/*尋卡*/
// 				
// 			if(status==MI_OK)//尋卡成功
// 			{
// 				//寻卡成功之后急停一下
// 						 motorQZ_control(TIM2,0,1,4);
// 						 motorQY_control(TIM2,0,2,4);
// 						 motorHZ_control(TIM2,0,3,4);
// 						 motorHY_control(TIM2,0,4,4);	
// 						 Delay(50);
// 						if(USFlag==1)	
// 								{
// 									MotoBelt(2);
// 									printf("OKOKOKOKOKOK");
// 									 Delay(5000);
// 									MotoBelt(0);
// 									USFlag=0;
// 								}		
// 						
// 				
// // 				printf("PcdRequest_MI_OK,  TagType is %02x%02x     \n",CT[0],CT[1]);	
// 				GPIO_SetBits(GPIOB,GPIO_Pin_8);
// // 				printf("PcdRequest_MI_OK         \n");				
// 			  status=MI_ERR;
//         status = PcdAnticoll(SN);/*防冲撞*/		
// 				
// 				if (status==MI_OK)//防衝撞
// 			  { 
// // 					printf("PcdAnticoll_MI_OK,  SN is %02x%02x%02x%02x     \n",SN[0],SN[1],SN[2],SN[3]);	
// // 					printf("PcdAnticoll_MI_OK         \n");
// 					status=MI_ERR;	
// 					status =PcdSelect(SN);//選卡
// 					if(status==MI_OK)
// 					{
// // 						printf("PcdSelect_MI_OK         \n ");
// 						status=MI_ERR;	
// 						status =PcdAuthState(0x60,0x09,KEY,SN);//驗證
// 						if(status==MI_OK)
// 						{
// // 							printf("PcdAuthState_MI_OK         \n ");
// 							status=MI_ERR;	
// 							status=PcdRead(s,RFID);
// 							if(status==MI_OK)//讀卡成功
// 			        {
// // 									printf("READ_MI_OK the %d area data is  %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x     \n",s,RFID[0],RFID[1],RFID[2],RFID[3],RFID[4],RFID[5],RFID[6],RFID[7],RFID[8],RFID[9],RFID[10],RFID[11],RFID[12],RFID[13],RFID[14],RFID[15]);
// 								  status=MI_ERR;
// 									PWMPulseHigh=80;
// 													
// 							
// 						
// 						  }
// 							
// // 							status=PcdWrite(s,RFIDWrite);
// // 							if(status==MI_OK)//写卡成功
// // 			        {
// // // 									printf("Write_MI_OK the %d area data is  %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x     \n",s,RFIDWrite[0],RFIDWrite[1],RFIDWrite[2],RFIDWrite[3],RFIDWrite[4],RFIDWrite[5],RFIDWrite[6],RFIDWrite[7],RFIDWrite[8],RFIDWrite[9],RFIDWrite[10],RFIDWrite[11],RFIDWrite[12],RFIDWrite[13],RFIDWrite[14],RFIDWrite[15]);
// // 								testt=1;  
// // 								status=MI_ERR;
// // 								
// // 							
// // 							
// // 						
// // 						  }
// 						
// 					}
// 			
// 			 }

// 		}
	}			
		
/****************以上为RFID-RC522操作******************************/
}	

void RFID_SN_Control(void)  //Read RFID SN 判断相应动作  99 88 77 66 02
{
	if(USFlag==1 && RFID[15]==0x01 && RFID[14]==0x66 && RFID[13]==0x77 && RFID[12]==0x88 && RFID[11]==0x99)
	{
// 																		MotoBelt(1);
																		 MotoBelt(2);
																		 Delay(5000);	
																			USFlag=0;
																		 MotoBelt(0);
	}
	if(USFlag==2 && RFID[15]==0x02 && RFID[14]==0x66 && RFID[13]==0x77 && RFID[12]==0x88 && RFID[11]==0x99)
	{
																		MotoBelt(1);
// 																	 MotoBelt(2);
																		 Delay(5000);	
																			USFlag=0;
																		 MotoBelt(0);
	}
	
																			
}
