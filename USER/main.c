/********************HARDWARE********************
 * Œƒº˛√˚  £∫main.c
 * √Ë ˆ    £∫AGV
 *  µ—È∆ΩÃ®£∫MINI STM32ø™∑¢∞Â ª˘”⁄STM32F103C8T6
 * ø‚∞Ê±æ  £∫ST3.0.0
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

/**∂®“Â≤Œ ˝**/

#define  PWMPulseLow       80	
#define  PWMPulseMid1      80
#define  PWMPulseMid2      80
#define  timeeeer1          45
#define  timeeeer2          70
#define  timeeeer3          150
//////////////////////////////////////////////////////////
//M1ø®∑÷ûÈ16ÇÄ…»Ö^£¨√øÇÄ…»Ö^”…4âK£®âK0°¢âK1°¢âK2°¢âK3£©ΩM≥…
//Œ“ÇÉ“≤å¢16ÇÄ…»Ö^µƒ64ÇÄâK∞¥Ω^å¶µÿ÷∑æéÃñ0~63
//µ⁄0…»Ö^µƒâK0£®º¥Ω^å¶µÿ÷∑0âK£©£¨À˚”√Ï∂¥Ê∑≈èS…Ã¥˙¥a£¨“—ΩõπÃªØ£¨≤ªø…∏¸∏ƒ
//√øÇÄ…»Ö^µƒâK0°¢âK1°¢âK2ûÈîµì˛âK£¨ø…”√Ï∂¥Ê∑≈îµì˛
//√øÇÄ…»Ö^µƒâK3ûÈøÿ÷∆âK£®Ω^å¶µÿ÷∑âK3°¢7°¢11....£©£¨∞¸¿®¡À√‹¥aA£¨¥Ê»°øÿ÷∆°¢√‹¥aB°£

/*******************************
PA4°¢5°¢6°¢7£∫SPI1_NSS°¢SPI1_SCK°¢SPI1_MISO°¢SPI1_MOSI
*RDID¡¨œﬂÀµ√˜£∫
*1--SS£®sda£© <----->PF0  pa2   ¥”ª˙—°‘Ò  
*2--SCK <----->PB13    ±÷” ‰≥ˆ
*3--MOSI<----->PB15   …‰∆µ ˝æ› ‰≥ˆ
*4--MISO<----->PB14   …‰∆µƒ£øÈ ‰»Î
*5--–¸ø’
*6--GND <----->GND
*7--RST <----->PF1   pa3  –›√ﬂøÿ÷∆
*8--VCC <----->VCC   
************************************/
/*»´æ÷±‰¡ø*/
unsigned char CT[20];//ø®¿‡–Õ
unsigned char SN[4]; //ø®∫≈
unsigned char RFID[16];			//¥Ê∑≈RFID 
unsigned char RFIDWrite[16]={0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x07,0x80,0x29,0xff,0xff,0xff,0xff,0xff,0x0A};			//RFID writedata 
u8 printout[6]={0xBB, 0x5B, 0x00, 0x00,0x00,0xB5 };
// unsigned char card1_bit=0;
// unsigned char card2_bit=0;
// unsigned char card3_bit=0;
// unsigned char card4_bit=0;
//unsigned char total=0;
//unsigned char lxl[4]={6,109,250,186};
// unsigned char card_1[4]={66,193,88,0};
// unsigned char card_2[4]={66,191,104,0};
// unsigned char card_3[4]={62,84,28,11};
// unsigned char card_4[4]={126,252,248,12};
u8 KEY[6]={0xff,0xff,0xff,0xff,0xff,0xff};
//unsigned char RFID1[16]={0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x07,0x80,0x29,0xff,0xff,0xff,0xff,0xff,0x01};
unsigned char senddata1[]={0x41 ,0x54 ,0x2B ,0x43 ,0x49 ,0x50 ,0x53 ,0x54 ,0x41 ,0x52 ,0x54 ,0x3D ,0x22 ,
	                         0x54 ,0x43 ,0x50 ,0x22 ,0x2C ,0x22 ,0x31 ,0x30 ,0x2E ,0x30 ,0x2E ,0x31 ,0x2E ,
                           0x31,0X37 ,0x22 ,0x2C ,0x38 ,0x30 ,0x0D ,0x0A };  //10.0.1.17:80
unsigned char senddata2[]={0x41 ,0x54 ,0x2B ,0x43 ,0x49 ,0x50 ,0x4D ,0x4F ,0x44 ,0x45 ,0x3D ,0x31 ,0x0D ,0x0A };
unsigned char senddata3[]={0x41 ,0x54 ,0x2B ,0x43 ,0x49 ,0x50  ,0x53 ,0x45 ,0x4E ,0x44, 0x0D ,0x0A };
u8 USFlag=0;
volatile u32 T3time=0; // ms º∆ ±±‰¡ø

/*±‰¡ø∂®“Â«¯*/
	u8 USART_RX_BUF[64];     //Ω” ’ª∫≥Â,◊Ó¥Û64∏ˆ◊÷Ω⁄.
	u8 USART_RX_BUF_LINK[64];     //LinkWifi ±∫ÚΩ” ’ª∫≥Â,◊Ó¥Û64∏ˆ◊÷Ω⁄.
	u8 USART_RX_STA=0;       //Ω” ’◊¥Ã¨±Íº«
	u8 tempNumInc=0;//Ω” ’º∆ ˝
	uint8_t T1,T2,T3,T4,T5,clp,distance;  // ‰»ÎDI±‰¡ø
	u8 counter_BUF=0;
	u8 counter_BUF_LINK=0;
	u8 testt=0;
	u8 espFlag=0;
  u8 Wifi_Flag=0;
	u8 Wifi_Touchuan=0; //Õ∏¥´±Í÷æ
  u8 WifiStartR=0;
  u8 UploadCardNumber=0,DownloadCardNumber=0,Standby=0;//Õ®–≈÷∏¡Ó÷∏¡Óagv◊∞‘ÿŒª÷√°¢–∂‘ÿŒª÷√°¢¥˝ª˙Œª÷√
  u8 ReadedCard;
//bool T1,T2,T3,T4,T5,clp;  // ‰»ÎDI±‰¡ø
u8 Track; 	 //––∂Ø¬∑œﬂ
u8 FLAG;//Õ®π˝∑«—≠º£∑Ω Ωøÿ÷∆agv––◊ﬂ

u8 TouYanZheng=0;//Õ®–≈÷∏¡ÓÕ∑—È÷§£¨»Áπ˚Õ∑—È÷§Õ®π˝£¨‘Úø™ ºΩ” ’÷∏¡Ó œﬁ ±Ω” ’£¨‘⁄πÊ∂® ±º‰ƒ⁄Ω” ’≤ªÕÍ’˚‘Ú≈◊∆˙Ω” ’ƒ⁄»›
u8 timeout=0;//”√”⁄Ω” ’≥¨ ±≈–∂œ
u32 SysTickCountFlag =0;  //SysTick÷–∂œº∆ ˝
u32 SysTickCountFlag_1_0ms =(1000*100);  //SysTick÷–∂œ1msº∆ ˝
u32 SysTickCountFlag_1_5ms =(1500*100);  //SysTick÷–∂œ1.5msº∆ ˝
u32 SysTickCountFlag_2_0ms =(2000*100);  //SysTick÷–∂œ2msº∆ ˝
u32 SysTickCountFlag_Max =100000;  //SysTick÷–∂œº∆ ˝µΩ
u32 HighSysTick =0;  //SysTick ∂Êª˙øÿ÷∆£∫20ms÷‹∆⁄£¨0.5ms~2.5ms∏ﬂµÁ∆Ω’ºø’±»-------0∂»~180∂»(∏ﬂµÁ∆Ω ±÷”øÿ÷∆)HighSysTick »°÷µ50~250
u32 LowSysTick = 0;  //SysTick ∂Êª˙øÿ÷∆£∫20ms÷‹∆⁄£¨0.5ms~2.5ms∏ﬂµÁ∆Ω’ºø’±»-------0∂»~180∂»(µÕµÁ∆Ω ±÷”øÿ÷∆)LowSysTick = 2000-HighSysTick£ª
u8  High_Low = 0;   // ‘⁄∏ﬂµÁ∆Ω ‰≥ˆ ±øÃªπ «µÕµÁ∆Ω ‰≥ˆ ±øÃ(0-∏ﬂµÁ∆Ω ±øÃ£¨1-µÕµÁ∆Ω ±øÃ)
//u32 SysTick_TimeSet=100000;     //SysTickµƒ ±º‰ª˘ ˝

u16  PWMPulseHigh=80;
unsigned char GetData;
extern __IO u16 ADC_ConvertedValue;	 
// »Ìº˛—” ±
void Delay(unsigned long time)
{unsigned long i,j;
  
	for(j=0; j<time; j++)
	{
	   for(i=0;i<12000;i++);
	}
}

// Ω‚∑≈PA11∫ÕPA12£¨ πµ√∆‰ø…“‘◊ˆ∆’Õ®GPIO π”√  »°œ˚”≥œÒcanΩ”ø⁄   …Ë÷√AFIO->MAPRµƒ14£∫13Œª=01
void USBCAN_GPIO_NoRemap(void)
{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // πƒ‹∂‘”¶GPIO ±÷”
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   // πƒ‹AFIO÷ÿ”≥…‰ ±÷”
		AFIO->MAPR |= 0x2000;				//Ω¯––÷ÿ”≥…‰≈‰÷√ 
		AFIO->MAPR &=~0x4000;
}	

//Ω‚∑≈PA15∫ÕPB3∫ÕPB4£¨ πµ√∆‰ø…“‘◊ˆ∆’Õ®GPIO π”√
void JTAGDisable_GPIO_NoRemap(void) 
{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // πƒ‹∂‘”¶GPIO ±÷”
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // πƒ‹∂‘”¶GPIO ±÷”
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   // πƒ‹AFIO÷ÿ”≥…‰ ±÷”
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //πÿ±’JTAG-DP,ø™∆ÙSW-DP
		
// 		AFIO->MAPR &=~0x01000000;										//÷±Ω”≤Ÿ◊˜ºƒ¥Ê∆˜ µœ÷								
// 		AFIO->MAPR |= 0x02000000;
// 		AFIO->MAPR &=~0x04000000;
}
	


//∫Ø ˝…˘√˜«¯”Ú
void LED_GPIO_Config(void);
void ControlDI_AGV_GPIO_Config(void);
void ControlDO_AGV_GPIO_Config(void);
u8 RFIDReader(void);
void MotoBelt(u16 direction);
void MotoBeltControl(void);//∆§¥¯µÁª˙Õ®–≈øÿ÷∆
u8  InfraredDetection(void);
void USART1_IRQHandler(void);
unsigned char UART1GetByte(void);
void NVIC_Configuration(void);
void Wifi_Connect(void);
void TIM3_NVIC_Configuration(void);
void TIM3_Configuration(void);
void BOOT1_ReleaseToGPIO(void);
//void RFID_SN_Control(void);
void AGVRun(void);
void DuoJi(u16 jiaodu);
void uartsend(u8 data);


int main(void)
{ 

 	USBCAN_GPIO_NoRemap();   // Ω‚∑≈PA11∫ÕPA12
 	JTAGDisable_GPIO_NoRemap(); 	//Ω‚∑≈PA15∫ÕPB3∫ÕPB4
	BOOT1_ReleaseToGPIO();//Ω‚∑≈boot1∂Àø⁄
  /* ≈‰÷√œµÕ≥ ±÷”Œ™72M */      
  SystemInit();	
  /* ≈‰÷√¥Æø⁄ */
  USART1_Config();
  /* ≥ı ºªØADC1 */
//  ADC1_Init();
	/*≥ı ºªØ◊¥Ã¨µ∆∂Àø⁄£¨œ÷“—æ≠”√Œ™∂Êª˙µƒøÿ÷∆“˝Ω≈*/
	LED_GPIO_Config();
	/*≥ı ºªØ ‰»Î*/
	ControlDI_AGV_GPIO_Config();
	/*≥ı ºªØ ‰≥ˆ*/
	ControlDO_AGV_GPIO_Config();	
	/*≥ı ºªØPWMøÿ÷∆*/
	PWM_RCC_Configuration();
	PWM_GPIO_Configuration();
	TIM2_Configuration();
	//TIM3_Configuration();
	motor_control();//≥ı ºªØ≥µ¬÷øÿ÷∆
	//PCø⁄≥ı ºªØ
	ControlDOC_AGV_GPIO_Config();
	NVIC_Configuration();
	/*∂Êª˙≥ı ºªØ…Ë÷√*/
	HighSysTick =150;  //SysTick ∂Êª˙øÿ÷∆£∫20ms÷‹∆⁄£¨0.5ms~2.5ms∏ﬂµÁ∆Ω’ºø’±»-------0∂»~180∂»(∏ﬂµÁ∆Ω ±÷”øÿ÷∆)HighSysTick »°÷µ50~250
  LowSysTick =  2000-HighSysTick;  //SysTick ∂Êª˙øÿ÷∆£∫20ms÷‹∆⁄£¨0.5ms~2.5ms∏ﬂµÁ∆Ω’ºø’±»-------0∂»~180∂»(µÕµÁ∆Ω ±÷”øÿ÷∆)LowSysTick = 2000-HighSysTick£ª
  High_Low = 0; 
  /* ≈‰÷√SysTick Œ™10us÷–∂œ“ª¥Œ */
	SysTick_Init();
	
	//GPIO_SetBits(GPIOB, GPIO_Pin_8);//led off
  /*◊™‘À∆§¥¯≥ı ºªØ*/
	MotoBelt(0);//0=Õ£÷π£¨1=’˝◊™£¨2=∑¥◊™
	USART_ITConfig(USART1, USART_IT_RXNE , ENABLE);		//USART1Ω” ’÷–∂œ πƒ‹
	TIM3_NVIC_Configuration(); /* TIM3(÷–∂œ”≈œ»º∂) ∂® ±≈‰÷√ */
  TIM3_Configuration(); 	 /* TIM3(≥ı ºªØ) ∂® ±≈‰÷√ */
	
//  	printf("Uart init OK            \n");	
	InitRc522();				//≥ı ºªØ…‰∆µø®ƒ£øÈ
	ReadedCard=0;//≥ı ºªØ∂¡»°µΩµƒø®∫≈
	Standby=0;
	Delay(5000);
	RFID[15]=0;
  DuoJi(300);//∂Êª˙◊™µΩ’⁄µ≤Œª÷
	
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
// 	}
 //***********SysTick Testing*********************
	
	

		while(1)
	 { 

			if(espFlag==0) //‘⁄Œ¥‘¯¡¨…œWIFI ±∫Ú£¨≤≈Ω¯––WIFI¡¥Ω”≤Ÿ◊˜£¨¡¥Ω”…œ∫ÛΩ´≤ª‘Ÿ÷¥––¥À≤Ÿ◊˜°£
			{ 
				Wifi_Connect();			
			}						
			while (espFlag)
			{					
				
				//***********∑÷≤º≤‚ ‘π˝≥Ã*********************
// 				while(1)
// 				{
//   				//AGVRun();	//–°≥µ––Ω¯◊‘Œ“øÿ÷∆
//  				RFIDReader();	//…‰∆µø®ºÏ≤‚
// 				}
								
				//***********∑÷≤º≤‚ ‘π˝≥Ã*********************	
			
				// UART1GetByte();//÷∏¡ÓΩ” ’
				AGVRun();	//–°≥µ––Ω¯◊‘Œ“øÿ÷∆													
				if( (UploadCardNumber!=0) || (DownloadCardNumber!=0) )//Ω” ’µΩ◊∞–∂‘ÿ÷∏¡Ó
				{     			
					if(RFID[15]==0)
					{				
						RFIDReader();	//…‰∆µø®ºÏ≤‚
					}
					else if(RFID[15]!=0)
					{ 
						if((RFID[15]==UploadCardNumber)||(RFID[15]==DownloadCardNumber))
						{ ReadedCard =RFID[15];}
						else if((RFID[15]!=UploadCardNumber)&&(RFID[15]!=DownloadCardNumber))
						{ RFID[15]=0;
            PWMPulseHigh=80;//ª÷∏¥agv––Ω¯ÀŸ∂»
             }
					}
						
					if((ReadedCard == UploadCardNumber)&&(UploadCardNumber!=0) && (DownloadCardNumber!=0))  //∂¡µΩø®∆¨   get into Upload process   
					{
							PWMPulseHigh=80;//ª÷∏¥agv––Ω¯ÀŸ∂»	
							//FLAG=0;// AGV STOP
							 motorQZ_control(TIM2,0,1,1);//stop
							 motorQY_control(TIM2,0,2,1);
							 motorHZ_control(TIM2,0,3,1);
							 motorHY_control(TIM2,0,4,1);		
               Delay(500);	//µ»¥˝∂Êª˙◊™∂ØµΩŒª							
							 DuoJi(300);//∂Êª˙◊™µΩ’⁄µ≤Œª÷√	
							 Delay(1000);	//µ»¥˝∂Êª˙◊™∂ØµΩŒª
							 motorQZ_control(TIM2,PWMPulseHigh,1,2);//run
							 motorQY_control(TIM2,PWMPulseHigh,2,2);
							 motorHZ_control(TIM2,PWMPulseHigh,3,2);
							 motorHY_control(TIM2,PWMPulseHigh,4,2);							
							 Delay(300);	//µ»¥˝∂Êª˙◊™∂ØµΩŒª
							 motorQZ_control(TIM2,0,1,1);//stop
							 motorQY_control(TIM2,0,2,1);
							 motorHZ_control(TIM2,0,3,1);
							 motorHY_control(TIM2,0,4,1);			
							 Delay(500);	//µ»¥˝∂Êª˙◊™∂ØµΩŒª
							 motorQZ_control(TIM2,PWMPulseHigh,1,3);//back
						   motorQY_control(TIM2,PWMPulseHigh,2,3);
						   motorHZ_control(TIM2,PWMPulseHigh,3,3);
						   motorHY_control(TIM2,PWMPulseHigh,4,3);
							 Delay(200);	//µ»¥˝∂Êª˙◊™∂ØµΩŒª
							 motorQZ_control(TIM2,0,1,1);//stop
							 motorQY_control(TIM2,0,2,1);
							 motorHZ_control(TIM2,0,3,1);
							 motorHY_control(TIM2,0,4,1);	
// 							 motorQZ_control(TIM2,PWMPulseHigh,1,2);//run
// 							 motorQY_control(TIM2,PWMPulseHigh,2,2);
// 							 motorHZ_control(TIM2,PWMPulseHigh,3,2);
// 							 motorHY_control(TIM2,PWMPulseHigh,4,2);							
// 							 Delay(30);
// 							 motorQZ_control(TIM2,0,1,1);//stop
// 							 motorQY_control(TIM2,0,2,1);
// 							 motorHZ_control(TIM2,0,3,1);
// 							 motorHY_control(TIM2,0,4,1);	
// 	             Delay(100);	//µ»¥˝∂Êª˙◊™∂ØµΩŒª							 
// 							 motorQZ_control(TIM2,PWMPulseHigh,1,2);//run
// 							 motorQY_control(TIM2,PWMPulseHigh,2,2);
// 							 motorHZ_control(TIM2,PWMPulseHigh,3,2);
// 							 motorHY_control(TIM2,PWMPulseHigh,4,2);							
// 							 Delay(30);	//µ»¥˝∂Êª˙◊™∂ØµΩŒª
// 							 motorQZ_control(TIM2,0,1,1);//stop
// 							 motorQY_control(TIM2,0,2,1);
// 							 motorHZ_control(TIM2,0,3,1);
// 							 motorHY_control(TIM2,0,4,1);							
// 							 Delay(100);	//µ»¥˝∂Êª˙◊™∂ØµΩŒª

							 //PWMPulseHigh=60;//ª÷∏¥agv––Ω¯ÀŸ∂»
 							//FLAG=1;//AGV RUN Ω´ÀÆ∆ø◊∞‘ÿ
               //Delay(1000);//µ»¥˝◊∞‘ÿÕÍ≥…
              // DuoJi(180);	//∂Êª˙◊™µΩÕ®π˝Œª÷√
							 Delay(500);//µ»¥˝agv–∂‘ÿµΩŒª
							 DuoJi(220);	//∂Êª˙◊™µΩÕ®π˝Œª÷√
							 MotoBelt(1);//–∂‘ÿ 
							 Delay(500);//µ»¥˝agv–∂‘ÿµΩŒª
						   MotoBelt(0);//–∂‘ÿ
 							 Delay(700);	//µ»¥˝∂Êª˙◊™∂ØµΩŒª
  						 //PWMPulseHigh=80;//ª÷∏¥agv––Ω¯ÀŸ∂»			
               RFID[15]=0;							
  						 ReadedCard = 0;   //  get out Upload process	
							 //printf("%02x %02x %02x %02x %02x %02x \n", 187, 91,0,0, 179, 181);					     				
							 UploadCardNumber=0;
							 uartsend(0xbb);
							 uartsend(0x5b);
							 uartsend(0x00);
							 uartsend(0x00);
							 uartsend(0xb3);
							 uartsend(0xb5);
											
// 						 printf("Upload finished");
							 
						}
						
			
												
				  if((ReadedCard==DownloadCardNumber) && (UploadCardNumber==0)&& (DownloadCardNumber!=0))
					{												
							
							 motorQZ_control(TIM2,0,1,1);// AGV STOP
							 motorQY_control(TIM2,0,2,1);
							 motorHZ_control(TIM2,0,3,1);
							 motorHY_control(TIM2,0,4,1);
							 MotoBelt(2);//–∂‘ÿ 
							 Delay(4000);//µ»¥˝agv–∂‘ÿµΩŒª
						   MotoBelt(0);//–∂‘ÿ
						   PWMPulseHigh=80;//ª÷∏¥agv––Ω¯ÀŸ∂»	
						  
							 motorQZ_control(TIM2,PWMPulseHigh,1,2);//AGV RUN
							 motorQY_control(TIM2,PWMPulseHigh,2,2);
							 motorHZ_control(TIM2,PWMPulseHigh,3,2);
							 motorHY_control(TIM2,PWMPulseHigh,4,2);
							 RFID[15]=0;						
							 ReadedCard = 0;   //  get out Download process
					 //  printf("Download finished");										 		  
							 DownloadCardNumber=0;	
							 Standby = 1;
						   uartsend(0xbb);
							 uartsend(0x5b);
							 uartsend(0x00);
							 uartsend(0x00);
							 uartsend(0xb4);
							 uartsend(0xb5);
							
					}
					if((ReadedCard==DownloadCardNumber) && (UploadCardNumber!=0))//»Áπ˚∂¡µΩ–∂‘ÿø®∆¨£¨µ´ «ªπŒ¥◊∞‘ÿ£¨ƒ«√¥æÕª÷∏¥agvÀŸ∂»
					{
		           RFID[15]=0;	
						   ReadedCard = 0;   //  get out Download process									 
							 PWMPulseHigh=80;//ª÷∏¥agv––Ω¯ÀŸ∂»	

					}
			
				}
				if(Standby == 1)   //–∂‘ÿÕÍ≥…∫Û
				{  
									
					if(RFID[15] == 0)
					{				
						RFIDReader();	//…‰∆µø®ºÏ≤‚
					}
				  if(RFID[15] == 0xff)
					{ 
						
						 Standby =0;
						 FLAG=0;// AGV STOP
						 motorQZ_control(TIM2,0,1,1);//stops
						 motorQY_control(TIM2,0,2,1);
						 motorHZ_control(TIM2,0,3,1);
						 motorHY_control(TIM2,0,4,1);	
						 PWMPulseHigh=80;//ª÷∏¥agv––Ω¯ÀŸ∂»
						 RFID[15]=0;
					//	 Delay(1000);	//µ»¥˝∂Êª˙◊™∂ØµΩŒª
					}										
					if((RFID[15]!= 0xff) && (RFID[15]!= 0))
					{		 			   
				    	RFID[15] = 0;		
						  PWMPulseHigh=80;//ª÷∏¥agv––Ω¯ÀŸ∂»			
	            						
					}
					
			 }
		
		  }
	}
}

//¥Æø⁄∑¢ÀÕ“ª∏ˆ◊÷Ω⁄ 
void uartsend(u8 data)
{
  USART_SendData(USART1, data);
	while (!(USART1->SR & USART_FLAG_TXE));	
}


//∂ÊôCΩ«∂»øÿ÷∆ 50-250÷ÆÈg‘O÷√£¨å¶ë™0-180∂»  
void DuoJi(u16 jiaodu)
{
// 	 High_Low = 0;
	 HighSysTick=jiaodu;   
	 SysTick->CTRL |= 0x00000003; 
}


//–°≥µ––Ω¯øÿ÷∆
void AGVRun(void)
		{
			u8 testu8;
			if(FLAG!=0)
			{
				testu8=InfraredDetection();//∫ÏÕ‚ºÏ≤‚∞Â—≠º£ºÏ≤‚
			}
			else testu8=FLAG;
			
			/*********“‘œ¬ «AGV Control*************/
					if(FLAG==0||testu8==0)
					{ //printf("stop all 0(0),Track= %d %d \n",Track,testu8);
						
							 motorQZ_control(TIM2,0,1,1);
							 motorQY_control(TIM2,0,2,1);
							 motorHZ_control(TIM2,0,3,1);
							 motorHY_control(TIM2,0,4,1);
					}
					if(FLAG!=0&&testu8==1)
				 { //printf("Rollback 1(1) ,Track= %d %d \n",Track,testu8);
							 motorQZ_control(TIM2,PWMPulseHigh,1,3);
						   motorQY_control(TIM2,PWMPulseHigh,2,3);
						   motorHZ_control(TIM2,PWMPulseHigh,3,3);
						   motorHY_control(TIM2,PWMPulseHigh,4,3);
				 }
				
					if(FLAG!=0&&testu8==2)
					{// printf("Go along(2) ,Track= %d %d \n",Track,testu8);
						 motorQZ_control(TIM2,PWMPulseHigh,1,2);
						 motorQY_control(TIM2,PWMPulseHigh,2,2);
						 motorHZ_control(TIM2,PWMPulseHigh,3,2);
						 motorHY_control(TIM2,PWMPulseHigh,4,2);
					}
						 
						
				if(FLAG!=0&&testu8==3)
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
				if(FLAG!=0&&testu8==4)
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
				if(FLAG!=0&&testu8==5)		 
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
				if(FLAG!=0&&testu8==6)
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
				if(FLAG!=0&&testu8==7)
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
				if(FLAG!=0&&testu8==8)
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
				if(FLAG!=0&&testu8==9)
				{//	printf("closely(9) ,Track= %d %d \n",Track,testu8);
						 motorQZ_control(TIM2,0,1,4);
						 motorQY_control(TIM2,0,2,4);
						 motorHZ_control(TIM2,0,3,4);
						 motorHY_control(TIM2,0,4,4);
				}

				if(FLAG!=0&&testu8==10)
				{ //printf("Estop(10) ,Track= %d %d \n",Track,testu8);
						 motorQZ_control(TIM2,0,1,4);
						 motorQY_control(TIM2,0,2,4);
						 motorHZ_control(TIM2,0,3,4);
						 motorHY_control(TIM2,0,4,4);
				}

			}




//–°≥µ∆§¥¯’˝∑¥◊™øÿ÷∆
void MotoBelt(u16 direction)	//0=Õ£÷π£¨1=’˝◊™£¨2=∑¥◊™
	{
		if(direction==1)  //’˝◊™
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_10);  //1
			GPIO_ResetBits(GPIOB, GPIO_Pin_2);   //4	
      GPIO_SetBits(GPIOB, GPIO_Pin_11);    //2	
			GPIO_SetBits(GPIOA, GPIO_Pin_15);    //3			
		}
		if(direction==2)  //∑¥◊™
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_11);  //2
			GPIO_ResetBits(GPIOA, GPIO_Pin_15);  //3
      GPIO_SetBits(GPIOB, GPIO_Pin_10);    //1	
      GPIO_SetBits(GPIOB, GPIO_Pin_2);     //4				
		}
		if(direction==0)  //Õ£÷π
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_10);  //1
			GPIO_SetBits(GPIOB, GPIO_Pin_11);  //2	
			GPIO_SetBits(GPIOA, GPIO_Pin_15);  //3
			GPIO_SetBits(GPIOB, GPIO_Pin_2);   //4		
		}
	
	}
	
		
void MotoBeltControl(void)//∆§¥¯µÁª˙Õ®–≈øÿ÷∆
{
	if(USART_RX_STA==2)//Ω” ’’˝»∑
			{			
				
				GPIO_ResetBits(GPIOB, GPIO_Pin_8);//LED√
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


//—≠º£ºÏ≤‚
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
		
	if( (!T1) && (!T2) && (!T3)&& (!T4)&& (!T5)&&((UploadCardNumber!=0) || (DownloadCardNumber!=0) || (Standby==1) ))//»´≤ø±ª’⁄µ≤ ±,»œŒ™Ω¯»ÎºıÀŸ¥¯£¨◊º±∏RFIDºÏ≤‚
	{

				PWMPulseHigh=60;
	}
	if( (T1) && (T2) && (T3)&& (T4)&& (T5))
	{

		HongWaiStatus=1;	//roll back (1)
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

// Wifi¡¥Ω”≤Ÿ◊˜
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
			Wifi_Flag=8;  //WIFI¡¨Ω”OK±Í÷æ(Wifi_Flag=8;)
			espFlag=1;
			Wifi_Touchuan=1;
// 	  printf("WIFI connected");
       uartsend(0xbb);
			 uartsend(0x5b);
			 uartsend(0x00);
			 uartsend(0x00);
			 uartsend(0xb1);
			 uartsend(0xb5);
			DuoJi(220);//∂Êª˙◊™µΩ’⁄µ≤Œª÷
			Delay(5000);
		}
	}
//RFID∂¡ø®
u8 RFIDReader(void)
{
	unsigned char status;
	unsigned char RFID_status = RFID_NO ;
	unsigned char s=0x08;
	u8 CardNumber;
// /*>>>>>>>>>>>RFID>>>>>>>>>>>*/				
	switch (RFID_status)	{
		case RFID_NO :    status = PcdRequest(PICC_REQALL,CT);/*å§ø®*/
											if (status==MI_OK)
											{
														 
												RFID_status = RFID_XunKa_OK;
// 											printf("PcdRequest_MI_OK,  TagType is %02x%02x     \n",CT[0],CT[1]);	
												status=MI_ERR;
												status = PcdAnticoll(SN);/*∑¿≥Â◊≤*/	
											}
		case RFID_XunKa_OK: 			
												if (status==MI_OK)//∑¿–n◊≤
												{      
                         													
													RFID_status = RFID_FangChongZhuang_OK ;
// 												printf("PcdAnticoll_MI_OK,  SN is %02x%02x%02x%02x     \n",SN[0],SN[1],SN[2],SN[3]);	
													status=MI_ERR;
													status =PcdSelect(SN);//ﬂxø®	
														
												}									
	case RFID_FangChongZhuang_OK: 
																if (status==MI_OK)
																{  
                            															
																	RFID_status = RFID_XuanKa_OK  ;
																	status=MI_ERR;
																	status =PcdAuthState(0x60,0x09,KEY,SN);//Úû◊C
																}
												
	case 	RFID_XuanKa_OK:		
													if (status==MI_OK)
													{         
														
														RFID_status = RFID_YanZheng_OK   ;
														status=MI_ERR;
// status=PcdWrite(s,RFIDWrite); //–¥ø®
// status=MI_ERR;
														 status=PcdRead(s,RFID); //∂¡ø®
														
													}					
		
	case RFID_YanZheng_OK:				
																if (status==MI_OK)
																{  

																	  CardNumber=RFID[15]; 																						
// 																	printf("READ_Card the %d area data is  %02x  \n",s,CardNumber);																									
	      uartsend(0xbb);
			 uartsend(0x5b);
			 uartsend(CardNumber);
			 uartsend(0x00);
			 uartsend(0xb6);
			 uartsend(0xb5);

																	
      USART_SendData(USART1, 0xbb);
			while (!(USART1->SR & USART_FLAG_TXE));
			USART_SendData(USART1, 0x5b);
			while (!(USART1->SR & USART_FLAG_TXE));
			USART_SendData(USART1, CardNumber);
			while (!(USART1->SR & USART_FLAG_TXE));
			USART_SendData(USART1, 0x00);
			while (!(USART1->SR & USART_FLAG_TXE));
			USART_SendData(USART1, 0xb6);
			while (!(USART1->SR & USART_FLAG_TXE));
			USART_SendData(USART1, 0xb5);
			while (!(USART1->SR & USART_FLAG_TXE));


																	//printf("%2x %2x %2x %2x %2x %2x \n", 0xBB, 0x5B,CardNumber,0x00, 0xB6, 0xB5);				
																		RFID_status = RFID_DuKa_OK   ;																	
																		status=MI_ERR;
																
																}										
	case 	RFID_DuKa_OK :			
                             
				                     RFID_status = RFID_NO;							
	default: RFID_status = RFID_NO;
	
	return CardNumber;
	

// /*<<<<<<<<<<<<<<<<RFID<<<<<<<<<<<<<<*/	
/**********************“‘œ¬Œ™RFID-RC522≤Ÿ◊˜********************/
// 	 GPIO_ResetBits(GPIOB,GPIO_Pin_8);
// 		 // Delay(500);
//       GPIO_SetBits(GPIOB,GPIO_Pin_8);
// 		 // Delay(500);
// 		
// 			status = PcdRequest(PICC_REQALL,CT);/*å§ø®*/
// 				
// 			if(status==MI_OK)//å§ø®≥…π¶
// 			{
// 				//—∞ø®≥…π¶÷Æ∫Ûº±Õ£“ªœ¬
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
//         status = PcdAnticoll(SN);/*∑¿≥Â◊≤*/		
// 				
// 				if (status==MI_OK)//∑¿–n◊≤
// 			  { 
// // 					printf("PcdAnticoll_MI_OK,  SN is %02x%02x%02x%02x     \n",SN[0],SN[1],SN[2],SN[3]);	
// // 					printf("PcdAnticoll_MI_OK         \n");
// 					status=MI_ERR;	
// 					status =PcdSelect(SN);//ﬂxø®
// 					if(status==MI_OK)
// 					{
// // 						printf("PcdSelect_MI_OK         \n ");
// 						status=MI_ERR;	
// 						status =PcdAuthState(0x60,0x09,KEY,SN);//Úû◊C
// 						if(status==MI_OK)
// 						{
// // 							printf("PcdAuthState_MI_OK         \n ");
// 							status=MI_ERR;	
// 							status=PcdRead(s,RFID);
// 							if(status==MI_OK)//◊xø®≥…π¶
// 			        {
// // 									printf("READ_MI_OK the %d area data is  %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x     \n",s,RFID[0],RFID[1],RFID[2],RFID[3],RFID[4],RFID[5],RFID[6],RFID[7],RFID[8],RFID[9],RFID[10],RFID[11],RFID[12],RFID[13],RFID[14],RFID[15]);
// 								  status=MI_ERR;
// 									PWMPulseHigh=80;
					
// 						  }
// 							
// // 							status=PcdWrite(s,RFIDWrite);
// // 							if(status==MI_OK)//–¥ø®≥…π¶
// // 			        {
// // // 									printf("Write_MI_OK the %d area data is  %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x     \n",s,RFIDWrite[0],RFIDWrite[1],RFIDWrite[2],RFIDWrite[3],RFIDWrite[4],RFIDWrite[5],RFIDWrite[6],RFIDWrite[7],RFIDWrite[8],RFIDWrite[9],RFIDWrite[10],RFIDWrite[11],RFIDWrite[12],RFIDWrite[13],RFIDWrite[14],RFIDWrite[15]);
// // 								testt=1;  
// // 								status=MI_ERR; 						
// // 						  }
// 						
// 					}
// 			
// 			 }

// 		}
	}			
		
/****************“‘…œŒ™RFID-RC522≤Ÿ◊˜******************************/
}	


