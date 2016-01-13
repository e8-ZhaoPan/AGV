/**
  ******************************************************************************
  * @file    spi_driver.c
  * $Author: wdluo $
  * $Revision: 17 $
  * $Date:: 2012-07-06 11:16:48 +0800 #$
  * @brief   SPI底层驱动函数.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "spi_driver.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  使能SPI时钟
  * @param  SPIx 需要使用的SPI
  * @retval None
  */
static void SPI_RCC_Configuration(SPI_TypeDef* SPIx ,u8 Remap)
{
	if(Remap==0)    //SPI没有重映射
	{
		if(SPIx==SPI1){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1,ENABLE);
	}else{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	}
	}
	
	if(Remap==1)   //SPI有重映射
	{
		if(SPIx==SPI1){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_SPI1,ENABLE);
	}else{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	}
	}
	
}
/**
  * @brief  配置指定SPI的引脚
  * @param  SPIx 需要使用的SPI
  * @retval None
  */
static void SPI_GPIO_Configuration(SPI_TypeDef* SPIx , u8 Remap )
{
	GPIO_InitTypeDef GPIO_InitStruct;
		if(Remap==0)   //SPI没有重映射
		{
			if(SPIx==SPI1){					 					 
		GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6|GPIO_Pin_7;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 
		GPIO_Init(GPIOA, &GPIO_InitStruct);
		//初始化片选输出引脚
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStruct);
		GPIO_SetBits(GPIOA,GPIO_Pin_4);
	}else{
		GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14|GPIO_Pin_15;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		//初始化片选输出引脚
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
	} 
	}
	
		if(Remap==1)    //SPI有重映射
		{
			GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);   //SPI1重映射使能
			if(SPIx==SPI1){					 					 
		GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4|GPIO_Pin_5;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		//初始化片选输出引脚
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStruct);
		GPIO_SetBits(GPIOA,GPIO_Pin_15);
	}else{
		GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14|GPIO_Pin_15;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		//初始化片选输出引脚
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
	} 
	}
	
}
/**
  * @brief  根据外部SPI设备配置SPI相关参数
  * @param  SPIx 需要使用的SPI
  * @retval None
  */
void SPI_Configuration(SPI_TypeDef* SPIx)
{
	SPI_InitTypeDef SPI_InitStruct;

	SPI_RCC_Configuration(SPIx,0);

	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;   //波特率预分频值为32  Chain提示此处注意，通信时钟由主SPI的时钟分配而得，不需要设置从SPI的时钟
	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;    //双线双向全双工
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;    //主SPI设备
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;  //SPI数据大小	
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;   //时钟悬空低
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;   //数据捕获与第一个时钟沿
	SPI_InitStruct.SPI_NSS = SPI_NSS_Hard;   //NSS由外部管脚管理
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;   //数据传输从MSB开始
	SPI_InitStruct.SPI_CRCPolynomial = 7;   //定义用于CRC值计算的多项式
	SPI_Init(SPIx, &SPI_InitStruct);    //配置以上SPI的配置
	
	SPI_GPIO_Configuration(SPIx,0);

	SPI_SSOutputCmd(SPIx, ENABLE);   //使能指定的SPI SS输出
	SPI_Cmd(SPIx, ENABLE);   //使能SPI外设
}
/**
  * @brief  写1字节数据到SPI总线
  * @param  SPIx 需要使用的SPI
  * @param  TxData 写到总线的数据
  * @retval 数据发送状态
  *		@arg 0 数据发送成功
  * 	@arg -1 数据发送失败
  */
int32_t SPI_WriteByte(SPI_TypeDef* SPIx, uint16_t TxData)
{
	uint8_t retry=0;				 
	while((SPIx->SR&SPI_I2S_FLAG_TXE)==0);				//等待发送区空	
	{
		retry++;
		if(retry>200)return -1;
	}			  
	SPIx->DR=TxData;	 	  				//发送一个byte 
	retry=0;
	while((SPIx->SR&SPI_I2S_FLAG_RXNE)==0); 				//等待接收完一个byte  
	{
		retry++;
		if(retry>200)return -1;
	}  
	SPIx->DR;						    
	return 0;          				//返回收到的数据
}
/**
  * @brief  从SPI总线读取1字节数据
  * @param  SPIx 需要使用的SPI
  * @param  p_RxData 数据储存地址
  * @retval 数据读取状态
  *		@arg 0 数据读取成功
  * 	@arg -1 数据读取失败
  */
int32_t SPI_ReadByte(SPI_TypeDef* SPIx, uint16_t *p_RxData)
{
	uint8_t retry=0;				 
	while((SPIx->SR&SPI_I2S_FLAG_TXE)==0);				//等待发送区空	
	{
		retry++;
		if(retry>200)return -1;
	}			  
	SPIx->DR=0xFF;	 	  				//发送一个byte 
	retry=0;
	while((SPIx->SR&SPI_I2S_FLAG_RXNE)==0); 				//等待接收完一个byte  
	{
		retry++;
		if(retry>200)return -1;
	}
	*p_RxData = SPIx->DR;  						    
	return 0;          				//返回收到的数据
}
/**
  * @brief  向SPI总线写多字节数据
  * @param  SPIx 需要使用的SPI
  * @param  p_TxData 发送数据缓冲区首地址
  * @param	sendDataNum 发送数据字节数
  * @retval 数据发送状态
  *		@arg 0 数据发送成功
  * 	@arg -1 数据发送失败
  */
int32_t SPI_WriteNBytes(SPI_TypeDef* SPIx, uint8_t *p_TxData,uint32_t sendDataNum)
{
	uint8_t retry=0;
	while(sendDataNum--){
		while((SPIx->SR&SPI_I2S_FLAG_TXE)==0);				//等待发送区空	
		{
			retry++;
			if(retry>200)return -1;
		}			  
		SPIx->DR=*p_TxData++;	 	  				//发送一个byte 
		retry=0;
		while((SPIx->SR&SPI_I2S_FLAG_RXNE)==0); 				//等待接收完一个byte  
		{
			SPIx->SR = SPIx->SR;
			retry++;
			if(retry>200)return -1;
		} 
		SPIx->DR;
	}
	return 0;
}
/**
  * @brief  从SPI总线读取多字节数据
  * @param  SPIx 需要使用的SPI
  * @param  p_RxData 数据储存地址
  * @param	readDataNum 读取数据字节数
  * @retval 数据读取状态
  *		@arg 0 数据读取成功
  * 	@arg -1 数据读取失败
  */
int32_t SPI_ReadNBytes(SPI_TypeDef* SPIx, uint8_t *p_RxData,uint32_t readDataNum)
{
	uint8_t retry=0;
	while(readDataNum--){
		SPIx->DR = 0xFF;
		while(!(SPIx->SR&SPI_I2S_FLAG_TXE)){
			retry++;
			if(retry>200)return -1;
		}
		retry = 0;
		while(!(SPIx->SR&SPI_I2S_FLAG_RXNE)){
			retry++;
			if(retry>200)return -1;
		}
		*p_RxData++ = SPIx->DR;
	}	
	return 0;
}

/*********************************END OF FILE**********************************/

