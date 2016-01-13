/**
  ******************************************************************************
  * @file    spi_driver.c
  * $Author: wdluo $
  * $Revision: 17 $
  * $Date:: 2012-07-06 11:16:48 +0800 #$
  * @brief   SPI�ײ���������.
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
  * @brief  ʹ��SPIʱ��
  * @param  SPIx ��Ҫʹ�õ�SPI
  * @retval None
  */
static void SPI_RCC_Configuration(SPI_TypeDef* SPIx ,u8 Remap)
{
	if(Remap==0)    //SPIû����ӳ��
	{
		if(SPIx==SPI1){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1,ENABLE);
	}else{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	}
	}
	
	if(Remap==1)   //SPI����ӳ��
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
  * @brief  ����ָ��SPI������
  * @param  SPIx ��Ҫʹ�õ�SPI
  * @retval None
  */
static void SPI_GPIO_Configuration(SPI_TypeDef* SPIx , u8 Remap )
{
	GPIO_InitTypeDef GPIO_InitStruct;
		if(Remap==0)   //SPIû����ӳ��
		{
			if(SPIx==SPI1){					 					 
		GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6|GPIO_Pin_7;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 
		GPIO_Init(GPIOA, &GPIO_InitStruct);
		//��ʼ��Ƭѡ�������
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStruct);
		GPIO_SetBits(GPIOA,GPIO_Pin_4);
	}else{
		GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14|GPIO_Pin_15;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		//��ʼ��Ƭѡ�������
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
	} 
	}
	
		if(Remap==1)    //SPI����ӳ��
		{
			GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);   //SPI1��ӳ��ʹ��
			if(SPIx==SPI1){					 					 
		GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4|GPIO_Pin_5;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		//��ʼ��Ƭѡ�������
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStruct);
		GPIO_SetBits(GPIOA,GPIO_Pin_15);
	}else{
		GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14|GPIO_Pin_15;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; 
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		//��ʼ��Ƭѡ�������
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
	} 
	}
	
}
/**
  * @brief  �����ⲿSPI�豸����SPI��ز���
  * @param  SPIx ��Ҫʹ�õ�SPI
  * @retval None
  */
void SPI_Configuration(SPI_TypeDef* SPIx)
{
	SPI_InitTypeDef SPI_InitStruct;

	SPI_RCC_Configuration(SPIx,0);

	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;   //������Ԥ��ƵֵΪ32  Chain��ʾ�˴�ע�⣬ͨ��ʱ������SPI��ʱ�ӷ�����ã�����Ҫ���ô�SPI��ʱ��
	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;    //˫��˫��ȫ˫��
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;    //��SPI�豸
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;  //SPI���ݴ�С	
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;   //ʱ�����յ�
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;   //���ݲ������һ��ʱ����
	SPI_InitStruct.SPI_NSS = SPI_NSS_Hard;   //NSS���ⲿ�ܽŹ���
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;   //���ݴ����MSB��ʼ
	SPI_InitStruct.SPI_CRCPolynomial = 7;   //��������CRCֵ����Ķ���ʽ
	SPI_Init(SPIx, &SPI_InitStruct);    //��������SPI������
	
	SPI_GPIO_Configuration(SPIx,0);

	SPI_SSOutputCmd(SPIx, ENABLE);   //ʹ��ָ����SPI SS���
	SPI_Cmd(SPIx, ENABLE);   //ʹ��SPI����
}
/**
  * @brief  д1�ֽ����ݵ�SPI����
  * @param  SPIx ��Ҫʹ�õ�SPI
  * @param  TxData д�����ߵ�����
  * @retval ���ݷ���״̬
  *		@arg 0 ���ݷ��ͳɹ�
  * 	@arg -1 ���ݷ���ʧ��
  */
int32_t SPI_WriteByte(SPI_TypeDef* SPIx, uint16_t TxData)
{
	uint8_t retry=0;				 
	while((SPIx->SR&SPI_I2S_FLAG_TXE)==0);				//�ȴ���������	
	{
		retry++;
		if(retry>200)return -1;
	}			  
	SPIx->DR=TxData;	 	  				//����һ��byte 
	retry=0;
	while((SPIx->SR&SPI_I2S_FLAG_RXNE)==0); 				//�ȴ�������һ��byte  
	{
		retry++;
		if(retry>200)return -1;
	}  
	SPIx->DR;						    
	return 0;          				//�����յ�������
}
/**
  * @brief  ��SPI���߶�ȡ1�ֽ�����
  * @param  SPIx ��Ҫʹ�õ�SPI
  * @param  p_RxData ���ݴ����ַ
  * @retval ���ݶ�ȡ״̬
  *		@arg 0 ���ݶ�ȡ�ɹ�
  * 	@arg -1 ���ݶ�ȡʧ��
  */
int32_t SPI_ReadByte(SPI_TypeDef* SPIx, uint16_t *p_RxData)
{
	uint8_t retry=0;				 
	while((SPIx->SR&SPI_I2S_FLAG_TXE)==0);				//�ȴ���������	
	{
		retry++;
		if(retry>200)return -1;
	}			  
	SPIx->DR=0xFF;	 	  				//����һ��byte 
	retry=0;
	while((SPIx->SR&SPI_I2S_FLAG_RXNE)==0); 				//�ȴ�������һ��byte  
	{
		retry++;
		if(retry>200)return -1;
	}
	*p_RxData = SPIx->DR;  						    
	return 0;          				//�����յ�������
}
/**
  * @brief  ��SPI����д���ֽ�����
  * @param  SPIx ��Ҫʹ�õ�SPI
  * @param  p_TxData �������ݻ������׵�ַ
  * @param	sendDataNum ���������ֽ���
  * @retval ���ݷ���״̬
  *		@arg 0 ���ݷ��ͳɹ�
  * 	@arg -1 ���ݷ���ʧ��
  */
int32_t SPI_WriteNBytes(SPI_TypeDef* SPIx, uint8_t *p_TxData,uint32_t sendDataNum)
{
	uint8_t retry=0;
	while(sendDataNum--){
		while((SPIx->SR&SPI_I2S_FLAG_TXE)==0);				//�ȴ���������	
		{
			retry++;
			if(retry>200)return -1;
		}			  
		SPIx->DR=*p_TxData++;	 	  				//����һ��byte 
		retry=0;
		while((SPIx->SR&SPI_I2S_FLAG_RXNE)==0); 				//�ȴ�������һ��byte  
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
  * @brief  ��SPI���߶�ȡ���ֽ�����
  * @param  SPIx ��Ҫʹ�õ�SPI
  * @param  p_RxData ���ݴ����ַ
  * @param	readDataNum ��ȡ�����ֽ���
  * @retval ���ݶ�ȡ״̬
  *		@arg 0 ���ݶ�ȡ�ɹ�
  * 	@arg -1 ���ݶ�ȡʧ��
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
