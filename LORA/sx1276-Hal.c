/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file	   sx1276-Hal.c
 * \brief	  SX1276 Hardware Abstraction Layer
 *
 * \version	2.0.B2 
 * \date	   Nov 21 2012
 * \author	 Miguel Luis
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#include <stdint.h>
#include <stdbool.h> 

#include "platform.h"

#if defined( USE_SX1276_RADIO )

#include "ioe.h"
//#include "spi.h"
#include "stm32f10x_gpio.h"		// порты ввода-вывода
#include "stm32f10x_rcc.h"		// ситстема тактирования
#include "stm32f10x_spi.h"		// SPI

#include "sx1276-Hal.h"

/*!
 * SX1276 RESET I/O definitions
 */
#if defined( STM32F4XX ) || defined( STM32F2XX )
	#define RESET_IOPORT							GPIOG
	#define RESET_PIN								GPIO_Pin_12
#else
	#define RESET_IOPORT							GPIOB
	#define RESET_PIN								GPIO_Pin_0
#endif

/*!
 * SX1276 SPI NSS I/O definitions
 */
#if defined( STM32F4XX ) || defined( STM32F2XX )
	#define NSS_IOPORT								GPIOA
	#define NSS_PIN									GPIO_Pin_15
#else
	#define NSS_IOPORT								GPIOA
	#define NSS_PIN									GPIO_Pin_4
#endif

/*!
 * SX1276 DIO pins  I/O definitions
 */
#if defined( STM32F4XX ) || defined( STM32F2XX )
	#define DIO0_IOPORT								GPIOG
	#define DIO0_PIN								GPIO_Pin_13
#else
	#define DIO0_IOPORT								GPIOB
	#define DIO0_PIN								GPIO_Pin_5
#endif

#if defined( STM32F4XX ) || defined( STM32F2XX )
	#define DIO1_IOPORT								GPIOB
	#define DIO1_PIN								GPIO_Pin_8
#else
	#define DIO1_IOPORT								GPIOB
	#define DIO1_PIN								GPIO_Pin_0
#endif

#if defined( STM32F4XX ) || defined( STM32F2XX )
	#define DIO2_IOPORT								GPIOA
	#define DIO2_PIN								GPIO_Pin_2
#else
	#define DIO2_IOPORT								GPIOC
	#define DIO2_PIN								GPIO_Pin_5
#endif

#if defined( STM32F4XX ) || defined( STM32F2XX )
	#define DIO3_IOPORT
	#define DIO3_PIN								RF_DIO3_PIN
#else
	#define DIO3_IOPORT								GPIOC
	#define DIO3_PIN								RF_DIO3_PIN
#endif

#if defined( STM32F4XX ) || defined( STM32F2XX )
	#define DIO4_IOPORT
	#define DIO4_PIN								RF_DIO4_PIN
#else
	#define DIO4_IOPORT 							GPIOC
	#define DIO4_PIN								RF_DIO4_PIN
#endif

#if defined( STM32F4XX ) || defined( STM32F2XX )
	#define DIO5_IOPORT
	#define DIO5_PIN								RF_DIO5_PIN
#else
	#define DIO5_IOPORT								GPIOB
	#define DIO5_PIN								GPIO_Pin_1
#endif

#if defined( STM32F4XX ) || defined( STM32F2XX )
	#define RXTX_IOPORT
	#define RXTX_PIN								FEM_CTX_PIN
#else
	#define RXTX_IOPORT
	#define RXTX_PIN								FEM_CTX_PIN
#endif


void SX1276InitIo( void )
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//
//#if defined( STM32F4XX ) || defined( STM32F2XX )
//	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOG, ENABLE );
//#else
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
//#endif
//
//#if defined( STM32F4XX ) || defined( STM32F2XX )
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//#else
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//#endif
//
//	// Configure NSS as output
//	GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
//	GPIO_InitStructure.GPIO_Pin = NSS_PIN;
//	GPIO_Init( NSS_IOPORT, &GPIO_InitStructure );
//
//	// Configure radio DIO as inputs
//#if defined( STM32F4XX ) || defined( STM32F2XX )
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//#else
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//#endif
//
//	// Configure DIO0
//	GPIO_InitStructure.GPIO_Pin =  DIO0_PIN;
//	GPIO_Init( DIO0_IOPORT, &GPIO_InitStructure );
//
//	// Configure DIO1
//	GPIO_InitStructure.GPIO_Pin =  DIO1_PIN;
//	GPIO_Init( DIO1_IOPORT, &GPIO_InitStructure );
//
//	// Configure DIO2
//	GPIO_InitStructure.GPIO_Pin =  DIO2_PIN;
//	GPIO_Init( DIO2_IOPORT, &GPIO_InitStructure );
//
//	// REAMARK: DIO3/4/5 configured are connected to IO expander
//
//	// Configure DIO3 as input
//
//	// Configure DIO4 as input
//
//	// Configure DIO5 as input
}

void SX1276SetReset( uint8_t state )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	if( state == RADIO_RESET_ON )
	{
		// Set RESET pin to 0
		GPIO_WriteBit( RESET_IOPORT, RESET_PIN, Bit_RESET );
		//GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	}
	else
	{	// Set RESET pin to 1
		GPIO_WriteBit( RESET_IOPORT, RESET_PIN, Bit_SET );
	}
}

void SX1276Write( uint8_t addr, uint8_t data )
{
	SX1276WriteBuffer( addr, &data, 1 );
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
	SX1276ReadBuffer( addr, data, 1 );
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
	u8 i;
	GPIO_ResetBits(GPIOA, GPIO_Pin_4); // NSS в 0

	SPI_I2S_SendData(SPI1, addr | 0x80);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ждем пока освободится буфер передачи

	for( i = 0; i < size; i++ )
		{
		SPI_I2S_SendData(SPI1, buffer [i]);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ждем пока освободится буфер передачи
		}
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);

	GPIO_SetBits(GPIOA, GPIO_Pin_4);  // NSS в 1
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
	u8 i ;
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);

	//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ждем пока освободится буфер передачи
	SPI_I2S_SendData(SPI1, addr & 0x7F);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
	//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ждем пока освободится буфер передачи
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 		// ждем пока байт прочитается целиком
	SPI_I2S_ReceiveData (SPI1); // читаем байт для сброса флага чтения

	for( i = 0; i < size; i++ )
		{
		//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ждем пока освободится буфер передачи
		SPI_I2S_SendData(SPI1, 0x00);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // ждем пока освободится буфер передачи
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); // ждем пока байт прочитается целиком
		buffer [i] = SPI_I2S_ReceiveData (SPI1);
		}

	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
	SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
	SX1276ReadBuffer( 0, buffer, size );
}

inline uint8_t SX1276ReadDio0( void )
{
	return GPIO_ReadInputDataBit( DIO0_IOPORT, DIO0_PIN );
}

inline uint8_t SX1276ReadDio1( void )
{
	return GPIO_ReadInputDataBit( DIO1_IOPORT, DIO1_PIN );
}

inline uint8_t SX1276ReadDio2( void )
{
	return GPIO_ReadInputDataBit( DIO2_IOPORT, DIO2_PIN );
}

inline uint8_t SX1276ReadDio3( void )
{
	return GPIO_ReadInputDataBit( DIO3_IOPORT, DIO3_PIN );
	//return IoePinGet( RF_DIO3_PIN );
}

inline uint8_t SX1276ReadDio4( void )
{
	return GPIO_ReadInputDataBit( DIO4_IOPORT, DIO4_PIN );
	//return IoePinGet( RF_DIO4_PIN );
}

inline uint8_t SX1276ReadDio5( void )
{
	return GPIO_ReadInputDataBit( DIO5_IOPORT, DIO5_PIN );
	//return IoePinGet( RF_DIO5_PIN );
}

inline void SX1276WriteRxTx( uint8_t txEnable )
{
	if( txEnable != 0 )
	{
//		IoePinOn( FEM_CTX_PIN );
//		IoePinOff( FEM_CPS_PIN );
	}
	else
	{
//		IoePinOff( FEM_CTX_PIN );
//		IoePinOn( FEM_CPS_PIN );
	}
}

#endif // USE_SX1276_RADIO
