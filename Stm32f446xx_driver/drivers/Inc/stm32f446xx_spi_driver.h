/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 2020/02/17
 *      Author: $NghiaPham$
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/									BIT POSITION DEFINITIONS OF SPI PERIPHERAL									_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     					 0
#define SPI_CR1_CPOL      					 1
#define SPI_CR1_MSTR     					 2
#define SPI_CR1_BR   						 3
#define SPI_CR1_SPE     					 6
#define SPI_CR1_LSBFIRST   			 		 7
#define SPI_CR1_SSI     					 8
#define SPI_CR1_SSM      					 9
#define SPI_CR1_RXONLY      		 		10
#define SPI_CR1_DFF     			 		11
#define SPI_CR1_CRCNEXT   			 		12
#define SPI_CR1_CRCEN   			 		13
#define SPI_CR1_BIDIOE     			 		14
#define SPI_CR1_BIDIMODE      				15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 				0
#define SPI_CR2_TXDMAEN				 		1
#define SPI_CR2_SSOE				 		2
#define SPI_CR2_FRF							4
#define SPI_CR2_ERRIE						5
#define SPI_CR2_RXNEIE				 		6
#define SPI_CR2_TXEIE						7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE							0
#define SPI_SR_TXE				 			1
#define SPI_SR_CHSIDE				 		2
#define SPI_SR_UDR					 		3
#define SPI_SR_CRCERR				 		4
#define SPI_SR_MODF					 		5
#define SPI_SR_OVR					 		6
#define SPI_SR_BSY					 		7
#define SPI_SR_FRE					 		8

/*
 * @SPI_DeviceMode
 * Master selection
 */
#define SPI_DEVICE_MODE_SLAVE				0				/*! Slave configuration 					*/
#define SPI_DEVICE_MODE_MASTER				1				/*! Master configuration 					*/

/*
 * @SPI_BusConfig
 * Bus selection
 */
#define SPI_BUS_CONFIG_FD					1				/*! Full-duplex configuration 				*/
#define SPI_BUS_CONFIG_HD					2				/*! Haft-duplex configuration 				*/
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3				/*! Simplex receive configuration 			*/

/*
 * @SPI_SclkSpeed
 * Baud rate control
 */
#define SPI_SCLK_SPEED_DIV2					0				/*! fPCLK/2 								*/
#define SPI_SCLK_SPEED_DIV4					1				/*! fPCLK/4 								*/
#define SPI_SCLK_SPEED_DIV8					2				/*! fPCLK/8									*/
#define SPI_SCLK_SPEED_DIV16				3				/*! fPCLK/16 								*/
#define SPI_SCLK_SPEED_DIV32				4				/*! fPCLK/32 								*/
#define SPI_SCLK_SPEED_DIV64				5				/*! fPCLK/64 								*/
#define SPI_SCLK_SPEED_DIV128				6				/*! fPCLK/128 								*/
#define SPI_SCLK_SPEED_DIV256				7				/*! fPCLK/256 								*/

/*
 * @SPI_DFF
 * Data frame format
 */
#define SPI_DFF_8BITS						0				/*! 8-bit data frame 						*/
#define SPI_DFF_16BITS						1				/*! 16-bit data frame 						*/

/*
 * @SPI_CPHA
 * Clock phase
 */
#define SPI_CPHA_LOW						0				/*! The first clock transition				*/
#define SPI_CPHA_HIGH						1				/*! The second clock transition				*/

/*
 * @SPI_CPOL
 * Clock polarity
 */
#define SPI_CPOL_LOW						0				/*! CK to 0 when idle 						*/
#define SPI_CPOL_HIGH						1				/*! CK to 1 when idle 						*/

/*
 * @SPI_SSM
 * Software slave management
 */
#define SPI_SSM_DI							0				/*! Software slave management disabled 		*/
#define SPI_SSM_EN							1				/*! Software slave management enable 		*/

/*
 * SPI related status flags definitions in SR register
 */
#define SPI_TXE_FLAG						(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG						(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG						(1 << SPI_SR_BSY)

/*
 * Possible SPI application state
 */
#define SPI_READY							0
#define SPI_BUSY_IN_RX						1
#define SPI_BUSY_IN_TX						2

/*
 * Possible SPI application events
 */
#define SPI_EVENT_TX_CMPLT					1
#define SPI_EVENT_RX_CMPLT					2
#define SPI_EVENT_OVR_ERR					3
#define SPI_EVENT_CRC_ERR					3

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * This is handle structure for a SPIx pin
 */
typedef struct {
	SPI_RegDef_t 		*pSPIx;
	SPI_Config_t	 	 SPIConfig;
	uint8_t				*pTxBuffer;							/*! To store the app. Tx buffer address		*/
	uint8_t				*pRxBuffer;							/*! To store the app. Rx buffer address		*/
	uint32_t			 TxLen;								/*! To store Tx Length						*/
	uint32_t			 RxLen;								/*! To store Rx Length						*/
	uint8_t				 TxState;							/*! To store Tx State						*/
	uint8_t				 RxState;							/*! To store Rx State						*/
}SPI_Handle_t;

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/												APIs SUPPORT BY THIS DRIVERS									_/
//_/							FOR MORE INFORMATION ABOUT THE APIs CHECK THE FUCTION DEFINITIONs					_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

//------------------------------------------------------------------
// PERIPHERAL CLOCK SETUP
//------------------------------------------------------------------
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//------------------------------------------------------------------
// INITIALIZE AND DEINITIALIZE
//------------------------------------------------------------------
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//------------------------------------------------------------------
// DATA TRANSMIT AND RECEIVE
//------------------------------------------------------------------
void SPI_TransmitData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Lenghth);
uint8_t SPI_TransmitDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Lenghth);

//------------------------------------------------------------------
// IRQ CONFIGUARATION AND ISR HANDLING
//------------------------------------------------------------------
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

//------------------------------------------------------------------
// OTHER PERIPHERAL CONTROL
//------------------------------------------------------------------
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

//------------------------------------------------------------------
// APPLICATION CALLBACK
//------------------------------------------------------------------
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
