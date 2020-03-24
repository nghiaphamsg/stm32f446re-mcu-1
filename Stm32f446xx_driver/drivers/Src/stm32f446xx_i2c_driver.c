/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Mar 7, 2020
 *      Author: AD
 */

#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_rcc_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START );
}


static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;					//shift SlaveAddr left by 1 position. SlaveAddr now is at [7:1].
	SlaveAddr &= ~(1);							//for WRITE, clear 0th bit
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;					//shift SlaveAddr left by 1 position. SlaveAddr now is at [7:1].
	SlaveAddr |= 1;								//for READ set 0th bit
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
	uint32_t dummyRead;

	if (pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL )) {
		//master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if (pI2CHandle->RxSize == 1) {
				//Disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//Clear the ADDR flag
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
			else {
				//Clear the ADDR flag
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}
	}
	else {
		//slave mode
		//Clear the ADDR flag
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle) {
	if(pI2CHandle->TxLen > 0) {
		//1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle) {
	if (pI2CHandle->RxSize == 1) {
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}
	if (pI2CHandle->RxSize > 1) {
		if (pI2CHandle->RxLen == 2) {
			//clear the ACK bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}
		//Read DR
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}
	if (pI2CHandle->RxLen == 0) {
		//Close the I2C data reception and notify the application
		//1. generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2 . Close the I2C RX
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

/*********************************************************************************
 * @fn				- I2C_PeriClockControl
 * @brief			- This function disable or enable peripheral clock for the give I2C port
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @return			- None
 * @note			- None
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE){
		if (pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

/********************************************************************************************************************
 * @fn				- I2C_Init
 * @brief			- This function initialize the give I2C port and give I2C pin
 * @param[in]		- Handle structure for a I2C pin
 * @return			- None
 * @note			- None
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t temp = 0;

	//Enable CLK
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//Configure Acknowledge enable
	temp |= (pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = temp;

	//Peripheral clock frequency (FREQ)
	temp = 0;
	temp |= RCC_Get2PCLKValue()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (temp & 0x3F);

	//Interface address
	temp = 0;
	temp |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD71);
	temp |=  ( 1 << 14 );
	pI2CHandle->pI2Cx->OAR1 = temp;

	//Calculate CCR
	temp = 0;
	uint16_t ccrConfig = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// Standard mode
		ccrConfig = (RCC_Get2PCLKValue()/(2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		temp |= (ccrConfig & 0xFFF);
	}
	else {
		// Fast mode
		temp |= (1 << I2C_CCR_FS);
		temp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccrConfig = (RCC_Get2PCLKValue()/(3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else {
			ccrConfig = (RCC_Get2PCLKValue()/(25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		temp |= (ccrConfig & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = temp;

	//TRISE configuration
	temp = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// Standard mode
		temp = (RCC_Get2PCLKValue()/1000000U) + 1;
	}
	else {
		// Fast mode
		temp = ((RCC_Get2PCLKValue()*300 )/1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (temp & 0x3F);
}

 /*******************************************************************************************************************
  * @fn				- I2C_DeInit
  * @brief			- This function de-initialize the give I2C port and give I2C pin
  * @param[in]		- Base address of the I2C peripheral
  * @return			- None
  * @note			- Use RCC APB1 peripheral reset register (RCC_APB1RSTR)
  */
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}

/********************************************************************************************************************
 * @fn				- I2C_MasterSendData
 * @brief			- This function send data
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- buffer when transmit data
 * @param[in]		- Data length
 * @param[in]		- Slave address
 * @return			- None
 * @note			- Call this as blocking API, because the function call will wait until all the bytes are transmitted.
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr) {

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generate is completed by checking the SB flag in the SR1
	// Note:(EV5) Until SB is cleared SCL will be stretched (Pulled to LOW)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/w bit set to w(0) (total 8bits) (EV5)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that start generate is completed by checking the ADDR flag in the SR1 (EV6)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according to its software sequence
	//Note:(EV6) Until ADDR is cleared SCL will be stretched (Pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send the data until length becomes 0 (EV8_1/EV8)
	while(Length > 0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Length--;
	}

	//7. When Length becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//when BTF=1 SCL will be stretched (pulled to LOW) (EV8_2)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//Note:(EV8_2) generating STOP, automatically clears the BTF
	if (Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

/********************************************************************************************************************
 * @fn				- I2C_MasterReceiveData
 * @brief			- This function receive data
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- buffer when transmit data
 * @param[in]		- Data length
 * @param[in]		- Slave address
 * @return			- None
 * @note			- Call this as blocking API, because the function call will wait until all the bytes are transmitted.
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr) {

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generate is completed by checking the SB flag in the SR1
	// Note:(EV5)Until SB is cleared SCL will be stretched (Pulled to LOW)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/w bit set to r(1) (total 8bits) (EV5)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that start generate is completed by checking the ADDR flag in the SR1 (EV6)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Procedure to read only 1 byte from slave
	if (Length == 1) {
		//Disable ACK
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//Clear the ADDR flag according to its software sequence
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait till RXE is set
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//Generate STOP condition and master need not to wait for the completion of stop condition.
		if (Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	//6. Procedure to read data from slave when Length > 1
	if (Length > 1) {
		//Clear the ADDR flag according to its software sequence
		I2C_ClearADDRFlag(pI2CHandle);

		//Read the data until Length becomes zero
		for (uint32_t i = Length; i > 0; i--)
		{
			//Wait till RXE is set
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			//if last 2 bytes are remaining
			if(i == 2) {
				//Disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				if (Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//Read data in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}

	//Re-Enable ACK
	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}

/********************************************************************************************************************
 * @fn				- I2C_SlaveSendData
 * @brief			- This function transfer data for slave transmitter
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- data when transmit
 * @return			- None
 * @note			- None
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data) {
	pI2C->DR = data;
}

/********************************************************************************************************************
 * @fn				- I2C_SlaveReceiveData
 * @brief			- This function transfer data for slave receiver
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- data when transmit
 * @return			- None
 * @note			- None
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C) {
	return (uint8_t) pI2C->DR;
}

/********************************************************************************************************************
 * @fn				- I2C_MasterSendDataIT
 * @brief			- This function send data and handler
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- buffer when transmit data
 * @param[in]		- Data length
 * @param[in]		- Slave address
 * @return			- None
 * @note			- Call this as blocking API, because the function call will wait until all the bytes are transmitted.
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer 	= pTxBuffer;
		pI2CHandle->TxLen 		= Length;
		pI2CHandle->TxRxState 	= I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr 	= SlaveAddr;
		pI2CHandle->Sr 			= Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}
	return busystate;
}

/********************************************************************************************************************
 * @fn				- I2C_MasterReceiveDataIT
 * @brief			- This function receive data and handler
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- buffer when transmit data
 * @param[in]		- Data length
 * @param[in]		- Slave address
 * @return			- None
 * @note			- Call this as blocking API, because the function call will wait until all the bytes are transmitted.
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr) {
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer 	= pRxBuffer;
		pI2CHandle->RxLen 		= Length;
		pI2CHandle->TxRxState 	= I2C_BUSY_IN_RX;
		pI2CHandle->RxSize 		= Length; 					//Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr 	= SlaveAddr;
		pI2CHandle->Sr 			= Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

/*******************************************************************************************************************
  * @fn				- I2C_IRQInterruptConfig
  * @brief			-
  * @param[in]		- IRQ number
  * @param[in]		- IRQ priority
  * @param[in]		- ENABLE or DISABLE macros
  * @return			- None
  * @note			- None
  */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	 if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ISER1 |= ( 1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= ( 1 << IRQNumber % 64);
		}
	 } else {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 |= ( 1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= ( 1 << IRQNumber % 64);
		}
	}
}

/*******************************************************************************************************************
 * @fn			- I2C_IRQPriorityConfig
 * @brief		-
 * @param[in]	- IRQ number
 * @param[in]	- IRQ priority
 * @return		- None
 * @note		- None
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	 uint8_t iprx, iprxSection, shiftAmount;
	 iprx = IRQNumber / 4;
	 iprxSection = IRQNumber % 4;
	 shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);
	 *(NVIC_PR_BASE_ADDR + iprx ) = (IRQPriority << shiftAmount);
}

/*******************************************************************************************************************
  * @fn			- I2C_EV_IRQHandling
  * @brief		- Interrupt handling for interrupts generated by I2C events
  * @param[in]	- Base address of the I2C peripheral
  * @return		- None
  * @note		- None
  */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN );
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN );
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB );

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if (temp1 && temp3) {
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	//2. Handle for interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR );
	if (temp1 && temp3) {
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//3. Handle for interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF	);
	if (temp1 && temp3) {
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			//make sure that TXE is also set .
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE) ) {
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0 ) {
					//1. Generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. Reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. Notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX ) {
			;
		}
	}

	//4. Handle for interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF );
	if (temp1 && temp3) {
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle for interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE	);
	if (temp1 && temp2 && temp3) {
		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL)) {
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else {
			//Slave mode
			//Make sure that the slave is really in transmitter mode
		    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)) {
		    	I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		    }
		}
	}

	//6. Handle for interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE );
	if (temp1 && temp2 && temp3) {
		//Check device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL)) {
			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else {
			//Slave mode
			//Make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/*******************************************************************************************************************
    * @fn			- I2C_ER_IRQHandling
    * @brief		- Interrupt handling for interrupts generated by I2C errors
    * @param[in]	- Base address of the I2C peripheral
    * @return		- None
    * @note			- None
    */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {
	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);

/**************************Check for Bus error***************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 ) {
		//This is Bus error
		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error****************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2) {
		//This is arbitration lost error
		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error********************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2) {
		//This is ACK failure error
	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error****************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2) {
		//This is Overrun/Underrun
	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

/*************************Check for Time out error**********************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2) {
		//This is Time out error
	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

/********************************************************************************************************************
 * @fn				- I2C_PeripheralControl
 * @brief			- Enable or Disable PE bit in the CR1 register
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- Enable or Disable
 * @return			- None
 * @note			- None
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/********************************************************************************************************************
 * @fn				- I2C_ManageAcking
 * @brief			- Enable or Disable ACK
 * @param[in]		- Base address of the I2C peripheral
 * @return			- None
 * @note			- None
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == I2C_ACK_ENABLE)
	{
		//Enable the ACK
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}
	else {
		//Disable the ACK
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}

/********************************************************************************************************************
 * @fn				- I2C_CloseReceiveData
 * @brief			- This function close receive data
 * @param[in]		- Base address of the I2C peripheral
 * @return			- None
 * @note			- None
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}

}

/********************************************************************************************************************
 * @fn				- I2C_CloseSendData
 * @brief			- This function close send data
 * @param[in]		- Base address of the I2C peripheral
 * @return			- None
 * @note			- None
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/********************************************************************************************************************
 * @fn				- I2C_GenerateStopCondition
 * @brief			- This function generate stop condition
 * @param[in]		- Base address of the I2C peripheral
 * @return			- None
 * @note			- None
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP );
}

/********************************************************************************************************************
 * @fn				- I2C_GetFlapStatus
 * @brief			- This function get flap status
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- flap name
 * @return			- return status register
 * @note			- None
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
	if (pI2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/********************************************************************************************************************
 * @fn				- I2C_SlaveEnableDisableCallbackEvents
 * @brief			- This function enable or disable call back function
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- Enable or Disable
 * @return			- None
 * @note			- None
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	else {
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}

/********************************************************************************************************************
 * @fn				- I2C_ApplicationEventCallback
 * @brief			- Event and Error
 * @param[in]		- Base address of the I2C peripheral
 * @return			- None
 * @note			- None
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pSPIHandle,uint8_t AppEv) {
	//This is a weak implementation. The user application may override this function.
}
