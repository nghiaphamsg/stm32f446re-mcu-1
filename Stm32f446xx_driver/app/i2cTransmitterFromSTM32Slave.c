/*
 * i2cReceiveFromAdruinoSlave.c
 *
 *  Created on: 2020/03/11
 *      Author: 9004060731
 */

/*********************************************************************************************************************
 ************************************************** PIN CONFIGURE ****************************************************
 *********************************************************************************************************************
 * User Button 		PC13
 *
 * Master (STM32F446xx)
 * PB6		>>		I2C1_SCL
 * PB7		>>		I2C1_SDA
 * Alternate function: 4
 *
 * Slave (Arduino Uno R3)
 * Pin A5	>>		SCL
 * Pin A4	>>		SDA
 * *******************************************************************************************************************
 * *******************************************************************************************************************
 */
#include <string.h>

#include "stm32f446xx.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_gpio_driver.h"

#define MY_ADDR 						0x68
#define SLAVE_ADDR  					0x68

uint32_t data_len = 0;
uint8_t Tx_buf[]  = "STM32 Slave mode testing--abcdefghijkmnlopqrsxzw--123456789-10-11-12-13-14-15-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//////////////////////////////////////////***********************************************************************************************";
I2C_Handle_t I2C1Handle;
uint8_t CommandCode;


void I2C1_GPIOInits(void) {
	GPIO_Handle_t I2CPins;
	memset(&I2CPins, 0, sizeof(I2CPins));

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode 	= 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;

	I2CPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_6;					//I2C1_SCL
	GPIO_Init(&I2CPins);

	I2CPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_7;					//I2C1_SDA
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void) {

//	memset(&I2C1Handle, 0, sizeof(I2C1Handle));

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl 	= I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle 	= I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed 		= I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}


int main(void) {

	data_len = strlen((char*)Tx_buf);
	//I2C pin initialize
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//I2C IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	//Enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ACK bit is made 1 after PE=1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1);
}


void I2C1_EV_IRQHandler(void) {
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void) {
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {

	static uint32_t cnt = 0;
	static uint32_t w_ptr = 0;

	if(AppEv == I2C_ERROR_AF) {
		//This will happen during slave transmitting data to master .
		// slave should understand master needs no more data
		//slave concludes end of Tx


		//if the current active code is 0x52 then dont invalidate
		if(! (CommandCode == 0x52))
			CommandCode = 0XFF;

		//reset the cnt variable because its end of transmission
		cnt = 0;

		//Slave concludes it sent all the bytes when w_ptr reaches data_len
		if(w_ptr >= (data_len)) {
			w_ptr=0;
			CommandCode = 0xff;
		}
	}
	else if (AppEv == I2C_EV_STOP) {
				//This will happen during end slave reception
				//slave concludes end of Rx
				cnt = 0;
	}
	else if (AppEv == I2C_EV_DATA_REQ) {
		//Master wants some data. slave has to send it
		if(CommandCode == 0x51) {
			//send the length information to the master
			I2C_SlaveSendData(I2C1, ((data_len >> ((cnt%4) * 8)) & 0xFF));
			cnt++;
		}
		else if (CommandCode == 0x52) {
			//Send the contents of Tx_buf
			I2C_SlaveSendData(I2C1, Tx_buf[w_ptr++]);
		}
	}
	else if (AppEv == I2C_EV_DATA_RCV) {
		//Data is waiting for the slave to read . slave has to read it
		CommandCode = I2C_SlaveReceiveData(I2C1);

	}
}

