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

extern void initialise_monitor_handles(void);

#define MY_ADDR 						0x61
#define SLAVE_ADDR  					0x68

I2C_Handle_t I2C1Handle;
uint8_t rxComplt = RESET;				//Flag variable
uint8_t receiveBuff[128];				//Buffer

void delay250ms(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GPIOBtn;
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	GPIOBtn.pGPIOx = GPIOC;

	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;

	GPIO_Init(&GPIOBtn);
}

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
	initialise_monitor_handles();
	printf("Application will be start\n");
	uint8_t cmdCode, len;

	GPIO_ButtonInit();

	//I2C pin initialize
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//I2C IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	//Enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ACK bit is made 1 after PE=1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1)
	{
		//Wait until button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//To avoid button de-bouncing related issues 250ms of delay
		delay250ms();

		//Send command to read the 1 byte length information from slave
		cmdCode = 0x51;
		while(I2C_MasterSendDataIT(&I2C1Handle, &cmdCode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);
		while(I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

		//Send command get information from slave
		cmdCode = 0x52;
		while(I2C_MasterSendDataIT(&I2C1Handle, &cmdCode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);
		while(I2C_MasterReceiveDataIT(&I2C1Handle, receiveBuff, len, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY);

		rxComplt = RESET;
		while (rxComplt != SET);

		receiveBuff[len+1] = '\0';
		printf("Data : %s\n", receiveBuff);
		rxComplt = RESET;
	}
}


void I2C1_EV_IRQHandler(void) {
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void) {
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {

    if (AppEv == I2C_EV_TX_CMPLT) {
    	printf("Transmit is completed\n");
    }
    else if (AppEv == I2C_EV_RX_CMPLT) {
    	printf("Receive is completed\n");
    	rxComplt = SET;
    }
    else if (AppEv == I2C_ERROR_AF) {

    	printf("Error : ACK failure\n");
    	//In master ACK failure happens when slave fails to send ACK for the byte
    	//sent from the master.
    	I2C_CloseSendData(pI2CHandle);

    	//generate the stop condition to release the bus
    	I2C_GenerateStopCondition(I2C1);

    	//Hang in infinite loop
    	while(1);
    }
}
