/*
 * PCA9685.c
 *
 *  Created on: 10 abr. 2019
 *      Author: TOSHIBA
 */
#include "PCA9685.h"


void write_register(void *hi2c, uint8_t address,uint8_t register_pointer, uint16_t register_value)
{
    uint8_t data[3];

    data[0] = register_pointer;     // 0x0C in your example
    data[1] = register_value>>8;    // MSB byte of 16bit data
    data[2] = register_value;       // LSB byte of 16bit data

    HAL_I2C_Master_TransmitH(hi2c, 0xA0, data, 3, 100);  // data is the start pointer of our array
}

void read_register(void *hi2c, uint8_t address,uint8_t register_pointer, uint8_t* receive_buffer)
{
	//I2C_HandleTypeDef
    // first set the register pointer to the register wanted to be read
    HAL_I2C_Master_TransmitH(hi2c, 0xA0, &register_pointer, 1, 100);  // note the & operator which gives us the address of the register_pointer variable

    // receive the 2 x 8bit data into the receive buffer
    HAL_I2C_Master_ReceiveH(hi2c, 0xA0, receive_buffer, 2, 100);
}

void pca9685_init(void *hi2c, uint8_t address)
{

	uint8_t initStruct[2];
	uint8_t mode1_k = 1;

	HAL_GPIO_WritePinH(GPIO_PORT_H,GPIO_PIN_ADDR0_H|GPIO_PIN_ADDR1_H,GPIO_PIN_RESET);
	HAL_GPIO_WritePinH(GPIO_PORT_H,GPIO_PIN_HOLD_H,GPIO_PIN_RESET);
	HAL_GPIO_WritePinH(GPIO_PORT_H,GPIO_PIN_OE_H,GPIO_PIN_SET);

	set_all_pwm(hi2c,address,0,0);

	initStruct[0] = MODE2;
	initStruct[1] = OUTDRV;
	HAL_I2C_Master_TransmitH(hi2c, address, initStruct, 2, 1);

	initStruct[0] = MODE1;
	initStruct[1] = ALLCALL;
	HAL_I2C_Master_TransmitH(hi2c, address, initStruct, 2, 1);
	HAL_DelayH(20);
	HAL_I2C_Master_TransmitH(hi2c,address,initStruct,1,1);
	HAL_I2C_Master_ReceiveH(hi2c,address,&mode1_k,1,1);

	mode1_k = (mode1_k & (~SLEEP));
	initStruct[1] = mode1_k;
	HAL_I2C_Master_TransmitH(hi2c, address, initStruct, 2, 1);
	HAL_DelayH(20);
}

void set_all_pwm(void *hi2c,uint8_t address,uint8_t on, uint8_t off)
{
	uint8_t initStruct[2];

	initStruct[0] = ALL_LED_ON_L;
	initStruct[1] = (on & 0xFF);
	HAL_I2C_Master_TransmitH(hi2c, address, initStruct, 2, 1);
	initStruct[0] = ALL_LED_ON_H;
	initStruct[1] = (on >> 8);
	HAL_I2C_Master_TransmitH(hi2c, address, initStruct, 2, 1);
	initStruct[0] = ALL_LED_OFF_L;
	initStruct[1] = (off & 0xFF);
	HAL_I2C_Master_TransmitH(hi2c, address, initStruct, 2, 1);
	initStruct[0] = ALL_LED_OFF_H;
	initStruct[1] = (off >> 8);
	HAL_I2C_Master_TransmitH(hi2c, address, initStruct, 2, 1);
}

void pca9685_pwm(void *hi2c, uint8_t address, uint8_t num, uint16_t on, uint16_t off)
{
	uint8_t outputBuffer[2];

	outputBuffer[0] = (LED0_ON_L+4*num);
	outputBuffer[1] = (on & 0xFF);
	HAL_I2C_Master_TransmitH(hi2c, address, outputBuffer, 2, 1);
	outputBuffer[0] = (LED0_ON_H+4*num);
	outputBuffer[1] = (on >> 8);
	HAL_I2C_Master_TransmitH(hi2c, address, outputBuffer, 2, 1);

	outputBuffer[0] = (LED0_OFF_L+4*num);
	outputBuffer[1] = (off & 0xFF);
	HAL_I2C_Master_TransmitH(hi2c, address, outputBuffer, 2, 1);
	outputBuffer[0] = (LED0_OFF_H+4*num);
	outputBuffer[1] = (off >> 8);
	HAL_I2C_Master_TransmitH(hi2c, address, outputBuffer, 2, 1);
}

void set_pwm_freq(void *hi2c, uint8_t address,uint8_t freq)
{
	uint8_t presc = 0;
	uint8_t initStruct[2];
	uint8_t oldmode = 0;
	//uint8_t newmode =0;
	float prescaleval = 25000000.0;


	prescaleval /= 4096.0;
	prescaleval = (double) prescaleval/freq;
	prescaleval -= 1.0;
	prescaleval += 0.5;

	presc = (int)prescaleval;

	initStruct[0] = MODE1;
	initStruct[1] = 0;
	HAL_I2C_Master_TransmitH(hi2c,address,initStruct,1,1);
	HAL_I2C_Master_ReceiveH(hi2c,address,&oldmode,1,1);
	initStruct[1] = (oldmode & 0x7F) | 0x10;
	HAL_I2C_Master_TransmitH(hi2c, address, initStruct, 2, 1);

	initStruct[0] = PRESCALE;
	initStruct[1] = presc;
	HAL_I2C_Master_TransmitH(hi2c, address, initStruct, 2, 1);
	initStruct[0] = MODE1;
	initStruct[1] = oldmode;
	HAL_I2C_Master_TransmitH(hi2c, address, initStruct, 2, 1);
	HAL_DelayH(20);
	initStruct[1] = oldmode|0x80;//0x40
	HAL_I2C_Master_TransmitH(hi2c, address, initStruct, 2, 1);

}
//bloque de acoplamiento
void HAL_GPIO_WritePinH(void *GPIO,uint16_t PIN,int state)
{
	#ifdef HAL_DRIVER
		HAL_GPIO_WritePin(GPIO, PIN,state);//addr
	#else
		if(state == 0) 	LL_GPIO_ResetOutputPin(GPIO,PIN);
		else			LL_GPIO_SetOutputPin(GPIO,PIN);
	#endif
}
void HAL_I2C_Master_TransmitH(void *hi2c,uint16_t address,uint8_t *pData,uint16_t size,uint32_t timeout )
{
	#ifdef HAL_DRIVER
		HAL_I2C_Master_Transmit(hi2c,address,pData,size,timeout);//tst
	#else
		LL_I2C_HandleTransfer(hi2c,address,LL_I2C_ADDRSLAVE_7BIT,size,LL_I2C_MODE_AUTOEND,LL_I2C_GENERATE_START_WRITE);
		while(!LL_I2C_IsActiveFlag_STOP(hi2c))
		{
			if(LL_I2C_IsActiveFlag_TXIS(hi2c))
			{
				LL_I2C_TransmitData8(hi2c,*pData++);
			}
		}
		LL_I2C_ClearFlag_STOP(hi2c);
	#endif
}
void HAL_I2C_Master_ReceiveH(void *hi2c,uint16_t address,uint8_t *pData,uint16_t size,uint32_t timeout)
{
	#ifdef HAL_DRIVER
		HAL_I2C_Master_Receive(hi2c,address,pData,size,timeout);
	#else
		LL_I2C_HandleTransfer(hi2c,address,LL_I2C_ADDRSLAVE_7BIT,size,LL_I2C_MODE_AUTOEND,LL_I2C_GENERATE_START_READ);
		while(!LL_I2C_IsActiveFlag_STOP(hi2c))
		{
			if(LL_I2C_IsActiveFlag_RXNE(hi2c))
			{
				*pData++ = LL_I2C_ReceiveData8(hi2c);
			}
		}
		LL_I2C_ClearFlag_STOP(hi2c);
	#endif
}

void HAL_DelayH(uint32_t time)
{
	#ifdef HAL_DRIVER
		HAL_Delay(time);
	#else
		LL_mDelay(time);
	#endif
}
