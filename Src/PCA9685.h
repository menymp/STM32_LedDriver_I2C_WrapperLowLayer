/*
 * PCA9685.h
 *
 *  Created on: 10 abr. 2019
 *      Author: TOSHIBA
 */
#include "main.h"

#ifndef PCA9685_H_
#define PCA9685_H_

//#define HAL_DRIVER 1 //comentar si se va a trabajar con LowLayer



//# Registers/etc:
#define PCA9685_ADDRESS     0x40
#define MODE1               0x00
#define MODE2               0x01
#define SUBADR1             0x02
#define SUBADR2             0x03
#define SUBADR3             0x04
#define PRESCALE            0xFE
#define LED0_ON_L           0x06
#define LED0_ON_H           0x07
#define LED0_OFF_L          0x08
#define LED0_OFF_H          0x09
#define ALL_LED_ON_L        0xFA
#define ALL_LED_ON_H        0xFB
#define ALL_LED_OFF_L       0xFC
#define ALL_LED_OFF_H       0xFD

//# Bits:
#define RESTART             0x80
#define SLEEP               0x10
#define ALLCALL             0x01
#define INVRT               0x10
#define OUTDRV              0x04
#define RESET 				0x00

#ifdef HAL_DRIVER
	#define GPIO_PORT_H 			GPIOC
	#define GPIO_PIN_ADDR0_H	  	GPIO_PIN_6
	#define GPIO_PIN_ADDR1_H		GPIO_PIN_8
	#define GPIO_PIN_OE_H			GPIO_PIN_2
	#define GPIO_PIN_HOLD_H			GPIO_PIN_10
#else
	#define GPIO_PORT_H 			GPIOC
	#define GPIO_PIN_ADDR0_H	  	LL_GPIO_PIN_6
	#define GPIO_PIN_ADDR1_H		LL_GPIO_PIN_8
	#define GPIO_PIN_OE_H			LL_GPIO_PIN_2
	#define GPIO_PIN_HOLD_H			LL_GPIO_PIN_10
	#define GPIO_PIN_SET			1
	#define GPIO_PIN_RESET			0
#endif

void pca9685_init(void *hi2c, uint8_t address);
void pca9685_pwm(void *hi2c, uint8_t address, uint8_t num, uint16_t on, uint16_t off);
void set_all_pwm(void *hi2c,uint8_t address,uint8_t on, uint8_t off);
void write_register(void *hi2c, uint8_t address,uint8_t register_pointer, uint16_t register_value);
void read_register(void *hi2c, uint8_t address,uint8_t register_pointer, uint8_t* receive_buffer);
void set_pwm_freq(void *hi2c, uint8_t address,uint8_t freq);
//porting block
void HAL_GPIO_WritePinH(void *GPIO,uint16_t PIN,int state);
void HAL_I2C_Master_TransmitH(void *hi2c,uint16_t address,uint8_t *pData,uint16_t size,uint32_t timeout );
void HAL_I2C_Master_ReceiveH(void *hi2c,uint16_t address,uint8_t *pData,uint16_t size,uint32_t timeout);
void HAL_DelayH(uint32_t time);

#endif /* PCA9685_H_ */
