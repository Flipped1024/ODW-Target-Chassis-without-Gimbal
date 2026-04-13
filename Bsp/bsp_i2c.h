#ifndef __iic_H
#define __iic_H

#include "main.h"

typedef struct
{
	GPIO_TypeDef *SDA_GPIO;
	uint32_t SDA_PIN;
	GPIO_TypeDef *SCL_GPIO;
	uint32_t SCL_PIN;
} i2c_t;

/*user部分*/

/*IIC驱动核心代码*/
void IIC_Init(i2c_t *iic);
void SDA_Out(i2c_t *iic);
void SDA_In(i2c_t *iic);
void IIC_Start(i2c_t *iic);
void IIC_Stop(i2c_t *iic);
void IIC_writeByte(i2c_t *iic, uint8_t data);
uint8_t IIC_readByte(i2c_t *iic);
uint8_t IIC_waitAck(i2c_t *iic);
void IIC_ack(i2c_t *iic, uint8_t ack);
/*不允许更改*/

void DELAY_us(uint16_t us);

#endif