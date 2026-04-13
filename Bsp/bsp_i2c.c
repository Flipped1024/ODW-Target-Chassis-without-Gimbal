#include "bsp_i2c.h"
#include "bsp_dwt.h"

// IIC初始化
void IIC_Init(i2c_t *iic)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = iic->SCL_PIN | iic->SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(iic->SCL_GPIO, &GPIO_InitStruct);
	HAL_GPIO_Init(iic->SDA_GPIO, &GPIO_InitStruct);
}
// SDA 输出模式
void SDA_Out(i2c_t *iic)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = iic->SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(iic->SDA_GPIO, &GPIO_InitStruct);
}

// SDA 输出模式
void SDA_In(i2c_t *iic)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = iic->SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(iic->SDA_GPIO, &GPIO_InitStruct);
}
// IIC开始信号
void IIC_Start(i2c_t *iic)
{
	SDA_Out(iic);
	HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(iic->SDA_GPIO, iic->SDA_PIN, GPIO_PIN_SET);
	DELAY_us(4);
	HAL_GPIO_WritePin(iic->SDA_GPIO, iic->SDA_PIN, GPIO_PIN_RESET);
	DELAY_us(4);
	HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_RESET);
}
// IIC结束信号
void IIC_Stop(i2c_t *iic)
{
	SDA_Out(iic);
	HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_RESET);
	DELAY_us(4);
	HAL_GPIO_WritePin(iic->SDA_GPIO, iic->SDA_PIN, GPIO_PIN_RESET);
	DELAY_us(4);
	HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_SET);
	DELAY_us(4);
	HAL_GPIO_WritePin(iic->SDA_GPIO, iic->SDA_PIN, GPIO_PIN_SET);
	DELAY_us(4);
}
// IIC写入一字节
void IIC_writeByte(i2c_t *iic, uint8_t data)
{
	uint8_t i;
	SDA_Out(iic); // SDA输出模式

	for (i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_RESET);
		// MSB_first（LSB_first0x80改为0x01,data<<=1改为data>>=1）
		(data & 0x80) ? HAL_GPIO_WritePin(iic->SDA_GPIO, iic->SDA_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(iic->SDA_GPIO, iic->SDA_PIN, GPIO_PIN_RESET);
		data <<= 1;
		DELAY_us(4);
		HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_SET);
		DELAY_us(4);
	}
	HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_RESET);
}
// IIC读取一字节
uint8_t IIC_readByte(i2c_t *iic)
{
	uint8_t i = 0, data = 0;

	SDA_In(iic); // SDA输入模式
	data = 0;
	for (i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_RESET);
		DELAY_us(4);
		HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_SET);
		// MSB_first(LSB_first,data<<=1改为data>>=1)
		data <<= 1;
		data |= HAL_GPIO_ReadPin(iic->SDA_GPIO, iic->SDA_PIN); // MSB_first(LSB_first,改为data |= (HAL_GPIO_ReadPin(iic->SDA_GPIO, iic->SDA_PIN) << 8 ))
		DELAY_us(4);
	}
	return data;
}
uint8_t IIC_waitAck(i2c_t *iic) // 0有应答
{
	uint8_t ack = 1;
	HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_RESET);
	SDA_In(iic);
	DELAY_us(4);
	HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_SET);
	DELAY_us(4);
	ack = HAL_GPIO_ReadPin(iic->SDA_GPIO, iic->SDA_PIN);
	HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_RESET);
	return ack;
}
void IIC_ack(i2c_t *iic, uint8_t ack) // 0应答
{
	HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_RESET);

	SDA_Out(iic);
	if (ack == 0)
		HAL_GPIO_WritePin(iic->SDA_GPIO, iic->SDA_PIN, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(iic->SDA_GPIO, iic->SDA_PIN, GPIO_PIN_SET);
	DELAY_us(4);
	HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_SET);
	DELAY_us(4);
	HAL_GPIO_WritePin(iic->SCL_GPIO, iic->SCL_PIN, GPIO_PIN_RESET);
}

//延时
void DELAY_us(uint16_t us)
{
	// DWT延时
	static float DelayTime = 0.0f;
	DelayTime = (float)us * 0.000001f;
	DWT_Delay(DelayTime);
}
