/**
  ******************************************************************************
  * @file           : RT_ds18b20.c
  * @brief          : DS18B20 library
  * Created on:     : 13.12.2021
  * Author:         : Rubén Torres Bermúdez <rubentorresbermudez@gmail.com>
  ******************************************************************************
  * @attention
  *
  *  This C source file provides basic functionalities for controlling
  *  the DS18B20 digital thermometer in STM32 microcontroller.
  * 
  *  Use DS18B20_ConvertRead_Temp() for single A/D conversion.
  *  Use DS18B20_All_Convert() for to do the A/D conversion on all onewire device(s).
  *  Use DS18B20_Read_Temp() for single read temp.
  *
  *  Copyright (C) 2021 Rubén Torres Bermúdez
  ******************************************************************************
  */


#include <RT_ds18b20.h>
#include <string.h>




/**
  * @brief	Delay us.
  * @param  delay specifies the time in microseconds.
  * @return	None.
  * @note	Write this function in your main skecht.
  */
__weak void delayMicro(uint32_t delay){}



/**
  * @brief  Write a bit with onewire communication.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param  bit_tx specifies the value to be written.
  *          This parameter can be one of the GPIO_PinState enum values:
  *            @arg GPIO_PIN_RESET: to write 0.
  *            @arg GPIO_PIN_SET: to write 1.
  * @retval None.
  */
void write_Bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState bit_tx)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	delayMicro(3);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, bit_tx);
	delayMicro(57);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}



/**
  * @brief  Read a bit with onewire communication.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval The bit read.
  */
_Bool read_Bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	_Bool bit_rx;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	delayMicro(2);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	delayMicro(10);
	bit_rx = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
	delayMicro(48);
	return bit_rx;
}



/**
  * @brief  Write a byte with onewire communication.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param  data_tx specifies the byte to be written.
  * @retval None
  */
void write_Byte(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t data_tx)
{
	uint8_t mask = 0b00000001;
	uint8_t bit_tx = data_tx & mask;
	for(uint8_t i=0; i<8; i++)
	{
		write_Bit(GPIOx, GPIO_Pin, bit_tx);
		delayMicro(3);
        mask = mask << 1;
        bit_tx = data_tx & mask;
	}
}



/**
  * @brief  Read a byte with onewire communication.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval The byte read.
  */
uint8_t read_Byte(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	uint8_t byte_rx = 0;
	for(uint8_t i=0;  i < 8; i++)
	{
		byte_rx |= read_Bit(GPIOx, GPIO_Pin) << i;
	}
	return byte_rx;
}



/**
  * @brief  Send a command to initialices the onewire communication and the connected device(s).
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval This parameter can be one of the enum values:
  *            @arg 0: if there are device(s) on the onewire communication.
  *            @arg 1: if there aren't device(s) on the onewire communication.
  */
_Bool reset_CMD(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	_Bool fault;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	delayMicro(500);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	delayMicro(100);
	fault = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
	delayMicro(500);
	return fault;
}



void readROM_CMD(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t* dirb[])
{
		write_Byte(GPIOx, GPIO_Pin, READ_ROM);

		*dirb[0] = read_Byte(TempDS2_GPIO_Port, TempDS2_Pin);
		*dirb[1] = read_Byte(TempDS2_GPIO_Port, TempDS2_Pin);
		*dirb[2] = read_Byte(TempDS2_GPIO_Port, TempDS2_Pin);
		*dirb[3] = read_Byte(TempDS2_GPIO_Port, TempDS2_Pin);
		*dirb[4] = read_Byte(TempDS2_GPIO_Port, TempDS2_Pin);
		*dirb[5] = read_Byte(TempDS2_GPIO_Port, TempDS2_Pin);
		*dirb[6] = read_Byte(TempDS2_GPIO_Port, TempDS2_Pin);
		*dirb[7] = read_Byte(TempDS2_GPIO_Port, TempDS2_Pin);


//    uint8_t crc, family_code;
//    uint64_t rom_code, serial_num;
//	write_Byte(GPIOx, GPIO_Pin, READ_ROM);
//
//    family_code = read_Byte(GPIOx, GPIO_Pin);
//    serial_num = read_Byte(GPIOx, GPIO_Pin);
//    serial_num |= (read_Byte(GPIOx, GPIO_Pin) << 8);
//    serial_num |= (read_Byte(GPIOx, GPIO_Pin) << 16);
//    crc = read_Byte(GPIOx, GPIO_Pin);
//    rom_code = (crc << 24) | (serial_num << 8) |  family_code;
}



/**
  * @brief  Send a command to select a onewire device.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param	dirb[] specifies the 64-bit lasered ROM code of the onewire device LSBF.
  * @retval None.
  */
void matchROM_CMD (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t dirb[])
{
	write_Byte(GPIOx, GPIO_Pin, MATCH_ROM);
	write_Byte(GPIOx, GPIO_Pin, dirb[0]);
	write_Byte(GPIOx, GPIO_Pin, dirb[1]);
	write_Byte(GPIOx, GPIO_Pin, dirb[2]);
	write_Byte(GPIOx, GPIO_Pin, dirb[3]);
	write_Byte(GPIOx, GPIO_Pin, dirb[4]);
	write_Byte(GPIOx, GPIO_Pin, dirb[5]);
	write_Byte(GPIOx, GPIO_Pin, dirb[6]);
	write_Byte(GPIOx, GPIO_Pin, dirb[7]);
}



/**
  * @brief  Send a command to select all onewire device.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval None.
  */
void skipROM_CMD(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	write_Byte(GPIOx, GPIO_Pin, SKIP_ROM);
}



/**
  * @brief  Send a comad to convert A/D the temperature of the selected onewire device.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval This parameter can be one of the enum values:
  *            @arg 0: if there has been no failure in the conversion.
  *            @arg 1: if there has been failure in the conversion.
  */
_Bool convertT_CMD(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	_Bool fault = 1;
	write_Byte(GPIOx, GPIO_Pin, CONVERT_T);
	for(uint8_t i=0; i<255; i++)
	{
		if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET)
		{
			fault = 0;
			break;
		}
		delayMicro(50);
	}
	return fault;
}



/**
  * @brief  Send a comad to receive the scratchpad of the selected onewire device.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval None.
  */
void readScratchpad_CMD(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	write_Byte(GPIOx, GPIO_Pin, READ_SCRATCHPAD);
}



/**
  * @brief  Read the scratchpad of the selected onewire device and decode the data temperature.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval None.
  */
float decode_Temperature (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	int8_t k;
	uint8_t temp_LSB, temp_MSB;
	uint16_t u16_temp, mask = 1;
	float temperature = 0.0;
	temp_LSB = read_Byte(GPIOx, GPIO_Pin);
	temp_MSB = read_Byte(GPIOx, GPIO_Pin);

	u16_temp = ((temp_MSB << 8) | temp_LSB);

	for (k = -4; k < 7; k++)
	{
		if (u16_temp & mask)
		{
			temperature += powf(2,k);
		}
		mask = mask << 1;
	}
	return temperature;
}



/**
  * @brief  Shorcut to do the conversion and read of the temperature in a onewire device.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param	dirb[] specifies the 64-bit lasered ROM code of the onewire device LSBF.
  * @retval This parameter can be one of the enum values:
  *            @arg temp: return the device temperature.
  *            @arg fault: return -250.0 if there has been failure
  */
float DS18B20_ConvertRead_Temp(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t dirb[])
{
	float temp;
	float fault = -250.0;
	if(reset_CMD(GPIOx, GPIO_Pin))
	{
		return fault;
	}
	matchROM_CMD(GPIOx, GPIO_Pin, dirb);
	if(convertT_CMD(GPIOx, GPIO_Pin))
	{
		return fault;
	}
	if(reset_CMD(GPIOx, GPIO_Pin))
	{
		return fault;
	}
	matchROM_CMD(GPIOx, GPIO_Pin, dirb);
	readScratchpad_CMD(GPIOx, GPIO_Pin);
	temp = decode_Temperature(GPIOx, GPIO_Pin);
	return temp;
}



/**
  * @brief  Shorcut to do the A/D conversion on all onewire device(s).
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval This parameter can be one of the enum values:
  *            @arg 0: if there has been no failure in the conversion.
  *            @arg 1: if there has been failure in the conversion.
  */
_Bool DS18B20_All_Convert(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	_Bool fault = 0;
	if(reset_CMD(GPIOx, GPIO_Pin))
	{
		return fault;
	}
	skipROM_CMD(GPIOx, GPIO_Pin);
	if(convertT_CMD(GPIOx, GPIO_Pin))
	{
		return fault;
	}
	return fault;
}



/**
  * @brief  Shorcut to read the temperature of a onewire device.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param	dirb[] specifies the 64-bit lasered ROM code of the onewire device LSBF.
  * @retval This parameter can be one of the enum values:
  *            @arg temp: return the device temperature.
  *            @arg fault: return -250.0 if there has been failure
  */
float DS18B20_Read_Temp(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t dirb[])
{
	float temp;
	float fault = -250.0;
	if(reset_CMD(GPIOx, GPIO_Pin))
	{
		return fault;
	}
	matchROM_CMD(GPIOx, GPIO_Pin, dirb);
	readScratchpad_CMD(GPIOx, GPIO_Pin);
	temp = decode_Temperature(GPIOx, GPIO_Pin);
	return temp;
}
