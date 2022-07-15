/**
  ******************************************************************************
  * @file           : RT_ds18b20.h
  * @brief          : DS18B20 library
  * Created on:     : 13.12.2021
  * Author:         : Rubén Torres Bermúdez <rubentorresbermudez@gmail.com>
  ******************************************************************************
  * @attention
  *
  *  This library provides basic functionalities for controlling the DS18B20 
  *  digital thermometer in STM32 microcontroller.
  *  Include the appropriate hal lib and select the 1-wire signal line port and pin.
  *  For more information, see the .c file!
  * 
  *  Copyright (C) 2021 Rubén Torres Bermúdez
  ******************************************************************************
  */



#include <math.h>
#include <main.h>						//Include your main skecth
#include <stm32f4xx_hal.h>          	// <-- choose the hal lib appropriate for your device


#define SEARCH_ROM			0xF0
#define READ_ROM			0x33
#define MATCH_ROM			0x55
#define SKIP_ROM			0xCC
#define ALARM_SEARCH		0xEC
#define CONVERT_T	        0x44
#define WRITE_SCRATCHPAD	0x4E
#define READ_SCRATCHPAD		0xBE
#define COPY_SCRATCHPAD		0x48
#define RECALL_E2			0xB8
#define READ_POWERSUPPLY	0xB4


void write_Bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState bit_tx);
_Bool read_Bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void write_Byte(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t data_tx);
uint8_t read_Byte(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
_Bool reset_CMD(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void readROM_CMD(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t* dirb[]);
void matchROM_CMD (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t dirb[]);
void skipROM_CMD(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
_Bool convertT_CMD(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void readScratchpad_CMD(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
float decode_Temperature (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

float DS18B20_ConvertRead_Temp(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t dirb[]);
_Bool DS18B20_All_Convert(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
float DS18B20_Read_Temp(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t dirb[]);





