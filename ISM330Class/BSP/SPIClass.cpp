/*
 * SPIClass.cpp
 *
 *  Created on: 26 nov. 2023
 *      Author: Tarik
 */

#include "../BSP/SPIClass.h"

#include "stm32f4xx_hal.h"
SPIClass::SPIClass() {


}
SPIClass::SPIClass(SPI_HandleTypeDef spi)
{
	spi_device = spi;

}

SPIClass::~SPIClass() {

}
void SPIClass::begin()
{

}
void SPIClass::pinMode(uint16_t pin, PinDirectionDefinition OUTPUT)
{

}
void SPIClass::digitalWrite(uint16_t cs_pin, PinLevel pin_Level)
{
	GPIO_PinState value;

	if (LOW == pin_Level)
	{
		value = GPIO_PIN_RESET;
	}
	else
	{
		value = GPIO_PIN_SET;
	}

	HAL_GPIO_WritePin(GPIOE, cs_pin, value);
}
void SPIClass::beginTransaction()
{

}

uint8_t SPIClass::transfer(uint8_t addresse)
{
	uint8_t ReadData = 0;
	uint8_t localAdress = addresse;
	HAL_StatusTypeDef hal_status = HAL_OK;

	/* send one element */
	hal_status = HAL_SPI_TransmitReceive(&spi_device, &localAdress, &ReadData, 1, 1);

	if (hal_status != HAL_OK)
	{
		Error_Handler();
	}
	return ReadData;
}
void SPIClass::endTransaction()
{

}
