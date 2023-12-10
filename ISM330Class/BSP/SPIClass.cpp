/*
 * SPIClass.cpp
 *
 *  Created on: 26 nov. 2023
 *      Author: Tarik
 */

#include "../BSP/SPIClass.h"

#include "stm32f4xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef spi;
SPIClass::SPIClass() {


}
SPIClass::SPIClass(SPI_HandleTypeDef spi)
{

	if (spi.Instance == NULL)
	{
		spi.Instance = SPI4;
		spi.Init.Mode = SPI_MODE_MASTER;
		spi.Init.Direction = SPI_DIRECTION_2LINES;
		spi.Init.DataSize = SPI_DATASIZE_8BIT;
		spi.Init.CLKPolarity = SPI_POLARITY_LOW;
		spi.Init.CLKPhase = SPI_PHASE_1EDGE;
		spi.Init.NSS = SPI_NSS_SOFT;
		spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
		spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
		spi.Init.TIMode = SPI_TIMODE_DISABLE;
		spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		spi.Init.CRCPolynomial = 10;

		if (HAL_SPI_Init(&spi) != HAL_OK) 
		{
			Error_Handler();
		}
		// if init is okay affect to the objet
		spi_device = spi;
	}


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
