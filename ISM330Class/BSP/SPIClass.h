/*
 * SPIClass.h
 *
 *  Created on: 26 nov. 2023
 *      Author: Tarik
 */

#ifndef SPICLASS_H_
#define SPICLASS_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "main.h"
typedef enum
{
	OUTPUT = 0,
	INPUT  = 1
}PinDirectionDefinition;

typedef enum
{
	HIGH = 1,
	LOW = 0
}PinLevel;


class SPIClass {
public:
	SPIClass();
	SPIClass(SPI_HandleTypeDef& spi);
	virtual ~SPIClass();


	void begin();
	void pinMode(uint16_t pin, PinDirectionDefinition OUTPUT);
	void digitalWrite(uint16_t cs_pin, PinLevel pin_Level);
	void beginTransaction();
	uint8_t transfer(uint8_t addresse);
	void endTransaction();

	//SPI hall object handler
	SPI_HandleTypeDef spi_device;
};

#endif /* SPICLASS_H_ */
