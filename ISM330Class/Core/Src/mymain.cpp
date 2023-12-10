/*
 * Ims330dlc_InitObjet.cpp
 *
 *  Created on: 30 nov. 2023
 *      Author: Tarik
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx_hal_gpio.h"

#include "../../BSP/ISM330DLCSensor.h"
#include "../../BSP/SPIClass.h"
#include <cstdio>
#include <cmath>

/************ external objet and external for C++ execution in C*/
extern SPI_HandleTypeDef hspi4;
extern "C" void Ims330dlc_InitObjet(void);
extern "C" void Ism330dlc_CallBackFunction(void);
/****************************************************************/

#define INTEGRATION  10e-3
#define SENSOR_ODR 104.0f // In Hertz
#define ACC_FS 2 // In g
#define GYR_FS 2000 // In dps
#define MEASUREMENT_TIME_INTERVAL (1000.0f/SENSOR_ODR) // In ms
#define FIFO_SAMPLE_THRESHOLD 199
#define FLASH_BUFF_LEN 8192

/*************** Satic function *********************************/
//4 order runge kutta integration 
static void Runge_Kutta_Integration(int32_t *acceleration, float initialVelocity, float timeStep, float *integral);

//Euler integration with 1 order
static void integrate(int32_t *acceleration, float initialVelocity, float timeStep, float *integral);
/****************************************************************/

int32_t accelerometer[3];
int32_t gyroscope[3];
float velocity[3];
float Speed2AfterIngeration[3];
float velocity2;
float  AngleAfterIngeration[3];
float  tauxRotation;

/*****objects ******/
SPIClass dev_interface(hspi4);
ISM330DLCSensor AccGyr(&dev_interface, SPI4_CS_Pin, 1400000);
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void Ims330dlc_InitObjet(void)
{

	HAL_StatusTypeDef STATUS;
	ISM330DLCStatusTypeDef STATUS1;
	uint8_t transerfer = 0xA5;
	uint8_t pRxDATA;

	//hardware spi4 test
	for (uint16_t indexer = 0; indexer < 100; indexer++)
	{
		HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);

		STATUS = HAL_SPI_TransmitReceive(&hspi4, &transerfer, &pRxDATA, 1, 5);

		if (STATUS != HAL_OK)
		{
			Error_Handler();
		}
		HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
	}

	dev_interface.begin();

	// Initialize IMU.
	STATUS1 = AccGyr.begin();


	if (STATUS1 == ISM330DLC_STATUS_OK) {
		AccGyr.Enable_X();
		AccGyr.Enable_G();
	}

	/* Read the Acc id*/
	uint8_t id;
	float acc_sensitivity= 0.0;
	float gyro_sensitivity= 0.0;

	STATUS1 = AccGyr.ReadID(&id);
	AccGyr.Get_X_Sensitivity(&acc_sensitivity);
	AccGyr.Get_G_Sensitivity(&gyro_sensitivity);

	/* set odr */
	AccGyr.Set_X_ODR(12.0);
	AccGyr.Set_G_ODR(12.0);

	/* read odr */
	float  odr;
    AccGyr.Get_X_ODR(&odr);
    AccGyr.Get_G_ODR(&odr);

    /* set full scale*/
    AccGyr.Set_G_FS(2000.0);
    AccGyr.Set_X_FS(2);

    float fullScale;
    /* get full scale*/
    AccGyr.Get_G_FS(&fullScale);
    AccGyr.Get_X_FS(&fullScale);

	/* Object construction */

}
/**
  * @brief Runge-Kutta integration method
  * @param None
  * @retval None
  */
static void Runge_Kutta_Integration(int32_t * acceleration, float initialVelocity, float timeStep, float * integral) 
{

	float k1 = 0.0;
	float k2 = 0.0;
	float k3 = 0.0;
	float k4 = 0.0;
	for (uint8_t i = 0; i<3; i++)
	{
		k1 = acceleration[i];
		k2 = acceleration[i] + 0.5 * k1 * timeStep;
		k3 = acceleration[i] + 0.5 * k2 * timeStep;
		k4 = acceleration[i] + k3 * timeStep;

		integral [i] += initialVelocity + (k1 + 2*k2 + 2*k3 + k4) * (timeStep / 6.0);

	}

}
/**
  * @brief simple integration method 
  * @param None
  * @retval None
  */
static void integrate(int32_t* acceleration, float initialVelocity, float timeStep, float * integral)
{
    integral[0] += initialVelocity + (acceleration[0] * timeStep);
    integral[1] += initialVelocity + (acceleration[1] * timeStep);
    integral[2] += initialVelocity + (acceleration[2] * timeStep);
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void Ism330dlc_CallBackFunction(void)
{
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	/*Get accelerometer and gyroscope data in [mg] and [mdps]*/
	AccGyr.Get_X_Axes(accelerometer);
	AccGyr.Get_G_Axes(gyroscope);
	
	/**speed ingegration */
	if(accelerometer)
	{
		/* Integration methods */
		Runge_Kutta_Integration(accelerometer, 0, INTEGRATION, Speed1AfterIngeration);

		/**Normal integration for comparision */
		integrate(accelerometer, 0, INTEGRATION, Speed2AfterIngeration);
	}

	/* Convert mm/s to m/s */
	for (uint8_t i = 0; i<3 ; i++)
	{
		Speed1AfterIngeration[i] = Speed1AfterIngeration[i] * 1e-3;
		Speed2AfterIngeration[i] = Speed2AfterIngeration[i] * 1e-3;
	}

	/**limit integration if values are less then certain value */
	if (abs(gyroscope[0]) > 1000 && abs(gyroscope[1]) > 1000 && abs(gyroscope[2]) > 1000 )
	{
		Runge_Kutta_Integration(gyroscope, 0 ,INTEGRATION, AngleAfterIngeration);
	}

	/**Convert from mdeg  to deg*/
	for(uint8_t i = 0; i<3; i++)
	{
		Runge_Kutta_Integration[i] = Runge_Kutta_Integration[i] * 1e-3;
	}

}

#ifdef __cplusplus
}
#endif
