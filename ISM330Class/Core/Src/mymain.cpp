/*
 * mymain.cpp
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
extern "C" void mymain(void);
extern "C" void AccGyro_call_back_function(void);
/****************************************************************/

#define INTEGRATION  100e-3
#define SENSOR_ODR 104.0f // In Hertz
#define ACC_FS 2 // In g
#define GYR_FS 2000 // In dps
#define MEASUREMENT_TIME_INTERVAL (1000.0f/SENSOR_ODR) // In ms
#define FIFO_SAMPLE_THRESHOLD 199
#define FLASH_BUFF_LEN 8192

/*************** Satic function *********************************/
static void rungeKutta(int32_t *acceleration, float initialVelocity, float timeStep, float *integral);
static void integrate(int32_t *acceleration, float initialVelocity, float timeStep, float *integral);
static void rungeKuttaIntegration(int32_t * tauxRotation, float deltaTime, float * Output);
/****************************************************************/

int32_t accelerometer[3];
int32_t gyroscope[3];
float velocity[3];
float velocity1[3];
float velocity2;
float  angle[3];
float  tauxRotation;
/*****objects ******/
SPIClass dev_interface(hspi4);
ISM330DLCSensor AccGyr(&dev_interface, SPI4_CS_Pin, 1400000);

void mymain(void)
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
	while (1)
	{
		
	}

}

// Runge-Kutta integration method
static void rungeKutta(int32_t * acceleration, float initialVelocity, float timeStep, float * integral) {

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

static void integrate(int32_t* acceleration, float initialVelocity, float timeStep, float * integral) {
    integral[0] = initialVelocity + (acceleration[0] * timeStep);
    integral[1] = initialVelocity + (acceleration[1] * timeStep);
    integral[2] = initialVelocity + (acceleration[2] * timeStep);
}

// Méthode de Runge-Kutta d'ordre 4 pour l'intégration numérique
static void rungeKuttaIntegration(int32_t * tauxRotation, float deltaTime, float * Output) {
    float k1 = 0.0;
    float k2 = 0.0;
    float k3 = 0.0;
    float k4 = 0.0;

    for (uint16_t i = 0; i < 3; i++)
    {
        k1 = tauxRotation[i];
        k2 = tauxRotation[i] + 0.5 * k1 * deltaTime;
        k3 = tauxRotation[i] + 0.5 * k2 * deltaTime;
        k4 = tauxRotation[i] + k3 * deltaTime;
        Output [i] += (k1 + 2 * k2 + 2 * k3 + k4) * (deltaTime / 6.0);
	}

}


void AccGyro_call_back_function(void)
{
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	/*Get accelerometer and gyroscope data in [mg] and [mdps]*/
	AccGyr.Get_X_Axes(accelerometer);
	AccGyr.Get_G_Axes(gyroscope);

	if(accelerometer)
	{
		/* Integration methods */
		rungeKutta(accelerometer, 0, INTEGRATION, velocity);
		integrate(accelerometer, 0, INTEGRATION, velocity1);
	}

	/* Convert m/s to km/h */
	for (uint8_t i = 0; i<3 ; i++)
	{
		velocity[i] = velocity[i];
		velocity1[i] = velocity1[i];
	}

	for(uint8_t i=0; i<3 ; i++)
	{
		//printf(" Runge-Kutta velocity integration: %.3f mm/s, normal integration velocity1: %.3f mm/s for %d \n", velocity[i], velocity1[i], i);
	}


	// Integration de l'angle en utilisant la méthode de Runge-Kutta
	// integrer si la variation est importante
	if (abs(gyroscope[0]) > 1000 && abs(gyroscope[1]) > 1000 && abs(gyroscope[2]) > 1000 )
	{
		rungeKuttaIntegration(gyroscope, INTEGRATION, angle);
	}

	for(uint16_t i=0; i<3 ; i++)
	{
		//printing the angle on the console swv itm data
		//printf("L'angle integre avec Runge-Kutta est de : %.3f mdegres indice %d\n",angle[i], i);
	}
}

#ifdef __cplusplus
}
#endif
