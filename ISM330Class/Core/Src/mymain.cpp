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
#include "mymain.h"
#include "stm32f4xx_hal_gpio.h"

#include "../../BSP/ISM330DLCSensor.h"
#include "../../BSP/SPIClass.h"
#include <cstdio>
#include <cmath>
#include "../../BSP/ISM330DLC_ACC_GYRO_Driver.h"

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
void Ism330dlc_calibration (float r_Input[], float r_Offset[] );
/****************************************************************/

int32_t Accelerometer[3];
int32_t gyroscope[3];
float Speed1AfterIntegration[3];
float Speed2AfterIntegration[3];
float velocity2;
float  AngleAfterIntegration[3];
float  tauxRotation;
float r_Offset[3];
float r_Input[3];
float AccfilteredValue[3];
int32_t AccOffset[3];

/*****objects ******/
SPIClass dev_interface(hspi4);
ISM330DLCSensor AccGyr(&dev_interface, SPI4_CS_Pin, 1400000);
// Initialize the low-pass filter with a cutoff frequency of 10 Hz and a time step of 0.01 seconds
LowPassFilter myFilter0;
LowPassFilter myFilter1;
LowPassFilter myFilter2;
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
    AccGyr.Set_G_FS(500.0);
    AccGyr.Set_X_FS(2);

    float fullScale;
    /* get full scale*/
    AccGyr.Get_G_FS(&fullScale);
    AccGyr.Get_X_FS(&fullScale);

	/* Set high performance mode */
	AccGyr.WriteReg((uint8_t)ISM330DLC_ACC_GYRO_CTRL6_G,
					(uint8_t)ISM330DLC_ACC_GYRO_XL_HM_MODE_ENABLED);

	/* Read performance*/
	//TODO verify the performance is ok
	uint8_t performance;
	AccGyr.ReadReg((uint8_t)ISM330DLC_ACC_GYRO_CTRL6_G, &performance);


	/* digital filter configuration */
	//100Hz Filter BW
	//10ms  interrupt period
	initLowPassFilter(&myFilter0, 100.0, 0.01);
	initLowPassFilter(&myFilter1, 100.0, 0.01);
	initLowPassFilter(&myFilter2, 100.0, 0.01);

	
	//Accelerometer settings
	//ODR = 12
	//FS  = 2g
	//High performance is activated ==> analog filter activated
	//ODR<1666 => BW=400Hz
	//Digital low pass filter configuration	
		// 1/ LPF1_BW_SEL =1 bit in the CTRL1_XL => ODR/4
		// 2/ NPUT_COMPOSITE bit in the CTRL8_XL
		// 3/ High performance mode => BW = ODR/4 => BW = 12/4 = 3Hz
		// 4/ to use the second LPF2 
			// 1/ LPF2_XL_EN
			// 2/ onfiguring the HPCF_XL[1:0] field of the CTRL8_XL 
	
	/** Intern filter configuration  LPF1 **/
	AccGyr.Low_Pass_filter_configuration();

	/* static calibration */

	uint8_t Index;
	for(Index = 0; Index<3; Index++)
	{
		Accelerometer[Index] = 0;
		gyroscope [Index] = 0;
	}
	/* get x and g*/
	AccGyr.Get_X_Axes(Accelerometer);
	AccGyr.Get_G_Axes(gyroscope);

	/* classical calibration method */
	AccOffset[0] = 0;
	AccOffset[1] = 0;
	AccOffset[2] = 0;

	/* advanced calibration method based on the DT0053*/

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
	/* real time calculation */
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	/*Get Accelerometer and gyroscope data in [mg] and [mdps]*/
	AccGyr.Get_X_Axes(Accelerometer);
	AccGyr.Get_G_Axes(gyroscope);
	
	/* filter acceleration data before use */
	AccfilteredValue[0]  = updateLowPassFilter(&myFilter0, Accelerometer[0] );
	AccfilteredValue[1]  = updateLowPassFilter(&myFilter1, Accelerometer[1] );
	AccfilteredValue[2]  = updateLowPassFilter(&myFilter2, Accelerometer[2] );

	/*Calibration*/
	for(uint8_t index=0; index<3; index++)
	{
		r_Input[index]=Accelerometer[index];
	}

	/* calculate offset */
	Ism330dlc_calibration (r_Input,r_Offset);

	/**speed integration */
	if(Accelerometer)
	{
		/*  Integration methods */
		//Runge_Kutta_Integration(AccfilteredValue, 0, INTEGRATION, Speed1AfterIntegration);

		/** Normal integration for comparision */
		//integrate(AccfilteredValue, 0, INTEGRATION, Speed2AfterIntegration);
	}

	/* Convert mm/s to m/s */
	for (uint8_t i = 0; i<3 ; i++)
	{
		Speed1AfterIntegration[i] = Speed1AfterIntegration[i] /** 1e-3*/;
		Speed2AfterIntegration[i] = Speed2AfterIntegration[i] /** 1e-3*/;
	}

	/**limit integration if values are less then certain value */
	if (abs(gyroscope[0]) > 1000 && abs(gyroscope[1]) > 1000 && abs(gyroscope[2]) > 1000 )
	{
		Runge_Kutta_Integration(gyroscope, 0 ,INTEGRATION, AngleAfterIntegration);
	}

	/**Convert from mdeg  to deg*/
	for(uint8_t i = 0; i<3; i++)
	{
		AngleAfterIntegration[i] = AngleAfterIntegration[i] /** 1e-3*/;
	}

	/* real time calculation */
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void Ism330dlc_kalmanFilter(void)
{
	/* Implement kalman filter here*/
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void Ism330dlc_calibration (float r_Input[], float r_Offset[] )
{
    // Calculate calibration values
    float accdeltaXcal = -r_Input[0];
    float accdeltaYcal = -r_Input[1];
    float accdeltaZcal = 1000 - r_Input[2];

    r_Offset[0] = r_Input[0] + accdeltaXcal;
    r_Offset[1] = r_Input[1] + accdeltaYcal;
    r_Offset[2] = r_Input[2] + accdeltaZcal;
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void Ism330dlc_autoTest()
{

}


// Function to initialize the low-pass filter
void initLowPassFilter(LowPassFilter* filter, float cutoffFrequency, float dt) 
{
    filter->cutoffFrequency = cutoffFrequency;
    filter->dt = dt;
    
    float tau = 1.0 / (2.0 * M_PI * cutoffFrequency);
    filter->alpha = dt / (dt + tau);

    // Initialize the filtered value based on the initial conditions if needed
    filter->currentValue = 0.0;  // Adjust as needed
}

// Function to update the low-pass filter with a new input
float updateLowPassFilter(LowPassFilter* filter, float inputValue) 
{
	//
    filter->currentValue = filter->alpha * inputValue + (1.0 - filter->alpha) * filter->currentValue;
    return filter->currentValue;
}
#ifdef __cplusplus
}
#endif
