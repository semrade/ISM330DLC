/*
 * mymain.h
 *
 *  Created on: 30 nov. 2023
 *      Author: Tarik
 */

#ifndef INC_MYMAIN_H_
#define INC_MYMAIN_H_

#ifdef __cplusplus
extern "C" {
#endif


void Ims330dlc_InitObjet(void);
void Ism330dlc_kalmanFilter (void);
void initLowPassFilter(LowPassFilter* filter, float cutoffFrequency, float dt);
float updateLowPassFilter(LowPassFilter* filter, float inputValue);

typedef void (*functionPointer)(void);
void Ism330dlc_CallBackFunction(void);
functionPointer GyroAccelCalctable[1] = {Ism330dlc_CallBackFunction, 
                                            Ism330dlc_kalmanFilter
                                        };

typedef enum
{
    Power_Mode,
    HIGH_PERFORMANCE,
    NORMAL_MODE,
    LOW_POWER_MODE,
    SLEEP_MODE
}Ism330dlc_mode;

typedef struct
{
    float cutoffFrequency;  // Cutoff frequency in Hz
    float dt;               // Time step in seconds
    float alpha;            // Filter coefficient
    float currentValue;     // Current filtered value
} LowPassFilter;


#ifdef __cplusplus
}
#endif
#endif /* INC_MYMAIN_H_ */
