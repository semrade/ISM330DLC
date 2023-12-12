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

typedef struct
{
    float cutoffFrequency;  // Cutoff frequency in Hz
    float dt;               // Time step in seconds
    float alpha;            // Filter coefficient
    float currentValue;     // Current filtered value
} LowPassFilter;

void Ims330dlc_InitObjet(void);
void Ism330dlc_kalmanFilter (void);
void initLowPassFilter(LowPassFilter* filter, float cutoffFrequency, float dt);
float updateLowPassFilter(LowPassFilter* filter, float inputValue);

typedef void (*functionPointer)(void);


typedef enum
{
    Power_Mode,
    HIGH_PERFORMANCE,
    NORMAL_MODE,
    LOW_POWER_MODE,
    SLEEP_MODE
}Ism330dlc_mode;



#ifdef __cplusplus
}
#endif
#endif /* INC_MYMAIN_H_ */
