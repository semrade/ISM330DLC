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


void mymain(void);

typedef void (*functionPointer)(void);
void Ism330dlc_CallBackFunction(void);
functionPointer GyroAccelCalctable[1] = {Ism330dlc_CallBackFunction};

#ifdef __cplusplus
}
#endif
#endif /* INC_MYMAIN_H_ */
