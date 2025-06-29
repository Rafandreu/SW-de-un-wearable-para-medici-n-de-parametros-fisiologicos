/*
 * EDA.h
 *
 *  Created on: 17 ene 2025
 *      Author: usuario
 */

#ifndef EDA_H_
#define EDA_H_

void initLETIMER(void);
void initADC (void);
void LETIMER0_IRQHandler(void);
void ADC0_IRQHandler(void);
void limpiarBufferI2C(void);
#endif /* EDA_H_ */
