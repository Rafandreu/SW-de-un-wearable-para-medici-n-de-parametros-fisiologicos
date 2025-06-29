/*
 * IMU_functions.h
 *
 *  Created on: 18 mar 2025
 *      Author: usuario
 */

#ifndef IMU_FUNCTIONS_H_
#define IMU_FUNCTIONS_H_



#include "EDA.h"
#include "SI7051.h"
#include "MAX86150.h"
#include "IMU.h"
#include "control.h"
#include "I2C_functions.h"
#include <stdio.h>
#include "pin_config.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"  // Clock
#include "em_gpio.h"  // i/o pin
#include "em_system.h" //Definitions
#include "em_emu.h"
#include "em_ldma.h"
#include "em_timer.h"
#include "em_emu.h"
#include "em_adc.h"
#include "em_letimer.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "em_usart.h"
#include <string.h>
#include "em_i2c.h"




void selecionbank(int);
void reset(void);
void wakeup (void);
void habilitacionacelerometro(void);
void habilitacionagiroscopo(void);
void habilitaciontemp(void);
void frecuenciamuestreoacceleracion(float);
void configfiltroacelerometro(void);
void habilitacionfiltroacelerometro(void);
void configescalaacelerometro(void);
void autotestacelerometro(void);
void  configmediamuestrasacelerometro(void);
void frecuenciamuestreogiroscopo(float);
void configfiltrogiro(void);
void habilitacionfiltrogiro(void);
void configescalagiro(void);
void autotesgiro(void);
void  configmediamuestrasgiro(void);
void compacelerometro(void);
void compgiro(void);
void seleccionreloj(void);
void resetDMP(void);
void  habilitacionDMP(void);
void ICM20648_ReadAcceleration(void);
void ICM20648_ReadGyro(void);
void ICM20648_ReadTemp(void);
void habilitarFIFO(void);
void modeFIFO(void);
void habilitarinterrupcionOVFFIFO(void);
void resetFIFO(void);
void escrituraFIFO(void);
void ICM20648_ReadFIFO(void);





#endif /* IMU_FUNCTIONS_H_ */
