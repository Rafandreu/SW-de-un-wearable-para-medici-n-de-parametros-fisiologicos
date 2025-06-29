/*
 * MAX86150.h
 *
 *  Created on: 4 feb 2025
 *      Author: usuario
 */

#ifndef MAX86150_H_
#define MAX86150_H_



#include "EDA.h"
#include "SI7051.h"
#include "MAX86150.h"
#include "IMU.h"
#include "IMU_functions.h"
#include "I2C_functions.h"
#include "control.h"
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


void configureMAX86150(void);
void settingPPG(void);
void settingECG(void);
void LEDcurrentconfiguration (void);
void LEDconfiguration (void);
void ECGconfiguration(void);
void leermuestrasMAX86150(void);
void proxmode(void);
void writeMAX86150Data(uint32_t,uint32_t);
void resetMAX86150(void);
#endif /* MAX86150_H_ */
