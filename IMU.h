/*
 * IMU.h
 *
 *  Created on: 2 mar 2025
 *      Author: usuario
 */




#ifndef IMU_H_
#define IMU_H_

#include "EDA.h"
#include "SI7051.h"
#include "MAX86150.h"
#include "IMU.h"
#include "IMU_functions.h"
#include "control.h"
#include <inttypes.h>
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


void configureIMU(void);
void ICM20648_Init(void);
void ICM20648_Accel_Config(void);
void ICM20648_Giro_Config(void);
void ICM20648_Temp_Config(void);
void ICM20648_ConfigureFIFO(void);
void ICM20648_ConfigureDMP(void);

#endif /* IMU_H_ */
