/*
 * I2C_functions.h
 *
 *  Created on: 24 mar 2025
 *      Author: usuario
 */

#ifndef I2C_FUNCTIONS_H_
#define I2C_FUNCTIONS_H_

#include "EDA.h"
#include "SI7051.h"
#include "MAX86150.h"
#include "IMU.h"
#include "IMU_functions.h"
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

I2C_TransferReturn_TypeDef writeI2C(uint8_t slaveAddr, uint8_t regAddr, uint8_t *data, uint8_t dataLen);
I2C_TransferReturn_TypeDef readI2C(uint8_t slaveAddr, uint8_t regAddr, uint8_t *data, uint8_t length);
void I2C_InitConfig(void);


#endif /* I2C_FUNCTIONS_H_ */
