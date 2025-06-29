/*
 * SI7051.c
 *
 *  Created on: 22 ene 2025
 *      Author: usuario
 */


#include "EDA.h"
#include "SI7051.h"
#include "MAX86150.h"
#include "IMU.h"
#include "control.h"
#include "I2C_functions.h"
#include "IMU_functions.h"
#include <stdio.h>
#include <inttypes.h>
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


// SI7051_Config_Settings SI7051 Configuration Settings
#define SI7051_I2C_BUS_ADDRESS           0x40               /**< I2C bus address                        */

// Comandos del sensor SI7051
#define SI7051_CMD_MEASURE_TEMP          0xE3 /**< Medir temperatura, modo Hold Master */
#define SI7051_CMD_MEASURE_TEMP_NO_HOLD  0xF3 /**< Medir temperatura, sin Hold Master */
#define SI7051_CMD_RESET                 0xFE /**< Reset del sensor */
#define SI7051_CMD_WRITE_USER_REG1       0xE6 /**< Escribir registro de usuario 1 */
#define SI7051_CMD_READ_USER_REG1        0xE7 /**< Leer registro de usuario 1 */
#define SI7051_CMD_READ_ID_BYTE1         { 0xFA, 0x0F } /**< Leer ID electrónico, byte 1 */
#define SI7051_CMD_READ_ID_BYTE2         { 0xFC, 0xC9 } /**< Leer ID electrónico, byte 2 */
#define SI7051_CMD_READ_FW_REV           { 0x84, 0xB8 } /**< Leer revisión de firmware */


float readTemperature(void) {
    uint8_t tempData[2] = {0};
    uint8_t cmd = SI7051_CMD_MEASURE_TEMP;

    // Enviar comando de medición de temperatura
    if (writeI2C(SI7051_I2C_BUS_ADDRESS, cmd, NULL, 0) != i2cTransferDone) {
        printf("Error al enviar comando de medición de temperatura\n");
        return -100.0; // Valor de error
    }
    // Leer los 2 bytes de temperatura
    if (readI2C(SI7051_I2C_BUS_ADDRESS, cmd, tempData, 2) != i2cTransferDone) {
        printf("Error al leer datos de temperatura\n");
        return -100.0; // Valor de error
    }

    // Convertir los datos a temperatura en °C
    uint16_t rawTemp = (tempData[0] << 8) | tempData[1];
    //printf("Raw Temperature Data (integer): %u\n", rawTemp);

    float temperatura = ((175.72 * rawTemp) / 65536.0) - 46.85;
    //printf("Temperatura:%.2f°C\n", temperatura);
    printf("Temperatura:%.2f\n", temperatura);

    return temperatura;
}


void resetTemp(){
  if (writeI2C(SI7051_I2C_BUS_ADDRESS, SI7051_CMD_RESET, NULL, 0) != i2cTransferDone) {
          printf("Error al resetear temperatura\n");
      }
}

