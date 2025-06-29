/*
 * I2C_functions.c
 *
 *  Created on: 24 mar 2025
 *      Author: usuario
 */


#include "EDA.h"
#include "SI7051.h"
#include "MAX86150.h"
#include "IMU.h"
#include "IMU_functions.h"
#include "I2C_functions.h"
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

//I2C
#define I2C_SCL_PORT     gpioPortC
#define I2C_SCL_PIN      11
#define I2C_SDA_PORT     gpioPortC
#define I2C_SDA_PIN      10
int modo2=1;

// Definir el puerto y el pin para el LED que se encenderá en el main
#define MAIN_LED_PORT gpioPortA
#define MAIN_LED_PIN 6

// Definir el puerto y el pin para el LED que se encenderá en la interrupción
#define TIMER_LED_PORT gpioPortA
#define TIMER_LED_PIN 5

// Función para escribir en el dispositivo I2C
 I2C_TransferReturn_TypeDef writeI2C(uint8_t slaveAddr, uint8_t regAddr, uint8_t *data, uint8_t dataLen) {
    I2C_TransferSeq_TypeDef seq;
    seq.addr = slaveAddr << 1;  // Dirección del esclavo con bit R/W = 0 (escritura)
    seq.flags = I2C_FLAG_WRITE;

    // Configuración de buffer de datos
    uint8_t buffer[dataLen + 1];
    buffer[0] = regAddr;  // El primer byte es la dirección del registro
    for (int i = 0; i < dataLen; i++) {
        buffer[i + 1] = data[i];  // Los siguientes bytes son los datos
    }

    seq.buf[0].data = buffer;
    seq.buf[0].len = dataLen + 1;

    // Enviar la transferencia I2C
    I2C_TransferReturn_TypeDef result = I2C_TransferInit(I2C0, &seq);
    while (result == i2cTransferInProgress) {
        result = I2C_Transfer(I2C0);
    }
    return result;
}

 // Función para leer registros del MAX86150
 I2C_TransferReturn_TypeDef readI2C(uint8_t slaveAddr, uint8_t regAddr, uint8_t *data, uint8_t length) {

     I2C_TransferSeq_TypeDef seq;
     I2C_TransferReturn_TypeDef result;

     // Configurar la primera transferencia (escritura de la dirección del registro)
     seq.addr = slaveAddr << 1;  // Dirección con bit R/W en 0 (escritura)
     seq.flags = I2C_FLAG_WRITE;
     seq.buf[0].data = &regAddr;
     seq.buf[0].len = 1;

     result = I2C_TransferInit(I2C0, &seq);
     while (result == i2cTransferInProgress) {
         result = I2C_Transfer(I2C0);
     }
     if (result != i2cTransferDone) {
         printf("Error al establecer dirección del registro\n");
         return result;
     }

     // Configurar la segunda transferencia (lectura de los datos)
     //seq.addr = slaveAddr << 1;  // Dirección con bit R/W en 1 (lectura)
     memset(&seq, 0, sizeof(I2C_TransferSeq_TypeDef));
     seq.addr = (slaveAddr << 1) | 1;
     seq.flags = I2C_FLAG_READ;
     seq.buf[0].data = data;
     seq.buf[0].len = length;

     result = I2C_TransferInit(I2C0, &seq);
     while (result == i2cTransferInProgress) {
         result = I2C_Transfer(I2C0);
     }
     if (result != i2cTransferDone) {
         printf("Error al leer los datos\n");
     }

     return result;
 }



 void I2C_InitConfig(void) {
     CMU_ClockEnable(cmuClock_I2C0, true);

     GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModeWiredAnd, 1);
     GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN, gpioModeWiredAnd, 1);

     I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
     //i2cInit.freq = I2C_FREQ_STANDARD_MAX;
     i2cInit.freq = I2C_FREQ_FAST_MAX;
     I2C_Init(I2C0, &i2cInit);

     I2C0->ROUTELOC0 = (I2C_ROUTELOC0_SDALOC_LOC15 | I2C_ROUTELOC0_SCLLOC_LOC15);
     I2C0->ROUTEPEN = (I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN);

 }


