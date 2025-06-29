/*
 * IMU.c
 *
 *  Created on: 2 mar 2025
 *      Author: usuario
 */
#include "EDA.h"
#include "SI7051.h"
#include "MAX86150.h"
#include "IMU.h"
#include "IMU_functions.h"
#include "I2C_functions.h"
#include "control.h"
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


#define ICM20648_I2C_ADDRESS   0x68  // Dirección I2C del ICM-20648

//User Bank 0
#define ICM20648_WHO_AM_I            0x00
#define ICM20648_USER_CTRL           0x03
#define ICM20648_BIT_DMP_EN          0x80
#define ICM20648_BIT_FIFO_EN         0x40
#define ICM20648_BIT_DMP_RESET       0x08

#define ICM20648_LP_CONFIG           0x05
#define ICM20648_GYRO_CYCLE_BIT      0x10
#define ICM20648_ACCEL_CYCLE_BIT     0x20


#define ICM20648_PWR_MGMT_1          0x06

#define ICM20648_BIT_H_RESET             0x80                        /**< Device reset bit                                       */
#define ICM20648_BIT_SLEEP               0x40                        /**< Sleep mode enable bit                                  */
#define ICM20648_BIT_LP_EN               0x20                        /**< Low Power feature enable bit                           */
#define ICM20648_BIT_TEMP_DIS            0x08                        /**< Temperature sensor disable bit                         */
#define ICM20648_BIT_CLK                 0x07                       /**< Auto clock source selection setting                    */

#define INTERNAL_CLK_20M  0b000
#define INTERNAL_CLK_1  0b001
#define INTERNAL_CLK_2  0b010
#define INTERNAL_CLK_3  0b011
#define INTERNAL_CLK_4  0b100
#define INTERNAL_CLK_5  0b101
#define INTERNAL_CLK_STOP  0b111


#define ICM20648_PWR_MGMT_2          0x07

#define ICM20648_BIT_PWR_ACCEL_STBY      0x38                        /**< Disable accelerometer                                  */
#define ICM20648_BIT_PWR_GYRO_STBY       0x07                        /**< Disable gyroscope                                      */
#define ICM20648_BIT_PWR_ALL_OFF         0x7F

#define ICM20648_INT_PIN_CFG         0x0F
#define ICM20648_INT_ENABLE          0x10
#define ICM20648_INT_ENABLE_1        0x11
#define ICM20648_INT_ENABLE_2        0x12
#define ICM20648_INT_ENABLE_3        0x13
#define ICM20648_I2C_MST_STATUS      0x17
#define ICM20648_INT_STATUS          0x19
#define ICM20648_INT_STATUS_1        0x1A
#define ICM20648_INT_STATUS_2        0x1B
#define ICM20648_INT_STATUS_3        0x1C
#define ICM20648_DELAY_TIMEH         0x28
#define ICM20648_DELAY_TIMEL         0x29
#define ICM20648_ACCEL_XOUT_H        0x2D
#define ICM20648_ACCEL_XOUT_L        0x2E
#define ICM20648_ACCEL_YOUT_H        0x2F
#define ICM20648_ACCEL_YOUT_L        0x30
#define ICM20648_ACCEL_ZOUT_H        0x31
#define ICM20648_ACCEL_ZOUT_L        0x32
#define ICM20648_GYRO_XOUT_H         0x33
#define ICM20648_GYRO_XOUT_L         0x34
#define ICM20648_GYRO_YOUT_H         0x35
#define ICM20648_GYRO_YOUT_L         0x36
#define ICM20648_GYRO_ZOUT_H         0x37
#define ICM20648_GYRO_ZOUT_L         0x38
#define ICM20648_TEMP_OUT_H          0x39
#define ICM20648_TEMP_OUT_L          0x3A
#define ICM20648_EXT_SLV_SENS_DATA_00 0x3B
#define ICM20648_EXT_SLV_SENS_DATA_01 0x3C
#define ICM20648_EXT_SLV_SENS_DATA_02 0x3D
#define ICM20648_EXT_SLV_SENS_DATA_03 0x3E
#define ICM20648_EXT_SLV_SENS_DATA_04 0x3F
#define ICM20648_EXT_SLV_SENS_DATA_05 0x40
#define ICM20648_EXT_SLV_SENS_DATA_06 0x41
#define ICM20648_EXT_SLV_SENS_DATA_07 0x42
#define ICM20648_EXT_SLV_SENS_DATA_08 0x43
#define ICM20648_EXT_SLV_SENS_DATA_09 0x44
#define ICM20648_EXT_SLV_SENS_DATA_10 0x45
#define ICM20648_EXT_SLV_SENS_DATA_11 0x46
#define ICM20648_EXT_SLV_SENS_DATA_12 0x47
#define ICM20648_EXT_SLV_SENS_DATA_13 0x48
#define ICM20648_EXT_SLV_SENS_DATA_14 0x49
#define ICM20648_EXT_SLV_SENS_DATA_15 0x4A
#define ICM20648_EXT_SLV_SENS_DATA_16 0x4B
#define ICM20648_EXT_SLV_SENS_DATA_17 0x4C
#define ICM20648_EXT_SLV_SENS_DATA_18 0x4D
#define ICM20648_EXT_SLV_SENS_DATA_19 0x4E
#define ICM20648_EXT_SLV_SENS_DATA_20 0x4F
#define ICM20648_EXT_SLV_SENS_DATA_21 0x50
#define ICM20648_EXT_SLV_SENS_DATA_22 0x51
#define ICM20648_EXT_SLV_SENS_DATA_23 0x52
#define ICM20648_FIFO_EN_1           0x66
#define ICM20648_FIFO_EN_2           0x67
#define ICM20648_FIFO_RST            0x68
#define ICM20648_FIFO_MODE           0x69
#define ICM20648_FIFO_COUNTH         0x70
#define ICM20648_FIFO_COUNTL         0x71
#define ICM20648_FIFO_R_W            0x72
#define ICM20648_DATA_RDY_STATUS     0x74
#define ICM20648_FIFO_CFG            0x76
#define ICM20648_REG_BANK_SEL        0x7F


//User Bank 1
#define ICM20648_SELF_TEST_X_GYRO    0x02
#define ICM20648_SELF_TEST_Y_GYRO    0x03
#define ICM20648_SELF_TEST_Z_GYRO    0x04
#define ICM20648_SELF_TEST_X_ACCEL   0x0E
#define ICM20648_SELF_TEST_Y_ACCEL   0x0F
#define ICM20648_SELF_TEST_Z_ACCEL   0x10
#define ICM20648_XA_OFFS_H           0x14
#define ICM20648_XA_OFFS_L           0x15
#define ICM20648_YA_OFFS_H           0x17
#define ICM20648_YA_OFFS_L           0x18
#define ICM20648_ZA_OFFS_H           0x1A
#define ICM20648_ZA_OFFS_L           0x1B
#define ICM20648_TIMEBASE_CORRECTION_PLL 0x28


//User Bank 2
#define ICM20648_GYRO_SMPLRT_DIV     0x00
#define ICM20648_GYRO_CONFIG_1       0x01
#define ICM20648_GYRO_CONFIG_2       0x02
#define ICM20648_XG_OFFS_USRH        0x03
#define ICM20648_XG_OFFS_USRL        0x04
#define ICM20648_YG_OFFS_USRH        0x05
#define ICM20648_YG_OFFS_USRL        0x06
#define ICM20648_ZG_OFFS_USRH        0x07
#define ICM20648_ZG_OFFS_USRL        0x08
#define ICM20648_ODR_ALIGN_EN        0x09
#define ICM20648_ACCEL_SMPLRT_DIV_1  0x10
#define ICM20648_ACCEL_SMPLRT_DIV_2  0x11
#define ICM20648_ACCEL_INTEL_CTRL    0x12
#define ICM20648_ACCEL_WOM_THR       0x13
#define ICM20648_ACCEL_CONFIG        0x14
#define ICM20648_ACCEL_CONFIG_2      0x15
#define ICM20648_FSYNC_CONFIG        0x52
#define ICM20648_TEMP_CONFIG         0x53
#define ICM20648_MOD_CTRL_USR        0x54


//User Bank 3
#define ICM20648_I2C_MST_ODR_CONFIG  0x00
#define ICM20648_I2C_MST_CTRL        0x01
#define ICM20648_I2C_MST_DELAY_CTRL  0x02
#define ICM20648_I2C_SLV0_ADDR       0x03
#define ICM20648_I2C_SLV0_REG        0x04
#define ICM20648_I2C_SLV0_CTRL       0x05
#define ICM20648_I2C_SLV0_DO         0x06
#define ICM20648_I2C_SLV1_ADDR       0x07
#define ICM20648_I2C_SLV1_REG        0x08
#define ICM20648_I2C_SLV1_CTRL       0x09
#define ICM20648_I2C_SLV1_DO         0x0A
#define ICM20648_I2C_SLV2_ADDR       0x0B
#define ICM20648_I2C_SLV2_REG        0x0C
#define ICM20648_I2C_SLV2_CTRL       0x0D
#define ICM20648_I2C_SLV2_DO         0x0E
#define ICM20648_I2C_SLV3_ADDR       0x0F
#define ICM20648_I2C_SLV3_REG        0x10
#define ICM20648_I2C_SLV3_CTRL       0x11
#define ICM20648_I2C_SLV3_DO         0x12
#define ICM20648_I2C_SLV4_ADDR       0x13
#define ICM20648_I2C_SLV4_REG        0x14
#define ICM20648_I2C_SLV4_CTRL       0x15
#define ICM20648_I2C_SLV4_DO         0x16
#define ICM20648_I2C_SLV4_DI         0x17


//Acelerometro Config
#define ACCEL_SENSITIVITY_2G       16384
#define ACCEL_SENSITIVITY_4G       8192
#define ACCEL_SENSITIVITY_8G       4096
#define ACCEL_SENSITIVITY_16G      2048

#define ACCEL_2G_BIT       0b00
#define ACCEL_4G_BIT       0b01
#define ACCEL_8G_BIT       0b10
#define ACCEL_16G_BIT      0b11

#define ACC_AVG_4_SMPL       0b00
#define ACC_AVG_8_SMPL       0b01
#define ACC_AVG_16_SMPL      0b10
#define ACC_AVG_32_SMPL      0b11




//Giroscopo Config
#define GYRO_SENSITIVITY_250       131
#define GYRO_SENSITIVITY_500       65.5
#define GYRO_SENSITIVITY_1000      32.8
#define GYRO_SENSITIVITY_2000      16.4


#define GYRO_250DPS_BIT       0b00
#define GYRO_500DPS_BIT       0b01
#define GYRO_1000DPS_BIT      0b10
#define GYRO_2000DPS_BIT      0b11


#define GYRO_AVG_1_SMPL       0b000
#define GYRO_AVG_2_SMPL       0b001
#define GYRO_AVG_4_SMPL       0b010
#define GYRO_AVG_8_SMPL       0b011
#define GYRO_AVG_16_SMPL      0b100
#define GYRO_AVG_32_SMPL      0b101
#define GYRO_AVG_64_SMPL      0b110
#define GYRO_AVG_128_SMPL     0b111


//Temperatura Config
#define TEMP_SENSITIVITY       333.87


void ICM20648_Init() {

  //Despertar al sensor
  wakeup();
  selecionbank(0);

  //Seleccionar reloj
  seleccionreloj();

  //habilitar accelerometro HABRIA QUE PONER QUE LO LEA ANTES
  habilitacionacelerometro();

  //habilitar giroscopo
  habilitacionagiroscopo();

  //habilitar temperatura
  habilitaciontemp();

  //Comportamiento acelometro
  compacelerometro();

//Comportamiento acelometro
  compgiro();

  //DMP reset
  //resetDMP();

  //DMP enable
  //habilitacionDMP();

}

void ICM20648_Accel_Config() {
  //Habilitar filtro
  habilitacionfiltroacelerometro();
  //Configurar escala
  configescalaacelerometro();
  //Filtro acelerometro
  configfiltroacelerometro();
  //Autotest
  autotestacelerometro();
  //Configurar media de las muestras
  configmediamuestrasacelerometro();
  //Frecuencia de muestreo
  frecuenciamuestreoacceleracion(100);
}

void ICM20648_Giro_Config() {

  selecionbank(2);

  //Frecuencia muestreo Giroscopo (SOLO FUNCIONA SI ESTA ACTIVO EL FILTRO PASO BAJO)
  frecuenciamuestreogiroscopo(100);
  //Habilitar filtro
  habilitacionfiltrogiro();
  //Configurar escala
  configescalagiro();
  //Habilitar autotest
  autotesgiro();


  /*//MOSTRAR POR PANTALLA LA CADENA DE BITS
  printf("dato girocongif 00000001 (binario): ");
     for (int i = 7; i >= 0; i--) {
         printf("%d", (girocongif >> i) & 1);
     }
     printf("\n");
  */
//Volvemos a poner bank0
  selecionbank(0);

}

void ICM20648_Temp_Config() {
  //Se selecciona el user bank 2
  selecionbank(2);
  //Filtro paso bajo temperatura
  uint8_t tempDLPF = 0x00;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_TEMP_CONFIG, &tempDLPF,1) != i2cTransferDone) {
    printf("Error al configurar el filtro de la temperatura\n");
    return;
  }
  //Vovlemos a poner bank0
  selecionbank(0);

}

void ICM20648_ConfigureFIFO() {

  //Volvemos a poner bank0
  selecionbank(0);
  resetFIFO();
  escrituraFIFO();
  modeFIFO();
  habilitarFIFO();
}


void ICM20648_ConfigureDMP() {

  seleccionreloj();

//Deshabilitar accelerometro HABRIA QUE PONER QUE LO LEA ANTES
  habilitacionacelerometro();

  //Deshabilitar giroscopo
  habilitacionagiroscopo();

  //Deshabilitar temperatura
  habilitaciontemp();


//Configure Gyro/Accel in Low power mode with LP_CONFIG.
  compacelerometro();
  compgiro();

//Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1.
  configescalagiro();

//Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG.
  configescalaacelerometro();
/*
//Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2.
  habilitarinterrupcionOVFFIFO();

//Turn off what goes into the FIFO through FIFO_EN, FIFO_EN_2.
  uint8_t habilitarescritura = 0x00;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_EN_2, &habilitarescritura,1) != i2cTransferDone) {
    printf("Error al habilitar la escritura sobre interruciones FIFO\n");
    return;
  }


// Turn off data ready interrupt through INT_ENABLE_1.

  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_INT_ENABLE_1, &habilitarescritura,1) != i2cTransferDone) {
    printf("Error al habilitar la escritura de la FIFO\n");
    return;
  }

//Reset FIFO through FIFO_RST.
  resetFIFO();
*/
//Set gyro sample rate divider with GYRO_SMPLRT_DIV.
  frecuenciamuestreogiroscopo(100);

//Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  frecuenciamuestreoacceleracion(100);

//habilitar DMP

    habilitacionDMP();
 // Habilitar FIFO
    habilitarFIFO();
    escrituraFIFO();

}



void configureIMU() {
  //Despertar sensor (sacarlo de modo sleep)
  ICM20648_Init();

  if (ACELEROMETRO==1) {
    ICM20648_Accel_Config();
  }
  if (GIROSCOPO==1) {
    ICM20648_Giro_Config();
  }
  if (TEMPERATURA==1) {
    ICM20648_Temp_Config();
  }



  //Configurar los registros de configuraciC3n adicionales
  if (FIFOIMU==1) {
    ICM20648_ConfigureFIFO();
  }

  if (DMP==1) {
    ICM20648_ConfigureDMP();
  }

  //Leer muestras
  /*while (1) {
     if (ACELEROMETRO==1){
         ICM20648_ReadAcceleration();
       }
       if (GIROSCOPO==1){
           IICM20648_ReadGyro();
       }
       if (TEMPERATURA==1){
           ICM20648_ReadTemp();
         }

       ICM20648_ReadAcceleration();
       IICM20648_ReadGyro();
       ICM20648_ReadTemp();

    ICM20648_ReadFIFO();
  }*/
}

