/*
 * IMU_functions.c
 *
 *  Created on: 18 mar 2025
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

#define ICM20648_I2C_ADDRESS   0x68  // Dirección I2C del ICM-20648


//User Bank 0
#define ICM20648_WHO_AM_I            0x00
#define ICM20648_USER_CTRL           0x03
#define ICM20648_BIT_DMP_EN          0x80
#define ICM20648_BIT_FIFO_EN         0x40
#define ICM20648_BIT_DMP_RESET       0x08

#define ICM20648_LP_CONFIG           0x05
#define ICM20648_GYRO_CYCLE_BIT         0x10
#define ICM20648_ACCEL_CYCLE_BIT         0x20


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



void selecionbank(int bank) {

  uint8_t numbank = 0x00;
  switch (bank) {
  case 0:
    numbank =0x00;
    break;
  case 1:
    numbank =0x10;
    break;
  case 2:
    numbank =0x20;
    break;
  case 3:
    numbank =0x30;
    break;
  default:
    printf("Error al configurar el numero de banco\n");

  }

  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_REG_BANK_SEL, &numbank,1) != i2cTransferDone) {
    printf("Error al configurar el numero de banco\n");
    return;
  }

}

void reset() {
  selecionbank(0);
  uint8_t reset= ICM20648_BIT_H_RESET;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_1, &reset,1) != i2cTransferDone) {
    printf("Error al realizar el reset\n");
    return;
  }

  /* Wait 100ms to complete the reset sequence */
  // wait_ms(100);

}

void wakeup () {

  selecionbank(0);
  uint8_t dato =0x01;

  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_1, &dato,1) != i2cTransferDone) {
    printf("Error al despertar el sensor\n");
    return;
  }

}

void habilitacionacelerometro() {
  uint8_t dato_actual_mgmt_2;
  selecionbank(0);
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_2, &dato_actual_mgmt_2, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_PWR_MGMT_2 \n");
    return;
  }
  uint8_t dato1 =ICM20648_BIT_PWR_ACCEL_STBY; //Deshabilitar acelerometro 00111000


  if (ACELEROMETRO==0) { //Deshabilitar
    dato_actual_mgmt_2 = dato_actual_mgmt_2|dato1;  //OR para dejar el giroscopo en el mismo estado que antes
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_2, &dato_actual_mgmt_2,1) != i2cTransferDone) {
      printf("Error al deshabilitar el acelerometro\n");
      return;
    }

    printf("\n");
  }  else {  //Habilitar poniendo 0
    dato_actual_mgmt_2 = dato_actual_mgmt_2 &(~dato1);
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_2, &dato_actual_mgmt_2,1) != i2cTransferDone) {
      printf("Error al habilitar el acelerometro\n");
      return;
    }
  }

}

void habilitacionagiroscopo() {
  uint8_t dato_actual_mgmt_2;
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_2, &dato_actual_mgmt_2, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_PWR_MGMT_2 \n");
    return;
  }

  uint8_t dato1 = ICM20648_BIT_PWR_GYRO_STBY;// Deshabilitar giroscopo    00000111
  selecionbank(0);

  if (GIROSCOPO==0) { //Deshabilitar
    dato_actual_mgmt_2 = dato_actual_mgmt_2|dato1;  //OR para dejar el giroscopo en el mismo estado que antes
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_2, &dato_actual_mgmt_2,1) != i2cTransferDone) {
      printf("Error al deshabilitar el giroscopo\n");
      return;
    }

  }

  else {  //Habilitar poniendo 0
    dato_actual_mgmt_2 = dato_actual_mgmt_2 &(~dato1);
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_2, &dato_actual_mgmt_2,1) != i2cTransferDone) {
      printf("Error al habilitar el giroscopo\n");
      return;
    }
  }

  //Mostrar por pantalla pwr2
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_2, &dato_actual_mgmt_2, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_PWR_MGMT_2 \n");
    return;
  }
  //MOSTRAR POR PANTALLA LA CADENA DE BITS
  printf("dato dato_actual_mgmt_2 bo 00000111 (binario): ");
  for (int i = 7; i >= 0; i--) {
    printf("%d", (dato_actual_mgmt_2 >> i) & 1);
  }
  printf("\n");

}

void habilitaciontemp() {
  uint8_t dato_actual_mgmt_1;
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_1, &dato_actual_mgmt_1, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_PWR_MGMT_1 \n");
    return;
  }
  uint8_t dato1 =ICM20648_BIT_TEMP_DIS; //Deshabilitar temperatura 0x08
  selecionbank(0);

  if (TEMPERATURA==0) { //Deshabilitar
    dato_actual_mgmt_1 = dato_actual_mgmt_1|dato1;  //OR para dejar el giroscopo en el mismo estado que antes
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_1, &dato_actual_mgmt_1,1) != i2cTransferDone) {
      printf("Error al deshabilitar la temperatura\n");
      return;
    }
  }

  else {  //Habilitar poniendo 0
    dato_actual_mgmt_1 = dato_actual_mgmt_1 &(~dato1);
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_2, &dato_actual_mgmt_1,1) != i2cTransferDone) {
      printf("Error al habilitar la temperatura\n");
      return;
    }
  }


}

void frecuenciamuestreoacceleracion(float frecuencia) {

  selecionbank(2);
  uint16_t accelDiv;
  float accelfrecuencia;

  //Muestres Kh = 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])
  // Calculate the sample rate divider
  accelfrecuencia = (1125.0 / frecuencia) - 1.0;


  accelDiv = (uint16_t) accelfrecuencia;
  uint8_t accsamplerate1 = (uint8_t) (accelDiv >> 8);
  uint8_t accsamplerate2 = (uint8_t) (accelDiv & 0xFF);

  printf("dato muestreo 00101100 (binario): ");
  for (int i = 7; i >= 0; i--) {
    printf("%d", (accsamplerate2 >> i) & 1);
  }
  printf("\n");


  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_SMPLRT_DIV_1, &accsamplerate1,1) != i2cTransferDone) {
    printf("Error al configurar la frecuencia de muestreo del acelerometro\n");
    return;
  }

  //   write_register(ICM20648_REG_ACCEL_SMPLRT_DIV_2, (uint8_t) (accelDiv & 0xFF) );

  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_SMPLRT_DIV_2, &accsamplerate2,1) != i2cTransferDone) {
    printf("Error al configurar la frecuencia de muestreo del acelerometro\n");
    return;
  }


}

void configfiltroacelerometro() {
  uint8_t DLPF_config_bit = 0x38;
  uint8_t DLPF_value = 0b00111000; // Hz
  uint8_t dato_actual_accel_config;
  selecionbank(2);

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG, &dato_actual_accel_config, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_ACCEL_CONFIG \n");
    return;
  }

  //pongo a 0 los bits de los valores
  dato_actual_accel_config = dato_actual_accel_config &(~DLPF_config_bit);
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG, &dato_actual_accel_config,1) != i2cTransferDone) {
    printf("Error al poner a 0 bits filtro paso bajo acelerometro\n");
    return;
  }
  //vuelo a leer
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG, &dato_actual_accel_config, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_ACCEL_CONFIG \n");
    return;
  }
  //Configuro con el nuevo valor
  dato_actual_accel_config = dato_actual_accel_config |DLPF_value; ;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG, &dato_actual_accel_config,1) != i2cTransferDone) {
    printf("Error al configurar el filtro del acelerometro\n");
    return;
  }
  //vuelo a leer
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG, &dato_actual_accel_config, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_ACCEL_CONFIG \n");
    return;
  }

  //MOSTRAR POR PANTALLA LA CADENA DE BITS
  printf("dato accelconfig 00101001 (binario): ");
  for (int i = 7; i >= 0; i--) {
    printf("%d", (dato_actual_accel_config >> i) & 1);
  }
  printf("\n");
}

void habilitacionfiltroacelerometro() {
  uint8_t DLPF_enable = 1; //a 1 se habilita filtro
  uint8_t DLPF_enable_bit = 0x01;
  uint8_t dato_actual_accel_config;
  selecionbank(2);


  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG, &dato_actual_accel_config, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_PWR_MGMT_1 \n");
    return;
  }



  // if ((dato_actual_accel_config&0b1)==0){ //ultimo bit es 0 Habilitar filtro
  if (DLPF_enable) { //ultimo bit es 0 Habilitar filtro
    dato_actual_accel_config = dato_actual_accel_config|DLPF_enable_bit; //pongo un 1 en el ultimo bit
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG, &dato_actual_accel_config,1) != i2cTransferDone) {
      printf("Error al habilitar el acelerometro\n");
      return;
    }
    configfiltroacelerometro();
  }

  else {  //Deshabilitar poniendo 0
    dato_actual_accel_config = dato_actual_accel_config &(~DLPF_enable_bit);
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG, &dato_actual_accel_config,1) != i2cTransferDone) {
      printf("Error al deshabilitar el acelerometro\n");
      return;
    }
  }
}

void configescalaacelerometro() {
  uint8_t escala_config_bit = 0x60;
  uint8_t acel_escala = ACCEL_4G_BIT;
  acel_escala =(acel_escala << 1);
  uint8_t dato_actual_accel_config;
  selecionbank(2);

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG, &dato_actual_accel_config, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_ACCEL_CONFIG \n");
    return;
  }

  //pongo a 0 los bits de los valores
  dato_actual_accel_config = dato_actual_accel_config &(~escala_config_bit);
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG, &dato_actual_accel_config,1) != i2cTransferDone) {
    printf("Error al poner a 0 bits escala acelerometro\n");
    return;
  }
  //vuelo a leer
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG, &dato_actual_accel_config, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_ACCEL_CONFIG \n");
    return;
  }
  //Configuro con el nuevo valor
  dato_actual_accel_config = dato_actual_accel_config |acel_escala; ;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG, &dato_actual_accel_config,1) != i2cTransferDone) {
    printf("Error al configurar la escala el acelerometro\n");
    return;
  }
}

void autotestacelerometro() {
  uint8_t selftestenable = 0;
  uint8_t selftestbits = 0b00011100;
  uint8_t dato_actual_accel_config2;
  selecionbank(2);

  if (selftestenable==0) {

    if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG_2, &dato_actual_accel_config2, 1) != i2cTransferDone) {
      printf("Error al leer ICM20648_ACCEL_CONFIG2 \n");
      return;
    }

    //pongo a 0 los bits de los valores y deshabilito el selftest
    dato_actual_accel_config2 = dato_actual_accel_config2 &(~selftestbits);
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG_2, &dato_actual_accel_config2,1) != i2cTransferDone) {
      printf("Error al poner a 0 bits escala acelerometro\n");
      return;
    }
  } else {
    //habilito el selftest
    dato_actual_accel_config2 = dato_actual_accel_config2 |selftestbits; ;
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG_2, &dato_actual_accel_config2,1) != i2cTransferDone) {
      printf("Error al configurar la escala el acelerometro\n");
      return;
    }
  }

}


void  configmediamuestrasacelerometro() {
  //La media funciona si el filtro paso bajo estC! activo
  uint8_t mediamuestrasbit = 0b00000011;
  uint8_t avgsample = ACC_AVG_4_SMPL;
  uint8_t dato_actual_accel_config2;
  selecionbank(2);

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG_2, &dato_actual_accel_config2, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_ACCEL_CONFIG2 \n");
    return;
  }

  //pongo a 0 los bits de los valores
  dato_actual_accel_config2 = dato_actual_accel_config2 &(~mediamuestrasbit);
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG_2, &dato_actual_accel_config2,1) != i2cTransferDone) {
    printf("Error al poner a 0 bits de la media de las muestras acelerometro\n");
    return;
  }
  //vuelo a leer
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG_2, &dato_actual_accel_config2, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_ACCEL_CONFIG_2 \n");
    return;
  }
  //Configuro con el nuevo valor
  dato_actual_accel_config2 = dato_actual_accel_config2|avgsample;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG_2, &dato_actual_accel_config2,1) != i2cTransferDone) {
    printf("Error al configurar la media de las muestras\n");
    return;
  }

  //MOSTRAR POR PANTALLA LA CADENA DE BITS
  printf("dato accelconfig2 bo 00000000 (binario): ");
  for (int i = 7; i >= 0; i--) {
    printf("%d", (dato_actual_accel_config2 >> i) & 1);
  }
  printf("\n");

}

void frecuenciamuestreogiroscopo(float frecuencia) {
  //This register is only effective when FCHOICE = 1b
  selecionbank(2);
  //Muestras Kh = 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])
  // Calculate the sample rate divider
  uint8_t girosamplerate = (1125.0 / frecuencia) - 1.0;



  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_SMPLRT_DIV, &girosamplerate,1) != i2cTransferDone) {
    printf("Error al configurar la frecuencia de muestreo del giroscopo\n");
    return;
  }

}

void configfiltrogiro() {
  uint8_t DLPF_config_bit = 0x38;
  uint8_t DLPF_value = 0b00111000; //23.9 Hz
  uint8_t dato_actual_giro_config;
  selecionbank(2);

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_1, &dato_actual_giro_config, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_GYRO_CONFIG_1 \n");
    return;
  }

  //pongo a 0 los bits de los valores
  dato_actual_giro_config = dato_actual_giro_config &(~DLPF_config_bit);
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_1, &dato_actual_giro_config,1) != i2cTransferDone) {
    printf("Error al poner a 0 bits filtro paso bajo giroscopo\n");
    return;
  }
  //vuelo a leer
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_1, &dato_actual_giro_config, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_GYRO_CONFIG_1 \n");
    return;
  }
  //Configuro con el nuevo valor
  dato_actual_giro_config = dato_actual_giro_config |DLPF_value; ;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_1, &dato_actual_giro_config,1) != i2cTransferDone) {
    printf("Error al configurar el filtro del giroscopo\n");
    return;
  }
}

void habilitacionfiltrogiro() {
  uint8_t DLPF_enable = 1; //a 1 se habilita filtro
  uint8_t DLPF_enable_bit = 0x01;
  uint8_t dato_actual_giro_config;
  selecionbank(2);

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_1, &dato_actual_giro_config, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_GYRO_CONFIG_1 \n");
    return;
  }

  if (DLPF_enable) { //ultimo bit es 0 Habilitar filtro
    dato_actual_giro_config = dato_actual_giro_config|DLPF_enable_bit; //pongo un 1 en el ultimo bit
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_1, &dato_actual_giro_config,1) != i2cTransferDone) {
      printf("Error al habilitar el giroscopo\n");
      return;
    }
    configfiltrogiro();
  }

  else {  //Deshabilitar poniendo 0
    dato_actual_giro_config = dato_actual_giro_config &(~DLPF_enable_bit);
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_1, &dato_actual_giro_config,1) != i2cTransferDone) {
      printf("Error al deshabilitar el giroscopo\n");
      return;
    }
  }
}

void configescalagiro() {
  uint8_t escala_config_bit = 0x60;
  uint8_t acel_escala = GYRO_500DPS_BIT;
  acel_escala =(acel_escala << 1);
  uint8_t dato_actual_giro_config;
  selecionbank(2);

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_1, &dato_actual_giro_config, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_GYRO_CONFIG_1 \n");
    return;
  }

  //pongo a 0 los bits de los valores
  dato_actual_giro_config = dato_actual_giro_config &(~escala_config_bit);
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_1, &dato_actual_giro_config,1) != i2cTransferDone) {
    printf("Error al poner a 0 bits escala giroscopo\n");
    return;
  }
  //vuelvo a leer
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_1, &dato_actual_giro_config, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_GYRO_CONFIG_1 \n");
    return;
  }
  //Configuro con el nuevo valor
  dato_actual_giro_config = dato_actual_giro_config |acel_escala;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_1, &dato_actual_giro_config,1) != i2cTransferDone) {
    printf("Error al configurar la escala el giroscopo\n");
    return;
  }
}

void autotesgiro() {
  uint8_t selftestenable = 0;
  uint8_t selftestbits = 0b00111000;
  uint8_t dato_actual_giro_config2;
  selecionbank(2);

  if (selftestenable==0) {

    if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_2, &dato_actual_giro_config2, 1) != i2cTransferDone) {
      printf("Error al leer ICM20648_ACCEL_CONFIG2 \n");
      return;
    }
    //pongo a 0 los bits de los valores y deshabilito el selftest
    dato_actual_giro_config2 = dato_actual_giro_config2 &(~selftestbits);
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_2, &dato_actual_giro_config2,1) != i2cTransferDone) {
      printf("Error al poner a 0 bits escala acelerometro\n");
      return;
    }
  } else {
    //habilito el selftest
    dato_actual_giro_config2 = dato_actual_giro_config2 |selftestbits; ;
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_2, &dato_actual_giro_config2,1) != i2cTransferDone) {
      printf("Error al configurar la escala el acelerometro\n");
      return;
    }
  }

}

void  configmediamuestrasgiro() {
  //La media funciona si el filtro paso bajo estC! activo
  uint8_t mediamuestrasbit = 0b11000000;
  uint8_t avgsample = GYRO_AVG_2_SMPL;
  uint8_t dato_actual_giro_config2;
  selecionbank(2);

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_2, &dato_actual_giro_config2, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_GYRO_CONFIG_2 \n");
    return;
  }

  //pongo a 0 los bits de los valores
  dato_actual_giro_config2 = dato_actual_giro_config2 &(~mediamuestrasbit);
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_2, &dato_actual_giro_config2,1) != i2cTransferDone) {
    printf("Error al poner a 0 bits de la media de las muestras acelerometro\n");
    return;
  }
  //vuelo a leer
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_CONFIG_2, &dato_actual_giro_config2, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_GYRO_CONFIG_2 \n");
    return;
  }
  //Configuro con el nuevo valor
  dato_actual_giro_config2 = dato_actual_giro_config2|avgsample;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_CONFIG_2, &dato_actual_giro_config2,1) != i2cTransferDone) {
    printf("Error al configurar la media de las muestras giroscopo\n");
    return;
  }
}

void compacelerometro() {
//Si se llama a esta fucion directamente se activa comportamiento frecuencia muestro
  uint8_t accel_cycle = ICM20648_ACCEL_CYCLE_BIT;
  uint8_t dato_actual_lpconfig;

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_LP_CONFIG, &dato_actual_lpconfig, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_LP_CONFIG \n");
    return;
  }
//habilitar
  dato_actual_lpconfig = dato_actual_lpconfig|accel_cycle;  //OR para dejar el giroscopo en el mismo estado que antes
  if (writeI2C(ICM20648_I2C_ADDRESS,  ICM20648_LP_CONFIG, &dato_actual_lpconfig,1) != i2cTransferDone) {
    printf("Error al habilitar la frecuencia muetreo acelerometro\n");
    return;
  }

}

void compgiro() {
//Si se llama a esta fucion directamente se activa comportamiento frecuencia muestro
  uint8_t giro_cycle = ICM20648_GYRO_CYCLE_BIT;
  uint8_t dato_actual_lpconfig;

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_LP_CONFIG, &dato_actual_lpconfig, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_LP_CONFIG \n");
    return;
  }
//habilitar
  dato_actual_lpconfig = dato_actual_lpconfig|giro_cycle;  //OR para dejar el giroscopo en el mismo estado que antes
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_LP_CONFIG, &dato_actual_lpconfig,1) != i2cTransferDone) {
    printf("Error al habilitar la frecuencia muetreo acelerometro\n");
    return;
  }

}

void seleccionreloj() {
  uint8_t dato_actual_mgmt_1;
  selecionbank(0);
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_1, &dato_actual_mgmt_1, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_PWR_MGMT_1 \n");
    return;
  }
  uint8_t relojbits = ICM20648_BIT_CLK;
  uint8_t reloj = INTERNAL_CLK_2;

  dato_actual_mgmt_1 =dato_actual_mgmt_1&(~relojbits);

  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_1, &dato_actual_mgmt_1,1) != i2cTransferDone) {
    printf("Error al poner a 0 bits del reloj\n");
    return;
  }

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_1, &dato_actual_mgmt_1, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_PWR_MGMT_1 \n");
    return;
  }

  dato_actual_mgmt_1 =dato_actual_mgmt_1|reloj;

  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_PWR_MGMT_1, &dato_actual_mgmt_1,1) != i2cTransferDone) {
    printf("Error al configurar la frecuencia de muestreo del acelerometro\n");
    return;
  }

}

void resetDMP() {
  selecionbank(0);
  uint8_t DMPbitreset =ICM20648_BIT_DMP_RESET;
  uint8_t dato_actual_user;

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_USER_CTRL, &dato_actual_user, 1) != i2cTransferDone) {
    printf("Error al leer dato_actual_user \n");
    return;
  }

  dato_actual_user = dato_actual_user|DMPbitreset;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_USER_CTRL, &dato_actual_user,1) != i2cTransferDone) {
    printf("Error al deshabilitar la temperatura\n");
    return;
  }

}

void  habilitacionDMP() {
  selecionbank(0);
  uint8_t DMPbit =ICM20648_BIT_DMP_EN;
  uint8_t dato_actual_user;

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_USER_CTRL, &dato_actual_user, 1) != i2cTransferDone) {
    printf("Error al leer dato_actual_user \n");
    return;
  }


  if(DMP) { //Habilitar

    dato_actual_user = dato_actual_user|DMPbit;
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_USER_CTRL, &dato_actual_user,1) != i2cTransferDone) {
      printf("Error al deshabilitar la temperatura\n");
      return;
    }
  }

  else {  //Deshabilitar poniendo 0
    dato_actual_user = dato_actual_user &(~DMPbit); //Poner a 0
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_USER_CTRL, &dato_actual_user,1) != i2cTransferDone) {
      printf("Error al habilitar la temperatura\n");
      return;
    }
  }

}

void ICM20648_ReadAcceleration() {
  uint8_t rawData[6];
  float acelsensibilidad = ACCEL_SENSITIVITY_4G ;


  // Leer los 6 bytes del acelerC3metro (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_ACCEL_XOUT_H, rawData, 6) != i2cTransferDone) {
    printf("Error al leer datos del acelerC3metro\n");
    return;
  }

  // Convertir los datos a valores de 16 bits en complemento a 2
  int16_t accelX = (rawData[0] << 8) | rawData[1];
  int16_t accelY = (rawData[2] << 8) | rawData[3];
  int16_t accelZ = (rawData[4] << 8) | rawData[5];

  // Convertir a unidades de g
  float Ax = accelX / acelsensibilidad;
  float Ay = accelY / acelsensibilidad;
  float Az = accelZ / acelsensibilidad;

  // Imprimir los valores obtenidos
  printf("AceleraciC3n: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Ax, Ay, Az);
}

void ICM20648_ReadGyro() {
  uint8_t rawData2[6];
  uint8_t girosensibilidad = GYRO_SENSITIVITY_500;



  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_GYRO_XOUT_H, rawData2, 6) != i2cTransferDone) {
    printf("Error al leer datos del acelerC3metro\n");
    return;
  }

  int16_t gyroX = (rawData2[0] << 8) | rawData2[1];
  int16_t gyroY = (rawData2[2] << 8) | rawData2[3];
  int16_t gyroZ = (rawData2[4] << 8) | rawData2[5];


  float Gx = gyroX / girosensibilidad;
  float Gy = gyroY / girosensibilidad;
  float Gz = gyroZ / girosensibilidad;

  printf("Giroscopio: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Gx, Gy, Gz);

}

void ICM20648_ReadTemp() {
  uint8_t rawData[2];

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_TEMP_OUT_H, rawData, 2) == 0) {
    int16_t tempRaw = (rawData[0] << 8) | rawData[1];
    float tempCelsius = (((float)tempRaw -21) / TEMP_SENSITIVITY) + 21.0;  // ConversiC3n segC:n el datasheet

// *temperature = ( (float) raw_temp / 333.87) + 21.0;

    printf("Temperatura: %.2f B0C\n", tempCelsius);
  } else {
    printf("Error al leer temperatura\n");
  }
}

void habilitarFIFO() {
  uint8_t FIFObit =ICM20648_BIT_FIFO_EN;
  uint8_t dato_actual_user;
  selecionbank(0);

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_USER_CTRL, &dato_actual_user, 1) != i2cTransferDone) {
    printf("Error al leer dato_actual_user \n");
    return;
  }


  if (FIFOIMU==0) { //Deshabilitar
    dato_actual_user = dato_actual_user &(~FIFObit); //Poner a 0

    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_USER_CTRL, &dato_actual_user,1) != i2cTransferDone) {
      printf("Error al deshabilitar la FIFO\n");
      return;
    }
  }

  else {  //Habilitar poniendo 1
    dato_actual_user = dato_actual_user|FIFObit;
    if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_USER_CTRL, &dato_actual_user,1) != i2cTransferDone) {
      printf("Error al habilitar la FIFO\n");
      return;
    }
  }
  //MOSTRAR POR PANTALLA LA CADENA DE BITS
  printf("dato FIFO enable 01000000 (binario): ");
  for (int i = 7; i >= 0; i--) {
    printf("%d", (dato_actual_user >> i) & 1);
  }
  printf("\n");

}

void modeFIFO() {
  //Comportamiento ante nuevas muestras cuando esta llena
  selecionbank(0);
  uint8_t fifomode = 0x00; //0 se reemplazan las muestras viejas por nuevas

  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_MODE, &fifomode,1) != i2cTransferDone) {
    printf("Error al habilitar la escritura de la FIFO\n");
    return;
  }
}

void habilitarinterrupcionOVFFIFO() {
  selecionbank(0);
  uint8_t dato_actual_int2;
  uint8_t fifoOVFBIT=0x01;

  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_INT_ENABLE_2, &dato_actual_int2, 1) != i2cTransferDone) {
    printf("Error al leer ICM20648_INT_ENABLE_2 \n");
    return;
  }

  dato_actual_int2 = dato_actual_int2|fifoOVFBIT;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_INT_ENABLE_2, &dato_actual_int2,1) != i2cTransferDone) {
    printf("Error al configurar OVF FIFO\n");
    return;
  }
}

void resetFIFO() {
  selecionbank(0);
  uint8_t resetFIFO = 0x0F;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_RST, &resetFIFO,1) != i2cTransferDone) {
    printf("Error al resetear la FIFO\n");
    return;
  }
  for(int i=0; i<1000; i++)

    resetFIFO = 0x00;
  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_RST, &resetFIFO,1) != i2cTransferDone) {
    printf("Error al resetear la FIFO\n");
    return;
  }
}

void escrituraFIFO() {
//Sensores que escriben en la FIFO
  selecionbank(0);
  uint8_t habilitarescritura;
  habilitarescritura = (ACCEL_FIFO_EN << 4) | (GYRO_Z_FIFO_EN << 3) | (GYRO_Y_FIFO_EN << 2) | (GYRO_X_FIFO_EN << 1) | TEMP_FIFO_EN;
  habilitarescritura = 0x00 | habilitarescritura;


//MOSTRAR POR PANTALLA LA CADENA DE BITS
  /*printf("dato habilitarescritura 00010001 (binario): ");
  for (int i = 7; i >= 0; i--) {
    printf("%d", (habilitarescritura >> i) & 1);
  }
  printf("\n");
*/

  if (writeI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_EN_2, &habilitarescritura,1) != i2cTransferDone) {
    printf("Error al habilitar la escritura de la FIFO\n");
    return;
  }

}

void ICM20648_ReadFIFO() {
  selecionbank(0);
  uint8_t rawData[2];
  int16_t numBytesFIFO;
  uint16_t numBytesMuestras=0;
  //Leo el numero de BYTES disponibles para leer:
  if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_COUNTH,rawData, 2) != i2cTransferDone) {
    printf("Error al leer los BYTES disponibles de la FIFO\n");
  }
  numBytesFIFO = (rawData[0] << 8) | rawData[1];
  //printf("numBytesFIFO: X = %d\n", numBytesFIFO);
  //Procesamiento de las muestras leidas

  if (ACCEL_FIFO_EN) numBytesMuestras += 6; // 6 bytes para el acelerC3metro
  if (GYRO_X_FIFO_EN) numBytesMuestras += 2;  // 2 bytes para cada eje del giroscopio
  if (GYRO_Y_FIFO_EN) numBytesMuestras += 2;
  if (GYRO_Z_FIFO_EN) numBytesMuestras += 2;
  if (TEMP_FIFO_EN) numBytesMuestras += 2;  // 2 bytes para la temperatura
  //printf("numBytesmuestras: X = %d\n", numBytesMuestras);
  //inicializacion
  uint8_t girosensibilidad = GYRO_SENSITIVITY_500;
  float acelsensibilidad = ACCEL_SENSITIVITY_4G;
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
  int16_t tempRaw;
  float Ax;
  float Ay;
  float Az;
  float Gx;
  float Gy;
  float Gz;
  float tempCelsius;

  uint16_t numMuestras = numBytesFIFO/numBytesMuestras;
  uint8_t buffer [numBytesMuestras];

 // printf("numMuestras: X = %d\n", numMuestras);

  if (numBytesMuestras == 14) {
    //printf("dentro 14 bytes\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      //Leo una muestra
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,14) != i2cTransferDone) {
        printf("Error al leer FIFO IMU \n");
        return;
      }

      accelX = (buffer[0] << 8) | buffer[1];
      accelY = (buffer[2] << 8) | buffer[3];
      accelZ = (buffer[4] << 8) | buffer[5];

      // Convertir a unidades de g
      Ax = accelX / acelsensibilidad;
      Ay = accelY / acelsensibilidad;
      Az = accelZ / acelsensibilidad;
      printf("IMU: AceleraciC3n: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Ax, Ay, Az);

      gyroX = (buffer[6] << 8) | buffer[7];
      gyroY = (buffer[8] << 8) | buffer[9];
      gyroZ = (buffer[10] << 8)| buffer[11];

      Gx = gyroX / girosensibilidad;
      Gy = gyroY / girosensibilidad;
      Gz = gyroZ / girosensibilidad;
      printf("IMU: Giroscopio: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Gx, Gy, Gz);

      tempRaw = (buffer[12] << 8) | buffer[13];
      tempCelsius = (((float)tempRaw -21) / TEMP_SENSITIVITY) + 21.0;
      printf("IMU: Temperatura: %.2f B0C\n", tempCelsius);
    }

  }

  else if (numBytesMuestras == 12) {
    //printf("dentro 12 muesstras\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,12) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      accelX = (buffer[0] << 8) | buffer[1];
      accelY = (buffer[2] << 8) | buffer[3];
      accelZ = (buffer[4] << 8) | buffer[5];

      // Convertir a unidades de g
      Ax = accelX / acelsensibilidad;
      Ay = accelY / acelsensibilidad;
      Az = accelZ / acelsensibilidad;
      printf("IMU: Acel: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Ax, Ay, Az);

      gyroX = (buffer[6] << 8) | buffer[7];
      gyroY = (buffer[8] << 8) | buffer[9];
      gyroZ = (buffer[10] << 8)| buffer[11];

      Gx = gyroX / girosensibilidad;
      Gy = gyroY / girosensibilidad;
      Gz = gyroZ / girosensibilidad;
      printf("IMU: Giros: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Gx, Gy, Gz);
    }
  }

  else if ((numBytesMuestras == 8)&&(ACCEL_FIFO_EN==1)&&(TEMP_FIFO_EN==1)) {
    printf("dentro 8 muestras\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,8) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      accelX = (buffer[0] << 8) | buffer[1];
      accelY = (buffer[2] << 8) | buffer[3];
      accelZ = (buffer[4] << 8) | buffer[5];

      // Convertir a unidades de g
      Ax = accelX / acelsensibilidad;
      Ay = accelY / acelsensibilidad;
      Az = accelZ / acelsensibilidad;
      printf("IMU: AceleraciC3n: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Ax, Ay, Az);

      tempRaw = (buffer[6] << 8) | buffer[7];
      tempCelsius = (((float)tempRaw -21) / TEMP_SENSITIVITY) + 21.0;
      printf("IMU: Temperatura: %.2f B0C\n", tempCelsius);
    }
  }

  else if ((numBytesMuestras == 8)&&(ACCEL_FIFO_EN==0)&&(TEMP_FIFO_EN==1)) {
   // printf("dentro 8 muestras\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,8) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      gyroX = (buffer[0] << 8) | buffer[1];
      gyroY = (buffer[2] << 8) | buffer[3];
      gyroZ = (buffer[4] << 8) | buffer[5];

      Gx = gyroX / girosensibilidad;
      Gy = gyroY / girosensibilidad;
      Gz = gyroZ / girosensibilidad;

      printf("IMU: Giroscopio: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Gx, Gy, Gz);

      tempRaw = (buffer[6] << 8) | buffer[7];
      tempCelsius = (((float)tempRaw -21) / TEMP_SENSITIVITY) + 21.0;
      printf("IMU: Temperatura: %.2f B0C\n", tempCelsius);
    }
  }

  else if ((numBytesMuestras == 8)&&(ACCEL_FIFO_EN==1)&&(GYRO_X_FIFO_EN==1)) {
   // printf("dentro 8 muestras\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,8) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      accelX = (buffer[0] << 8) | buffer[1];
      accelY = (buffer[2] << 8) | buffer[3];
      accelZ = (buffer[4] << 8) | buffer[5];

      // Convertir a unidades de g
      Ax = accelX / acelsensibilidad;
      Ay = accelY / acelsensibilidad;
      Az = accelZ / acelsensibilidad;
      printf("IMU: AceleraciC3n: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Ax, Ay, Az);

      gyroX = (buffer[6] << 8) | buffer[7];
      Gx = gyroX / girosensibilidad;
      printf("IMU: Giroscopio: X = %.3f g\n", Gx);
    }
  }

  else if ((numBytesMuestras == 8)&&(ACCEL_FIFO_EN==1)&&(GYRO_Y_FIFO_EN==1)) {
    //printf("dentro 8 muestras\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,8) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      accelX = (buffer[0] << 8) | buffer[1];
      accelY = (buffer[2] << 8) | buffer[3];
      accelZ = (buffer[4] << 8) | buffer[5];

      // Convertir a unidades de g
      Ax = accelX / acelsensibilidad;
      Ay = accelY / acelsensibilidad;
      Az = accelZ / acelsensibilidad;
      printf("IMU: AceleraciC3n: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Ax, Ay, Az);

      gyroY = (buffer[6] << 8) | buffer[7];
      Gy = gyroY / girosensibilidad;
      printf("IMU: Giroscopio: Y = %.3f g\n", Gy);
    }
  }

  else if ((numBytesMuestras == 8)&&(ACCEL_FIFO_EN==1)&&(GYRO_Z_FIFO_EN==1)) {
    //printf("dentro 8 muestras\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,8) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      accelX = (buffer[0] << 8) | buffer[1];
      accelY = (buffer[2] << 8) | buffer[3];
      accelZ = (buffer[4] << 8) | buffer[5];

      // Convertir a unidades de g
      Ax = accelX / acelsensibilidad;
      Ay = accelY / acelsensibilidad;
      Az = accelZ / acelsensibilidad;
      printf("IMU: AceleraciC3n: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Ax, Ay, Az);

      gyroZ = (buffer[6] << 8) | buffer[7];
      Gz = gyroZ / girosensibilidad;
      printf("IMU:Giroscopio: Y = %.3f g\n", Gz);
    }
  }


  else if ((numBytesMuestras == 6)&&(ACCEL_FIFO_EN==1)) {
   // printf("dentro 6 muestras y acelermetro\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,6) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }

      accelX = (buffer[0] << 8) | buffer[1];
      accelY = (buffer[2] << 8) | buffer[3];
      accelZ = (buffer[4] << 8) | buffer[5];

      // Convertir a unidades de g
      Ax = accelX / acelsensibilidad;
      Ay = accelY / acelsensibilidad;
      Az = accelZ / acelsensibilidad;
      printf("IMU: AceleraciC3n: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Ax, Ay, Az);
    }
  }

  else if ((numBytesMuestras == 6)&&(GYRO_X_FIFO_EN)&&(GYRO_Y_FIFO_EN)&&(GYRO_Z_FIFO_EN)) {
    //printf("dentro 6 muesstras y giro completo\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,6) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      gyroX = (buffer[0] << 8) | buffer[1];
      gyroY = (buffer[2] << 8) | buffer[3];
      gyroZ = (buffer[4] << 8) | buffer[5];

      Gx = gyroX / girosensibilidad;
      Gy = gyroY / girosensibilidad;
      Gz = gyroZ / girosensibilidad;
      printf("IMU: Giroscopio: X = %.3f g, Y = %.3f g, Z = %.3f g\n", Gx, Gy, Gz);
    }

  }

  else if ((numBytesMuestras == 6)&&(GYRO_X_FIFO_EN==0)&&(GYRO_Y_FIFO_EN)&&(GYRO_Z_FIFO_EN)) {
    //printf("dentro 6 muesstras y giro completo\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,6) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      gyroY = (buffer[0] << 8) | buffer[1];
      gyroZ = (buffer[2] << 8) | buffer[3];

      Gy = gyroY / girosensibilidad;
      Gz = gyroZ / girosensibilidad;
      printf("IMU: Giroscopio: Y = %.3f g, Z = %.3f g\n", Gy, Gz);

      tempRaw = (buffer[4] << 8) | buffer[5];
      tempCelsius = (((float)tempRaw -21) / TEMP_SENSITIVITY) + 21.0;
      printf("IMU: Temperatura: %.2f B0C\n", tempCelsius);
    }

  }
  else if ((numBytesMuestras == 6)&&(GYRO_X_FIFO_EN)&&(GYRO_Y_FIFO_EN==0)&&(GYRO_Z_FIFO_EN)) {
   // printf("dentro 6 muesstras y giro completo\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,6) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      gyroX = (buffer[0] << 8) | buffer[1];
      gyroZ = (buffer[2] << 8) | buffer[3];

      Gx = gyroX / girosensibilidad;
      Gz = gyroZ / girosensibilidad;
      printf("IMU: Giroscopio: X = %.3f g, Z = %.3f g\n", Gx, Gz);

      tempRaw = (buffer[4] << 8) | buffer[5];
      tempCelsius = (((float)tempRaw -21) / TEMP_SENSITIVITY) + 21.0;
      printf("IMU: Temperatura: %.2f B0C\n", tempCelsius);
    }

  }

  else if ((numBytesMuestras == 6)&&(GYRO_X_FIFO_EN)&&(GYRO_Y_FIFO_EN)&&(GYRO_Z_FIFO_EN==0)) {
    //printf("dentro 6 muesstras y giro completo\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,6) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      gyroX = (buffer[0] << 8) | buffer[1];
      gyroY = (buffer[2] << 8) | buffer[3];

      Gx = gyroX / girosensibilidad;
      Gy = gyroY / girosensibilidad;
      printf("IMU: Giroscopio: X = %.3f g, Y = %.3f g\n", Gx, Gy);

      tempRaw = (buffer[4] << 8) | buffer[5];
      tempCelsius = (((float)tempRaw -21) / TEMP_SENSITIVITY) + 21.0;
      printf("IMU: Temperatura: %.2f B0C\n", tempCelsius);
    }
  }
  else if ((numBytesMuestras == 2)&&(TEMP_FIFO_EN)) {
   // printf("dentro 6 muesstras y giro completo\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,2) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      tempRaw = (buffer[0] << 8) | buffer[1];
      tempCelsius = (((float)tempRaw -21) / TEMP_SENSITIVITY) + 21.0;
      printf("IMU: Temperatura: %.2f B0C\n", tempCelsius);
    }

  }
  else if ((numBytesMuestras == 2)&&(GYRO_X_FIFO_EN)) {
    //printf("dentro 2 muestras con giro X\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,2) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      gyroX = (buffer[0] << 8) | buffer[1];
      Gx = gyroX / girosensibilidad;
      printf("IMU: Giroscopio: X = %.3f g \n", Gx);

    }
  }
  else if ((numBytesMuestras == 2)&&(GYRO_Y_FIFO_EN)) {
    //printf("dentro 2 muestras con giro X\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,2) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      gyroY = (buffer[0] << 8) | buffer[1];
      Gy = gyroY / girosensibilidad;
      printf("IMU: Giroscopio: Y = %.3f g \n", Gy);
    }
  }
  else if ((numBytesMuestras == 2)&&(GYRO_Z_FIFO_EN)) {
    //printf("dentro 2 muestras con giro X\n");
    for (uint8_t i = 0; i < numMuestras; i++) {
      if (readI2C(ICM20648_I2C_ADDRESS, ICM20648_FIFO_R_W, buffer,2) != i2cTransferDone) {
        printf("Error al leer FIFO \n");
        return;
      }
      gyroZ = (buffer[0] << 8) | buffer[1];
      Gz = gyroZ / girosensibilidad;
      printf("IMU: Giroscopio: Z = %.3f g \n", Gz);
    }
  }

}

