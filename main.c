/***************************************************************************//**
 * @file
 * @brief main() function.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "sl_component_catalog.h"
#include "sl_system_init.h"
#include "app.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#include "sl_power_manager.h"
#endif
#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "sl_system_kernel.h"
#else // SL_CATALOG_KERNEL_PRESENT
#include "sl_system_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT

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


#define PWM_FREQ 1000

// Definir el puerto y el pin para el LED que se encenderá en el main
#define MAIN_LED_PORT gpioPortA
#define MAIN_LED_PIN 6

// Definir el puerto y el pin para el LED que se encenderá en la interrupción
#define TIMER_LED_PORT gpioPortA
#define TIMER_LED_PIN 5

// Definir pines habilitacion IMU
#define IMU_ENABLE_PORT gpioPortF
#define IMU_ENABLE_PIN 11

// Sensor temperatura
#define TEMP_ENABLE_PORT gpioPortF
#define TEMP_ENABLE_PIN 9
#define I2C_SCL_PORT     gpioPortC
#define I2C_SCL_PIN      11
#define I2C_SDA_PORT     gpioPortC
#define I2C_SDA_PIN      10

volatile float temperature = 0.0;

void initGPIO(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  // Configure PA6 as output
  GPIO_PinModeSet(MAIN_LED_PORT, MAIN_LED_PIN, gpioModePushPull, 0);
  // Configurar el LED del puerto A, pin 5 como salida
  GPIO_PinModeSet(TIMER_LED_PORT , TIMER_LED_PIN, gpioModePushPull, 0);
  if (TEMPSENSOR==1){
     GPIO_PinModeSet(TEMP_ENABLE_PORT , TEMP_ENABLE_PIN, gpioModePushPull, 0);
     GPIO_PinOutSet(TEMP_ENABLE_PORT, TEMP_ENABLE_PIN);
  }
   // Encender el LED del puerto A, pin 5
    GPIO_PinOutSet(MAIN_LED_PORT, MAIN_LED_PIN);
}


int main(void)
{
  // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
  // Note that if the kernel is present, processing task(s) will be created by
  // this call.
  sl_system_init();
  CHIP_Init();
  // Initialize the application. For example, create periodic timer(s) or
  // task(s) if the kernel is present.
  app_init();

  //Enable GPIO clock
  CMU_ClockEnable(cmuClock_GPIO, true);

  //GPIO Configuration
  initGPIO();
  //I2C configuration
  if ((MAX86150==1)||(TEMPSENSOR==1)||(IMU==1)){
      I2C_InitConfig();
  }

  //EDA
  if (EDA==1){
        initADC();
  }
  //TEMPERATURA
  //if (TEMPSENSOR==1){  }

  //PPG
  /*if (MAX86150==1){
      configureMAX86150();
  }*/
  //IMU
  /*if (IMU==1){
       configureIMU();
   }*/
  //LETIMER para la EDA y TEMP
  if ((EDA==1)||(TEMPSENSOR==1)||(MAX86150==1)||(IMU==1)){
      initLETIMER();
          /*while (1){
              ICM20648_ReadFIFO();
              printf("leido");
              readTemperature();
          }*/
     }


#if defined(SL_CATALOG_KERNEL_PRESENT)
  // Start the kernel. Task(s) created in app_init() will start running.
  sl_system_kernel_start();
#else // SL_CATALOG_KERNEL_PRESENT
  while (1) {
    // Do not remove this call: Silicon Labs components process action routine
    // must be called from the super loop.
    sl_system_process_action();

    // Application process.
    app_process_action();



#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // Let the CPU go to sleep if the system allows it.
    sl_power_manager_sleep();
#endif
  }
#endif // SL_CATALOG_KERNEL_PRESENT
}
