/*
 * EDA.c
 *
 *  Created on: 17 ene 2025
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
#include "em_usart.h"
#include <string.h>
#include "em_i2c.h"
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



// GPIO
// Definir el puerto y el pin para el LED que se encenderá en el main
#define MAIN_LED_PORT gpioPortA
#define MAIN_LED_PIN 6

// Definir el puerto y el pin para el LED que se encenderá en la interrupción
#define TIMER_LED_PORT gpioPortA
#define TIMER_LED_PIN 5

#define TEMP_ENABLE_PORT gpioPortF
#define TEMP_ENABLE_PIN 9

// Definir pines habilitacion IMU
#define IMU_ENABLE_PORT gpioPortF
#define IMU_ENABLE_PIN 11


// Init to max ADC clock for Series 1
#define adcFreq  16000000
// Desired letimer interrupt frequency (in Hz)
//#define letimerDesired  1000


volatile uint32_t sample;
volatile uint32_t millivolts;
int modo=1;
int count=0;
int primera=1;

#define MAX86150_I2C_ADDRESS 0x5E
#define FIFO_DATA_REGISTER  0x07
uint8_t dummy[1] = {0};

void limpiarBufferI2C() {
    uint8_t dummys[1];
    //printf("limp buffer\n");
        // Realizar una lectura dummy de 1 byte sin usar el resultado
        readI2C(MAX86150_I2C_ADDRESS, FIFO_DATA_REGISTER, dummys, 1);

        // Generar una condición de STOP en el bus para resetear el buffer
        //I2C0->CMD = I2C_CMD_ABORT;  // Cancela cualquier transacción en curso

        //I2C0->CMD = I2C_CMD_STOP;   // Genera un STOP en el bus I2C
        //printf("lect buffer\n");
}



void initLETIMER(void)
{
  LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;

  // Enable clock to the LE modules interface
  CMU_ClockEnable(cmuClock_HFLE, true);

  // Select LFXO for the LETIMER
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);  //32768 Hz Low Frequency Crystal Oscillator (LFXO)
  CMU_ClockEnable(cmuClock_LETIMER0, true);  //32768 Hz

  // Reload COMP0 on underflow, idle output, and run in repeat mode
  letimerInit.comp0Top  = true;
  letimerInit.ufoa0     = letimerUFOANone;
  letimerInit.repMode   = letimerRepeatFree;

  // Need REP0 != 0 to pulse on underflow
  LETIMER_RepeatSet(LETIMER0, 0, 1);

  uint32_t comp0_value =CMU_ClockFreqGet(cmuClock_LETIMER0) * 0.05; // 100ms
  //uint32_t comp0_value = CMU_ClockFreqGet(cmuClock_LETIMER0)* 0.01; // 25ms

  LETIMER_CompareSet(LETIMER0, 0, comp0_value);
    // Compare on wake-up interval count
      //uint32_t topValue= CMU_ClockFreqGet(cmuClock_LETIMER0) / 2; //2 Hz
    //LETIMER_CompareSet(LETIMER0, 0, topValue);

    // Initialize and enable LETIMER
    LETIMER_Init(LETIMER0, &letimerInit);

    // Enable LETIMER0 interrupts for COMP0
    LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP0);

    // Enable LETIMER interrupts
    NVIC_ClearPendingIRQ(LETIMER0_IRQn);
    NVIC_EnableIRQ(LETIMER0_IRQn);

}


/**************************************************************************//**
 * @brief ADC initialization
 *****************************************************************************/
void initADC (void)
{
  // Enable clocks required
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);

  // Declare init structs
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  // Modify init structs
  init.prescale   = ADC_PrescaleCalc(adcFreq, 0);
  init.timebase   = ADC_TimebaseCalc(0);

  initSingle.diff       = false;       // single ended
  initSingle.reference  = adcRef5V;   // internal 5V reference
  initSingle.resolution = adcRes12Bit; // 12-bit resolution
  initSingle.acqTime    = adcAcqTime4; // set acquisition time to meet minimum requirements

  // Select ADC input. See README for corresponding EXP header pin.
  //initSingle.posSel = adcPosSelAPORT2XCH9;
 // initSingle.posSel = adcPosSelAPORT4YCH30;
  initSingle.posSel = adcPosSelAPORT3XCH6;

  // Initialize ADC and Single conversions
  ADC_Init(ADC0, &init);
  ADC_InitSingle(ADC0, &initSingle);

  // Enable ADC Single Conversion Complete interrupt
  ADC_IntEnable(ADC0, ADC_IEN_SINGLE);

  // Enable ADC interrupts
  NVIC_ClearPendingIRQ(ADC0_IRQn);
  NVIC_EnableIRQ(ADC0_IRQn);
}

/**************************************************************************//**
 * @brief LETIMER Handler
 *****************************************************************************/


void LETIMER0_IRQHandler(void)
{
    uint32_t flags = LETIMER_IntGet(LETIMER0);
    LETIMER_IntClear(LETIMER0, flags);
    if (modo==1){
          // GPIO_PinModeSet(TIMER_LED_PORT,  TIMER_LED_PIN, gpioModePushPull, 1);
           GPIO_PinOutSet(TIMER_LED_PORT, TIMER_LED_PIN);
           modo =0;
       } else {
           //GPIO_PinModeSet(TIMER_LED_PORT,  TIMER_LED_PIN, gpioModePushPull, 0);
           GPIO_PinOutClear(TIMER_LED_PORT, TIMER_LED_PIN);
           modo=1;
       }

    if (flags & LETIMER_IF_COMP0) {
        //printf("Interrupción a 10 Hz (cada 0.1 s)\n");

        switch (count){
          case 0:
           // printf("case 0\n");

            if(EDA==1){
              ADC_Start(ADC0, adcStartSingle);
            }

            if(TEMPSENSOR==1){
               GPIO_PinModeSet(TEMP_ENABLE_PORT , TEMP_ENABLE_PIN, gpioModePushPull, 1);
               GPIO_PinOutSet(TEMP_ENABLE_PORT, TEMP_ENABLE_PIN);
               readTemperature();
               resetTemp();
               limpiarBufferI2C();
               GPIO_PinOutClear(TEMP_ENABLE_PORT, TEMP_ENABLE_PIN);
                 if ((MAX86150==1) && ((TEMPSENSOR==1)||(IMU==1))){
                   CMU_ClockEnable(cmuClock_I2C0, false); // Apaga el reloj del I2C0
                   I2C0->CTRL &= ~I2C_CTRL_EN;
                   printf("Clock I2C apagado\n");
                 }
             }

            count++;
            break;
          case 1:
            //printf("case 1\n");
            if((TEMPSENSOR==1)&&((MAX86150==1)||(IMU==1))){
              CMU_ClockEnable(cmuClock_I2C0, true);
              I2C0->CTRL |= I2C_CTRL_EN;
              printf("Clock I2C activado\n");
            }
              count++;
            break;
          case 2:
            //printf("case 2\n");
             if ((MAX86150==1)&&((TEMPSENSOR==1)||(IMU==1))){
            resetMAX86150();
            configureMAX86150();
            } else if ((MAX86150==1)&&(primera==1)){
                resetMAX86150();
                configureMAX86150();
            }
              count++;
            break;
          default:
            count++;
            if (count>=3 && count<900){
                if (MAX86150){
                   leermuestrasMAX86150();
                   } else count=899;
            }

            if ((count==900)&&(IMU==1)&&(MAX86150==0)&&(primera==1)){
                GPIO_PinModeSet(IMU_ENABLE_PORT , IMU_ENABLE_PIN, gpioModePushPull, 0);
                GPIO_PinOutSet(IMU_ENABLE_PORT, IMU_ENABLE_PIN);
                configureIMU();
            }

            if (count>=900){
                if(IMU){
                      ICM20648_ReadFIFO();
                      }

                if( ((count%10)==0)&&(TEMPSENSOR ==1)){//si es multiplo de 5

                        GPIO_PinModeSet(TEMP_ENABLE_PORT , TEMP_ENABLE_PIN, gpioModePushPull, 1);
                        GPIO_PinOutSet(TEMP_ENABLE_PORT, TEMP_ENABLE_PIN);
                        readTemperature();
                        resetTemp();
                        //GPIO_PinOutClear(TEMP_ENABLE_PORT, TEMP_ENABLE_PIN);

                }
                if ((IMU==0)&&(TEMPSENSOR==0)){
                    count=1200;
                }
            }

            if (EDA){
                if((count%5)==0){
                    ADC_Start(ADC0, adcStartSingle);
                }
            }

           if (count==1200){
               count=0;
               if(MAX86150==1){
               GPIO_PinOutClear(IMU_ENABLE_PORT , IMU_ENABLE_PIN); //Apago IMU
               }
               primera=0;
            }

            break;
        }
      }
 }


/**************************************************************************//**
 * @brief ADC Handler
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  // Get ADC result
  sample = ADC_DataSingleGet(ADC0);
  //printf("ADC Sample:  %" PRIu32 "\n", sample);

  // Calcular conductividad
  //ADC
  double voltaje = (5*( (double) sample))/4096.0; //adc 12 bit 5V referencia
  //printf("EDA: ADC voltaje: %.6f V\n", voltaje);
  //Calcular la resistencia
  double respiel=(2.6 - voltaje)*(330000/0.2);
  //printf(" ADC resistencia: %.6f V\n", respiel);
  //Inservsa para la conductividad
  double conductividad =1/respiel;
 // printf("EDA: ADC conductividad: %.11f\n", conductividad);
  printf("EDA:, %.11f\n", conductividad);

}

