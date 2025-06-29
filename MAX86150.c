/*
 * MAX86150.C
 *
 *  Created on: 4 feb 2025
 *      Author: usuario
 */


#include "EDA.h"
#include "SI7051.h"
#include "MAX86150.h"
#include "IMU.h"
#include "IMU_functions.h"
#include "I2C_functions.h"
#include <stdio.h>
#include <inttypes.h>
#include "control.h"
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




#define MAX86150_I2C_ADDRESS 0x5E // Dirección 7 bits del esclavo (0b1011110 + R/W = 0x5D para escritura)


  #define FIFO_BUFFER_SIZE  32
  #define ID 0x5E       //ID del escalvo


  #define INTERRUPT_STATUS_1  0x00
  #define INTERRUPT_STATUS_2  0x01
  #define INTERRUPT_ENABLE_1  0x02
  #define INTERRUPT_ENABLE_2  0x03
  //FIFO REGISTERS
  #define FIFO_WRITE_POINTER  0x04
  #define OVERFLOW_COUNTER  0x05
  #define FIFO_READ_POINTER  0x06
  #define FIFO_DATA_REGISTER  0x07
  #define FIFO_CONFIGURATION  0x08
  //FIFO DATA CONTROL
  #define FIFO_DATA_CONTROL_REGISTER_1  0x09
  #define FIFO_DATA_CONTROL_REGISTER_2  0x0A
  //SYSTEM CONTROL
  #define SYSTEM_CONTROL  0x0D
  //PPG CONFIGURATION
  #define PPG_CONFIGURATION_1  0x0E
  #define PPG_CONFIGURATION_2  0x0F
  #define PROX_INTERRUPT_THRESHOLD  0x10  //IMPLEMENTAR PROXIMIDAD
  //LED PULSE AMPLITUDE
  #define LED1_PA  0x11
  #define LED2_PA  0x12
  #define LED_RANGE  0x14
  #define LED_PILOT_PA  0x15
  //ECG CONFIGURATION
  #define ECG_CONFIGURATION_1  0x3C
  #define ECG_CONFIGURATION_3  0x3E
  #define PART_ID  0xFF

  //A continuación definimos las distintas configuraciones de interrupciones
  //Bit (posición) en el cual se encuentra la interrupción
  #define A_FULL  0b10000000
  #define PPG_RDY  0b0100000
  #define ALC_OVF  0b00100000
  #define PROX_INT  0b00010000
  #define PWR_RDY  0b00000001

  #define VDD_OOR  0b10000000
  #define ECG_RDY  0b00000100
  //Valor para activar o desactivar  las interrupciones
  #define NONE 0b00000000
  #define A_FULL_EN  0b10000000
  #define PPG_RDY_EN  0b0100000
  #define ALC_OVF_EN  0b00100000
  #define PROX_INT_EN  0b00010000
  #define PWR_RDY_EN  0b00000001

  //FIFO Configuration (0x08)
  #define A_FULL_CLR_RD_DATA_NOCLR 0
  #define A_FULL_CLR_RD_DATA_CLR 1
  #define AFULL_ONCE 1
  #define FIFO_ROLLS_ON_FULL_OFF 0 //NEW SAMPLES ARE LOST
  #define FIFO_ROLLS_ON_FULL_ON 1 // OLD SAMPLES ARE LOST
  #define FIFO_A_FULL_FREE_SPACE_NULL 0b0000 //EN MEDIO PODEMOS AJUSTAR MANUALMENTE
  #define FIFO_A_FULL_FREE_SPACE_EMPTY 0b1111

  //DATA CONTROL REGISTER (SENSORS)
  #define SENSOR_NONE  0b0000
  #define LED1_IR  0b0001
  #define LED2_RED  0b0010
  #define PILOT_LED1_IR  0b0101
  #define PILOT_LED2_RED  0b0110
  #define ECG  0b1001

  //SYSTEM CONTROL
  #define FIFO_EN_OFF 0
  #define FIFO_EN_ON 1
  #define SHDN_OFF 0
  #define SHDN_ON 1
  #define RESET_OFF 0
  #define RESET_ON 1

  //PPG CONFIGURATION PPG_ADC_RGE CONTROL
  #define NANO_AMPERE_4096  0b00
  #define NANO_AMPERE_8192  0b01
  #define NANO_AMPERE_16384  0b10
  #define NANO_AMPERE_32768  0b11

  //PPG SAMPLES PER SECOND
  #define SAMPLES_10_PPS_1  0x0
  #define SAMPLES_20_PPS_1  0x1
  #define SAMPLES_50_PPS_1  0x2
  #define SAMPLES_84_PPS_1  0x3
  #define SAMPLES_100_PPS_1  0x4
  #define SAMPLES_200_PPS_1  0x5
  #define SAMPLES_400_PPS_1  0x6
  #define SAMPLES_800_PPS_1  0x7
  #define SAMPLES_1000_PPS_1  0x8
  #define SAMPLES_1600_PPS_1  0x9
  #define SAMPLES_3200_PPS_1  0xA
  #define SAMPLES_10_PPS_2  0xB
  #define SAMPLES_20_PPS_2  0xC
  #define SAMPLES_50_PPS_2  0xD
  #define SAMPLES_84_PPS_2  0xE
  #define SAMPLES_100_PPS_2  0xF

  //PPG LED PULSE WIDHT CONTROL
  #define MICRO_50 = 0b00
  #define MICRO_100  0b01
  #define MICRO_200  0b10
  #define MICRO_400  0b11

  //PPG SAMPLE AVERAGING OPTIONS
  #define SAMPLE_AVERAGE_1  0b000
  #define SAMPLE_AVERAGE_2  0b001
  #define SAMPLE_AVERAGE_4  0b010
  #define SAMPLE_AVERAGE_8  0b011
  #define SAMPLE_AVERAGE_16  0b100
  #define SAMPLE_AVERAGE_32  0b101

  //LED CURRENT
  #define MILLI_AMP_50  0b00
  #define MILLI_AMP_100  0b01

  //PROX INTERRUPT THRESHOLD (modificarlo si es necesario)
  #define PROX_INT_TRESH 0X04

  //LED1 Y LED2 PA CURRENT (modificarlo si es necesario)
  #define LED_RGE_NULL 0X00
  #define LED_RGE_MAX 0XFF
  #define LED_RGE_TYPICAL 0XF5

  //LED CURRENT DURING PROX MODE
  #define LED_PROX_NULL 0X00
  #define LED_PROX_MAX 0XFF

  //ECG Configuration
  #define ECG_ADC_CLK_ON 1
  #define ECG_ADC_CLK_OFF 0
  #define ECG_ADC_OSR 0b11

  #define PGA_ECG_GAIN 0b10
  #define IA_GAIN 0b00



void settingPPG(){
  uint8_t dato =0x00;
  int PPG= PPGLED1IR + PPGLED2RED;
  switch (PPG) {
      case 0:
        break;
      case 1:
        if (PPGLED1IR==1){
          dato = LED1_IR << 4;
        }
        else {
                dato = LED2_RED << 4;
            }

        if (writeI2C(MAX86150_I2C_ADDRESS, FIFO_DATA_CONTROL_REGISTER_1, &dato, 1) != i2cTransferDone) {
                 printf("Error en la secuencia 1\n");
        }

        break;
      case 2:
        dato = (LED2_RED << 4);  // Desplazar cadena1 hacia la izquierda 4 posiciones
        dato |= LED1_IR;
        if (writeI2C(MAX86150_I2C_ADDRESS, FIFO_DATA_CONTROL_REGISTER_1, &dato, 1) != i2cTransferDone) {
                 printf("Error en la secuencia 1\n");
        }
        break;
      default:
        printf("Opción invalida\n");
        break;
      }

}

void settingECG() {
    uint8_t dato;

    if((PPGLED1IR + PPGLED2RED) == 0){
        if (ECGSENSOR == 1) {
            dato = ECG << 4;

            // Escribir el valor en FIFO_DATA_CONTROL_REGISTER_2
            if (writeI2C(MAX86150_I2C_ADDRESS, FIFO_DATA_CONTROL_REGISTER_1, &dato, 1) != i2cTransferDone) {
                printf("Error al configurar ECG en FIFO_DATA_CONTROL_REGISTER_2\n");
                }
            }
    }else
      if (ECGSENSOR == 1) {
                  dato = ECG << 4;

                  // Escribir el valor en FIFO_DATA_CONTROL_REGISTER_2
                  if (writeI2C(MAX86150_I2C_ADDRESS, FIFO_DATA_CONTROL_REGISTER_2, &dato, 1) != i2cTransferDone) {
                      printf("Error al configurar ECG en FIFO_DATA_CONTROL_REGISTER_2\n");
                      }
      }

}


void resetMAX86150(){
  uint8_t resetMAX=0x01;
  writeI2C(MAX86150_I2C_ADDRESS, SYSTEM_CONTROL, &resetMAX, 1);

}

void LEDconfiguration () {
  uint8_t adcrge = NANO_AMPERE_32768;
  uint8_t samples = SAMPLES_100_PPS_1;
  uint8_t ledpw = MICRO_400;
  uint8_t average = SAMPLE_AVERAGE_1;
  uint8_t dato = 0;  // Inicializar resultado con ceros
  dato |= (adcrge << 6);  // Desplazar cadena1 hacia la izquierda 6 posiciones
  dato |= (samples << 2);  // Desplazar cadena2 hacia la izquierda 2 posiciones
  dato |= ledpw;         // Realizar la operación OR bit a bit con cadena3

  // Mostrar `dato` en decimal y binario
    printf("dato (decimal): %d\n", dato);
    printf("dato (binario): ");
    for (int i = 7; i >= 0; i--) {
        printf("%d", (dato >> i) & 1);
    }
    printf("\n");
  //escritura(PPG_CONFIGURATION_1, dato);
  if (writeI2C(MAX86150_I2C_ADDRESS, PPG_CONFIGURATION_1, &dato, 1) != i2cTransferDone) {
                   printf("Error en la secuencia led configuration\n");
          }
  uint8_t dato2 = 0;
  dato2 = average << 5;
  //escritura(PPG_CONFIGURATION_2,dato2);
  if (writeI2C(MAX86150_I2C_ADDRESS, PPG_CONFIGURATION_2, &dato2, 1) != i2cTransferDone) {
                     printf("Error en la secuencia led configuration 2\n");
            }
}

//Configuración de la corriente. Si se quiere configurar de otra forma realizarlo manualmente desde el define typical. Estos son los valores típicos
 void LEDcurrentconfiguration () {
   uint8_t LED1_PA_Current_Amplitude = LED_RGE_TYPICAL; //ponemos es que teníamos por defecto pero podemos poner otro
   uint8_t LED2_PA_Current_Amplitude = LED_RGE_TYPICAL;
   uint8_t LED_Current1 = MILLI_AMP_100;
   uint8_t LED_Current2 = MILLI_AMP_100;
   //escritura( LED1_PA, LED1_PA_Current_Amplitude);
   if (writeI2C(MAX86150_I2C_ADDRESS, LED1_PA, &LED1_PA_Current_Amplitude, 1) != i2cTransferDone) {
                        printf("Error en la secuencia led current\n");
               }
   //escritura( LED2_PA, LED2_PA_Current_Amplitude);
   if (writeI2C(MAX86150_I2C_ADDRESS, LED2_PA, &LED2_PA_Current_Amplitude, 1) != i2cTransferDone) {
                        printf("Error en la secuencia led current 2\n");
               }
   uint8_t dato = 0;
   dato  = (LED_Current1 << 2) | LED_Current2; // SON 4 BITS
   dato = (dato >> 4) | 0x00; // 8 BITS
   //escritura( LED_RANGE, dato);
   if (writeI2C(MAX86150_I2C_ADDRESS, LED_RANGE, &dato, 1) != i2cTransferDone) {
                        printf("Error en la secuencia led current 3\n");
               }
 }

 void ECGconfiguration() {

   uint8_t ecgclk = ECG_ADC_CLK_OFF;
   uint8_t ecgosr = ECG_ADC_OSR;  //200 Hz
   uint8_t ecggain = PGA_ECG_GAIN;
   uint8_t iagain  =  IA_GAIN;

     if (ECGSENSOR == 1) {
         // Configuración del primer registro ECG_CONFIGURATION_1
         ecgclk = (ecgclk << 2) | ecgosr;  // Desplazar ecgclk y combinar con ecgosr
         ecgclk = (ecgclk >> 5) | 0x00;     // Ajuste adicional

         if (writeI2C(MAX86150_I2C_ADDRESS, ECG_CONFIGURATION_1, &ecgclk, 1) != i2cTransferDone) {
             printf("Error al escribir ECG_CONFIGURATION_1\n");
             return;
         }

         // Configuración del segundo registro ECG_CONFIGURATION_3
         uint8_t dato = (ecggain << 2) | iagain; // Combinar ecggain e iagain (4 bits)
         dato = (dato >> 4) | 0x00; // Ajuste a 8 bits

         if (writeI2C(MAX86150_I2C_ADDRESS, ECG_CONFIGURATION_3, &dato, 1) != i2cTransferDone) {
             printf("Error al escribir ECG_CONFIGURATION_3\n");
             return;
         }
     }
 }

 void SettingInterrupcion(uint8_t direccion, uint8_t posicion, uint8_t valor) {
     uint8_t datoAntiguo = 0;

     // Leer el valor actual del registro en la dirección especificada
     if (readI2C(MAX86150_I2C_ADDRESS, direccion, &datoAntiguo, 1) != i2cTransferDone) {
         printf("Error al leer el registro de interrupciones\n");
         return;
     }

     // Modificar solo la posición deseada sin afectar los otros bits
     if (valor == 0) {
         // Deshabilitar la interrupción en la posición especificada
         valor = (~posicion) & datoAntiguo;
     } else {
         // Habilitar la interrupción sin desconfigurar las anteriores
         valor |= datoAntiguo;
     }

     // Escribir el nuevo valor en la misma dirección
     if (writeI2C(MAX86150_I2C_ADDRESS, direccion, &valor, 1) != i2cTransferDone) {
         printf("Error al escribir en el registro de interrupciones\n");
     }
 }

void proxmode(){
  SettingInterrupcion (INTERRUPT_ENABLE_1, PROX_INT, PROX_INT_EN ); //configuramos la interrupción
  //Habilitar la interrupción
  uint8_t limiteprox = PROX_INT_TRESH;
  //escritura (PROX_INTERRUPT_THRESHOLD, PROX_INT_TRESH );  //Límite proximity
  if (writeI2C(MAX86150_I2C_ADDRESS, PROX_INTERRUPT_THRESHOLD, &limiteprox, 1) != i2cTransferDone) {
                           printf("Error en la secuencia limite proxmode\n");
                  }
  //LED power during the PROX mode
  uint8_t LED_Current_prox_mode = LED_RGE_TYPICAL;
  if (writeI2C(MAX86150_I2C_ADDRESS, LED_PILOT_PA, &LED_Current_prox_mode, 1) != i2cTransferDone) {
                         printf("Error en la secuencia proxmode current\n");
                }

}



void signalprocessing(uint8_t buffer[], uint8_t PPG_num, uint8_t NUM_SAMPLES){
  uint8_t byte1,byte2,byte3,byte4,byte5,byte6,byte7,byte8,byte9;
  uint32_t sample;
  uint32_t sample2;
  uint32_t ecgsample;
  if ((PPG_num==1)&&(ECGSENSOR==0)){
      for(uint32_t i=0 ; i< NUM_SAMPLES ; i++){
           byte1 = buffer[i*3];
           byte1 &= 0x07; //pongo a 0 los 5 primeros bits
           byte2 = buffer[i*3 + 1];
           byte3 = buffer[i*3 + 2];
           sample = (((uint32_t)byte1)<<16)|((uint32_t)byte2<<8)|((uint32_t)byte3);
           printf("MAX86150:,%" PRIu32 "\n", sample);
      }


  }else if ((PPG_num==2)&&(ECGSENSOR==0)){
      for(uint32_t i=0 ; i< NUM_SAMPLES ; i++){
                 byte1 = buffer[i*6];
                 byte1 &= 0x07; //pongo a 0 los 5 primeros bits
                 byte2 = buffer[i*6 + 1];
                 byte3 = buffer[i*6 + 2];
                 byte4 = buffer[i*6 + 3];
                 byte4 &= 0x07;
                 byte5 = buffer[i*6 + 4];
                 byte6 = buffer[i*6 + 5];
                 sample = (((uint32_t)byte1)<<16)|((uint32_t)byte2<<8)|((uint32_t)byte3);
                 sample2 = (((uint32_t)byte4)<<16)|((uint32_t)byte5<<8)|((uint32_t)byte6);
                 printf("MAX86150:,%"PRIu32",%"PRIu32"\n", sample, sample2);
                 //printf("%"PRIu32",%"PRIu32"\n", sample, sample2);


            }
  }else if ((PPG_num==1)&&(ECGSENSOR==1)){
      for(uint32_t i=0 ; i< NUM_SAMPLES ; i++){
                 byte1 = buffer[i*6];
                 byte1 &= 0x07; //pongo a 0 los 5 primeros bits
                 byte2 = buffer[i*6 + 1];
                 byte3 = buffer[i*6 + 2];
                 byte7 = buffer[i*6 + 3];
                 byte8 = buffer[i*6 + 4];
                 byte9 = buffer[i*6 + 5];
                 sample = (((uint32_t)byte1)<<16)|((uint32_t)byte2<<8)|((uint32_t)byte3);
                 ecgsample = (((uint32_t)byte7)<<16)|((uint32_t)byte8<<8)|((uint32_t)byte9);
                 ecgsample &= 0x3FFFF; //mantener los 8 bits mas significativos
                 if (ecgsample & 0x20000){
                     ecgsample |= 0xFFFC0000;
                 }
                 int32_t ecg = (int32_t)ecgsample;
                 printf("MAX86150:, %" PRIu32 ", %"PRId32 "\n", sample, ecg);

            }
  }else if ((PPG_num==2)&&(ECGSENSOR==1)){
            for(uint32_t i=0 ; i< NUM_SAMPLES ; i++){
                       byte1 = buffer[i*9];
                       byte1 &= 0x07; //pongo a 0 los 5 primeros bits
                       byte2 = buffer[i*9 + 1];
                       byte3 = buffer[i*9 + 2];
                       byte4 = buffer[i*9 + 3];
                       byte4 &= 0x07;
                       byte5 = buffer[i*9 + 4];
                       byte6 = buffer[i*9 + 5];
                       byte7 = buffer[i*9 + 6];
                       byte8 = buffer[i*9 + 7];
                       byte9 = buffer[i*9 + 8];
                       sample = (((uint32_t)byte1)<<16)|((uint32_t)byte2<<8)|((uint32_t)byte3);
                       sample2 = (((uint32_t)byte4)<<16)|((uint32_t)byte5<<8)|((uint32_t)byte6);
                       ecgsample = (((uint32_t)byte7)<<16)|((uint32_t)byte8<<8)|((uint32_t)byte9);
                        ecgsample &= 0x3FFFF; //mantener los 8 bits mas significativos
                        if (ecgsample & 0x20000){
                            ecgsample |= 0xFFFC0000;
                        }
                        int32_t ecg = (int32_t)ecgsample;
                       printf("MAX86150:, %" PRIu32 ", %" PRIu32 ", %" PRId32 "\n", sample, sample2,ecg);
                  }

  }else if ((PPG_num==0)&&(ECGSENSOR==1)){
      for(uint32_t i=0 ; i< NUM_SAMPLES ; i++){
                 byte7 = buffer[i*3 + 0];
                 byte8 = buffer[i*3 + 1];
                 byte9 = buffer[i*3 + 2];
                 ecgsample = (((uint32_t)byte7)<<16)|((uint32_t)byte8<<8)|((uint32_t)byte9);
                  ecgsample &= 0x3FFFF; //mantener los 8 bits mas significativos
                  if (ecgsample & 0x20000){
                      ecgsample |= 0xFFFC0000;
                  }
                  int32_t ecg = (int32_t)ecgsample;
                 printf("MAX86150:, %" PRId32 "\n", ecg);
            }
  }
}

void leermuestrasMAX86150() {
     uint8_t punteros[3] = {0};

     // Leer WR_PTR (4), OVT_COUNT(5), RD_PTR(6)
     if (readI2C(MAX86150_I2C_ADDRESS, FIFO_WRITE_POINTER, punteros, 3) != i2cTransferDone) {
         printf("Error al leer los punteros FIFO\n");
         return;
     }

     uint8_t WR_PTR = punteros[0];
     uint8_t OVT_COUNT = punteros[1];
     uint8_t RD_PTR = punteros[2];

     // Imprimir los valores leídos
     // printf("WR_PTR: %d, OVT_COUNT: %d, RD_PTR: %d\n", WR_PTR, OVT_COUNT, RD_PTR);


     // Calcular número de muestras disponibles
     uint8_t NUM_SAMPLES_TO_READ = (OVT_COUNT != 0) ? 32 :
                                   (WR_PTR < RD_PTR) ? (32 - RD_PTR + WR_PTR) :
                                   (WR_PTR - RD_PTR);

    //printf("Muestras disponibles: %d\n", NUM_SAMPLES_TO_READ);

     // Determinar número de bytes a leer por muestra
     uint8_t PPG_num = PPGLED1IR + PPGLED2RED;
     uint8_t bytesPorMuestra = (PPG_num == 2 && ECGSENSOR == 0) ? 6 :
                               (PPG_num == 2 && ECGSENSOR == 1) ? 9 :
                               (PPG_num == 1 && ECGSENSOR == 0) ? 3 :
                               (PPG_num == 1 && ECGSENSOR == 1) ? 6 :
                               (PPG_num == 0 && ECGSENSOR == 1) ? 3 : 0;

     //Lectura de las muestras

     if (bytesPorMuestra > 0) {
         uint8_t buffer[bytesPorMuestra * NUM_SAMPLES_TO_READ];
         if (NUM_SAMPLES_TO_READ){
                    if (readI2C(MAX86150_I2C_ADDRESS, FIFO_DATA_REGISTER, buffer, bytesPorMuestra * NUM_SAMPLES_TO_READ) != i2cTransferDone) {
                              // printf("Error al leer los datos del FIFO\n");
                               return;
                           }
                }

               // Mostrar los datos leídos
               if (NUM_SAMPLES_TO_READ){
                   /*for (int i = 0; i < NUM_SAMPLES_TO_READ; i++) {
                       for (int a =0; a < bytesPorMuestra; a++){
                               printf("%d,", buffer[i*bytesPorMuestra+a]);
                               //printf("pos: %d,", (i*bytesPorMuestra+a));//Imprimir posicion en el buffer
                           }
                           printf("\n");
                }*/
               signalprocessing(buffer,PPG_num,NUM_SAMPLES_TO_READ);
               }

     } else {
         printf("Error: configuración de sensores no válida\n");
     }

 }


 void configureMAX86150() {
   //Definir comportamiento FIFO
   uint8_t fifo_enable = 0x04;
   uint8_t fifo_conf = 0x14;
   writeI2C(MAX86150_I2C_ADDRESS, SYSTEM_CONTROL, &fifo_enable, 1);
   writeI2C(MAX86150_I2C_ADDRESS, FIFO_CONFIGURATION, &fifo_conf, 1);


   if((PPGLED1IR + PPGLED2RED)>0){
       settingPPG();
       LEDconfiguration ();
       LEDcurrentconfiguration ();

       if (PROXY){
           proxmode();
       }
   }

   if (ECGSENSOR){
       settingECG();
       ECGconfiguration();
   }

  /* while (1){
       leermuestrasMAX86150();
       for (int a=0; a<150000;a++){}

   }*/
 }
