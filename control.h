/*
 * control.h
 *
 *  Created on: 21 mar 2025
 *      Author: usuario
 */

#ifndef CONTROL_H_
#define CONTROL_H_

  //Habilitacion sensores

    //HABILITAR EDA
    #define EDA 0
    //HABILITAR TEMPERATURA
    #define TEMPSENSOR 1
    //HABILITAR PPG
    #define MAX86150 0
    //HABILITAR IMU
    #define IMU 0

    //EDA

    //MAX86150
    #define PPGLED1IR 1  //Para activar o desactivar los sensores
    #define PPGLED2RED 1
    #define ECGSENSOR 0
    #define PROXY 0
    //SI7051

    //IMC 20648
    //Habilitar sensores
    #define ACELEROMETRO  0
    #define GIROSCOPO 1
    #define TEMPERATURA 1
    #define DMP 0
    #define FIFOIMU 1 //1 normalmente
    #define ACCEL_FIFO_EN   0b0 //modificar X si se quiere habilitar la fifo 0bX
    #define GYRO_X_FIFO_EN  0b1 // 1 normalmente
    #define GYRO_Y_FIFO_EN  0b1
    #define GYRO_Z_FIFO_EN  0b1
    #define TEMP_FIFO_EN   0b1



#endif /* CONTROL_H_ */
