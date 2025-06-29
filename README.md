# SW-de-un-wearable-para-medici-n-de-parametros-fisiologicos
DESARROLLO SOFTWARE  DE UN WEARABLE PARA LA MEDIDA DE PARÁMETROS FISIOLÓGICOS PARA SU USO EN ROBÓTICA SOCIAL


Este repositorio contiene el código fuente del proyecto desarrollado en Simplicity Studio para un dispositivo vestible que integra sensores fisiológicos vestibles (EDA, PPG, IMU, temperatura) con un robot  social, con el objetivo de detectar estados emocionales del usuario y adaptar la interacción en consecuencia.

// SENSORES UTILIZADOS EN EL PROYECTO
Los sensores utilizados son los siguientes:
- EDA: sensor anagóligo de desarrollo propio
- PPG: sensor digital MAX86150
- IMU: sensor digital ICM20648
- Temperatura: sensor digital SI7051

//REQUISITOS DEL SISTEMA
- Hardware: EFR32MG12 (o equivalente), sensores conectados vía I2C
- Software: Simplicity Studio 5, SDK Gecko

// FUNCIONALIDADES ACTUALES DEL SISTEMA
- EL sistema permite la monitorización de todos los sensores de forma individual y de forma conjunta
- Un contador realiza interrupción según las cuales se hacen mediciones en un sensor u otro
- La salida de los datos se realiza mediante un puerto serie con el formato "Nombre del sensor:, valorx"
- El sistema permite habilitar o deshabilitar los sensores en "control.h"
- En el proyecto se incluyen archivos para la configuración de BLE para la conexión inalámbrica con el robot social
