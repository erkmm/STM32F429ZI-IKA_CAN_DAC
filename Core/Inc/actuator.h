/*
 * actuator.h
 *
 *  Created on: Sep 29, 2025
 *      Author: benj
 */

#ifndef INC_ACTUATOR_H_
#define INC_ACTUATOR_H_

#include "main.h"  // HAL kütüphanesi ve GPIO tanımlamaları için

// Motor durumları
#define MOTOR_IDLE          0
#define MOTOR_RUNNING_LEFT  1
#define MOTOR_RUNNING_RIGHT 2

// Motor çalışma süresi (ms cinsinden) — ihtiyaç varsa dışarıdan tanımlanabilir
#ifndef MOTOR_RUN_TIME
#define MOTOR_RUN_TIME 1000
#endif

// Global değişkenlerin dış bildirimleri
extern uint8_t motorState;
extern uint32_t motorStartTime;
extern uint16_t lastProcessedDirection;
extern int direction_value;
extern uint8_t flag;

// Fonksiyon prototipleri
void MotorAktuatorKontrol(int direction_value);
void MotorAktuatorUpdate(void);
void MotorAktuatorStop(void);

uint8_t IsMotorRunning(void);
uint8_t GetMotorFlag(void);
void ClearMotorFlag(void);

#endif /* INC_ACTUATOR_H_ */

