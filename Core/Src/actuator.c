/*
 * actuator.c
 *
 *  Created on: Sep 29, 2025
 *      Author: benj
 */

#include "actuator.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
// ------------------------------
// Global Değişkenler
// ------------------------------
uint8_t motorState = MOTOR_IDLE;
uint32_t motorStartTime = 0;
uint16_t lastProcessedDirection = 0;
int direction_value = 0;
uint8_t flag = 0;

// ------------------------------
// Eşik Değer Tanımlamaları
// ------------------------------
#define DIRECTION_LEFT_MIN      500
#define DIRECTION_LEFT_MAX      1000

#define DIRECTION_RIGHT_MIN     64536
#define DIRECTION_RIGHT_MAX     65036

#define DIRECTION_CENTER_MIN   -200
#define DIRECTION_CENTER_MAX    200

// ------------------------------
// Motor Kontrol Fonksiyonu
// ------------------------------
void MotorAktuatorKontrol(int new_direction_value)
{
    // Eğer motor zaten çalışıyorsa, yeni komut alma
    if (motorState != MOTOR_IDLE) {
        return;
    }

    direction_value = new_direction_value;

    // Sola dönüş (sol motor aktif)
    if (direction_value >= DIRECTION_LEFT_MIN && direction_value <= DIRECTION_LEFT_MAX) {
        HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_SET);    // Sol motor ON
        HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_RESET);  // Sağ motor OFF

        motorState = MOTOR_RUNNING_LEFT;
        motorStartTime = HAL_GetTick();
        lastProcessedDirection = direction_value;
    }
    // Sağa dönüş (sağ motor aktif)
    else if (direction_value >= DIRECTION_RIGHT_MIN && direction_value <= DIRECTION_RIGHT_MAX) {
        HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_SET);    // Sağ motor ON
        HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_RESET);  // Sol motor OFF

        motorState = MOTOR_RUNNING_RIGHT;
        motorStartTime = HAL_GetTick();
        lastProcessedDirection = direction_value;
    }
    // Orta pozisyon (motorları durdur)
    else if (direction_value >= DIRECTION_CENTER_MIN && direction_value <= DIRECTION_CENTER_MAX) {
        HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_RESET);
        motorState = MOTOR_IDLE;
        flag = 0; // Orta pozisyon flag'i
    }
}

// ------------------------------
// Motor Durum Güncelleme Fonksiyonu
// ------------------------------
void MotorAktuatorUpdate(void)
{
    uint32_t currentTime = HAL_GetTick();

    if (motorState != MOTOR_IDLE) {
        if ((currentTime - motorStartTime) >= MOTOR_RUN_TIME) {
            // Motorları durdur
            HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_RESET);

            // Flag ve durum güncelle
            if (motorState == MOTOR_RUNNING_LEFT) {
                flag = 1;
            } else if (motorState == MOTOR_RUNNING_RIGHT) {
                flag = 2;
            }

            motorState = MOTOR_IDLE;
            lastProcessedDirection = 0;
            direction_value = 0;
        }
    }
}

// ------------------------------
// Acil Durdurma Fonksiyonu
// ------------------------------
void MotorAktuatorStop(void)
{
    HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_RESET);

    motorState = MOTOR_IDLE;
    flag = 0;
    direction_value = 0;
    lastProcessedDirection = 0;
}

// ------------------------------
// Durum ve Flag Fonksiyonları
// ------------------------------
uint8_t IsMotorRunning(void)
{
    return (motorState != MOTOR_IDLE) ? 1 : 0;
}

uint8_t GetMotorFlag(void)
{
    return flag;
}

void ClearMotorFlag(void)
{
    flag = 0;
}
