/*
 * steering.c
 *
 *  Created on: Sep 8, 2025
 *      Author: benj
 */


#include "main.h"
#include "steering.h"
#include <stdio.h>
#include <stdlib.h>

#define MAX_ACTUATOR_TIME       20000  // ms
#define MIN_PULSE_TIME          500    // ms
#define POSITION_UPDATE_INTERVAL 100   // ms

typedef enum {
    STEERING_IDLE,
    STEERING_MOVING
} SteeringStatus;

typedef struct {
    int16_t current_position;
    int16_t target_position;
    SteeringStatus status;
    uint32_t move_start_time;
    uint32_t move_duration;
    uint8_t move_direction;
    uint32_t last_update_time;
} SteeringState;

SteeringState steering_state = {0};

// Mock GPIO control (for test)
void actuator_control(uint8_t direction, uint8_t enable) {
    if (enable) {
        if (direction) {
            HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_SET);
        }
    } else {
        HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_RESET);
    }
}



uint32_t get_mock_time_ms(void) {
    return HAL_GetTick();  // Real or simulated
}

uint32_t calculate_move_duration(int16_t diff) {
    uint32_t duration = (abs(diff) * MAX_ACTUATOR_TIME) / 1000;
    if (duration < MIN_PULSE_TIME) duration = MIN_PULSE_TIME;
    return duration;
}

void steering_update(int16_t target_input) {
    uint32_t now = get_mock_time_ms();
    steering_state.target_position = target_input;

    switch (steering_state.status) {
        case STEERING_IDLE: {
            if (now - steering_state.last_update_time < POSITION_UPDATE_INTERVAL)
                return;
            steering_state.last_update_time = now;

            int16_t diff = steering_state.target_position - steering_state.current_position;

            if (abs(diff) < 20)
                return;

            uint8_t direction = (diff > 0) ? 1 : 0;
            uint32_t duration = calculate_move_duration(diff);

            actuator_control(direction, 1);
            steering_state.move_start_time = now;
            steering_state.move_duration = duration;
            steering_state.move_direction = direction;
            steering_state.status = STEERING_MOVING;
            break;
        }

        case STEERING_MOVING: {
            if (now - steering_state.move_start_time >= steering_state.move_duration) {
                actuator_control(0, 0);

                int16_t delta = (steering_state.move_duration * 1000) / MAX_ACTUATOR_TIME;
                if (!steering_state.move_direction) delta = -delta;
                steering_state.current_position += delta;

                //if(steering_state.current_position =! Actuator_Feedback[0])

                if (steering_state.current_position > 1000) steering_state.current_position = 1000;
                if (steering_state.current_position < -1000) steering_state.current_position = -1000;

                steering_state.status = STEERING_IDLE;

                printf("POS -> Current: %d | Target: %d\n",
                       steering_state.current_position,
                       steering_state.target_position);
            }
            break;
        }
    }
}

void steering_calibrate(void) {
    actuator_control(0, 1);
    HAL_Delay(MAX_ACTUATOR_TIME + 100);
    actuator_control(0, 0);
    steering_state.current_position = -1000;

    actuator_control(1, 1);
    HAL_Delay((MAX_ACTUATOR_TIME / 2) + 100);
    actuator_control(1, 0);
    steering_state.current_position = 0;

    printf("Calibration complete. Position: %d\n", steering_state.current_position);
}
