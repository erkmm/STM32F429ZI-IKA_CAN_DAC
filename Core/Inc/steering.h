/*
 * steering.h
 *
 *  Created on: Sep 8, 2025
 *      Author: benj
 */

#ifndef INC_STEERING_H_
#define INC_STEERING_H_

#include "stdint.h"

void steering_update(int16_t target_position);
void steering_calibrate(void);

#endif /* INC_STEERING_H_ */
