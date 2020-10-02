/*
 * edge_sensor.h
 *
 *  Created on: Sep 26, 2020
 *      Author: ralph
 */

#ifndef INC_EDGE_SENSOR_H_
#define INC_EDGE_SENSOR_H_

#include <stdint.h>

#define BUMP_BIT_LEFT 1
#define BUMP_BIT_RIGHT 2

typedef enum EDGE_SENSOR_STATE_t {
	ES_CLEAR,
	ES_HIT
} EDGE_SENSOR_STATE;


void enableEdgeSensors(uint32_t sensor);
void disableEdgeSensors(uint32_t sensor);
void updateEdgeSensors(void);
EDGE_SENSOR_STATE getEdgeSensorState(uint32_t sensor);

#endif /* INC_EDGE_SENSOR_H_ */
