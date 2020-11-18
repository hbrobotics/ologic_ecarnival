/*
 * edge_sensor.h
 *
 * Handle reading the edge sensors
 *
 *  Created on: Sep 26, 2020
 *      Author: Ralph Gnauck
 */

#ifndef INC_EDGE_SENSOR_H_
#define INC_EDGE_SENSOR_H_

#include <stdint.h>

// flags to indicate each sensor
#define BUMP_BIT_LEFT 1
#define BUMP_BIT_RIGHT 2


// enum to represent sensor state
typedef enum EDGE_SENSOR_STATE_t {
	ES_CLEAR, // not over edge
	ES_HIT    // sensor over drop
} EDGE_SENSOR_STATE;


void enableEdgeSensors(uint32_t sensor); // enable  a sensor
void disableEdgeSensors(uint32_t sensor); // disable a sensor

void updateEdgeSensors(void); // update sensors states (debounces switch sensor inputs)
EDGE_SENSOR_STATE getEdgeSensorState(uint32_t sensor); // get state of a sensor

#endif /* INC_EDGE_SENSOR_H_ */
