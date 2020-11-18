/*
 * edge_sensor.c
 *
 * Handle reading the edge sensors
 *
 *  Created on: Sep 26, 2020
 *      Author: Ralph Gnauck
 */
#include <stdio.h>

#include "main.h"

#include "gpio.h"
#include "edge_sensor.h"

#define EDGE_SENSOR_ACTIVE GPIO_PIN_SET // define if sensor is active HI or ACTIVE low logic on teh GPIO Pin


// bitmaps for sensor states (one bit per sensor)
static uint32_t sensor_state=0;   // debounced state of each sensor (hit(1) or clear(0))
static uint32_t sensor_changed=0; // each bit is 1 if sensor changes since last update, else 0
static uint32_t sensor_enabled=0; // bit per sensor 1=sensor enabled, 0= sensor disabled (disabled will be ignored and won't stop motors when sensor is hit)

static uint32_t debounce(uint32_t sample); // update the debounce filter using the new raw gpio values

static uint32_t readSensors(void); // read gpio input to get raw sensor state

// enable the sensors
// sensor bits correspond to sensors (1) = enable, 0= don't change
void enableEdgeSensors(uint32_t sensor) {
	sensor_enabled |= sensor;
}

// disable the sensors
// sensor bits correspond to sensors (1) = disable, 0= don't change
void disableEdgeSensors(uint32_t sensor) {
	sensor_enabled &= ~sensor;
}

// return the debounced state of selected sensor
// returns bits to indicate which sensors are HIT (bit=1) - only sensors where corresponding bit in input parameter sensor are also set will be returned
// only enabled sensors will return status - disabled sensors always return 0 in corresponding bit
EDGE_SENSOR_STATE getEdgeSensorState(uint32_t sensor) {

	uint32_t hit =  (sensor_state & sensor)?ES_HIT:ES_CLEAR; // get state of selected sensors

	if(sensor_changed & sensor) { // keep track if sensor has changed
		//printf("Edge Sensor %ld: %s\n",sensor,((hit==ES_HIT)?"Hit":"Clear"));
		sensor_changed &= ~sensor;
	}

	return hit & sensor_enabled; // mask out any disabled sensor status bits;
}

// update the debounce status of the sensor flags
void updateEdgeSensors(void) {

	uint32_t new_state = readSensors(); // get raw gpio state
	uint32_t state = debounce(new_state); // run through debounce filter

	sensor_changed = state ^ sensor_state; // detect which sensors have changed
	sensor_state = state ; // update debounced state variable
}

// read raw gpio status for each sensor
// retuns bitmap of sensor states
uint32_t readSensors(void) {

	// read each sensor IO pin, and addjust for GPIO Active Level
	uint32_t bump1=HAL_GPIO_ReadPin(CLIFF_1_GPIO_Port, CLIFF_1_Pin)==EDGE_SENSOR_ACTIVE?BUMP_BIT_LEFT:0;
	uint32_t bump2=HAL_GPIO_ReadPin(CLIFF_2_GPIO_Port, CLIFF_2_Pin)==EDGE_SENSOR_ACTIVE?BUMP_BIT_RIGHT:0;

	return bump1 | bump2; // build bitmap of sensor states
}

// run a debounce filter using a parallel 2bit counter
// should be called every 10ms to give 30ms deglitch timing
uint32_t debounce(uint32_t sample)
{
    static uint32_t state, x0, x1;
    uint32_t delta;
    delta = sample ^ state;
    x1 = (x1 ^ x0) & delta;
    x0 = ~x0 & delta;
    state ^= (x0 & x1);
    return state;
}
