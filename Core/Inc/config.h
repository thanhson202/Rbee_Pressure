/*
 * config.h
 *
 *  Created on: AUG 1, 2024
 *      Author: thuanphat7
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "stdio.h"
#include <string.h>

// Codename of the farm, where we deploy this node to.

#define FARM "demox"
//#define FARM "demox"
#define a7672s 1
#define a7670c 2
#define a7670sa 3

#define SIMCOM_MODEL a7670c // #default is a7670c if you use model other please choose enter your
         // model

// hiệu chỉnh giá trị sensor
#define KALMAN_ADC_MIN 785
#define KALMAN_ADC_MAX 4095
#define DISTANCE_MIN 0
#define DISTANCE_MAX 102

#define INTERVAL_PUPLISH_DATA 10 // the time the device sends data to the server
#define TIME_SLEEP 600
// Serial number. Must be lower case.
#ifndef SERIAL_NUMBER
#define SERIAL_NUMBER "rb000031"
#endif

#define MQTT_USER "node"   // User - connect to MQTT broker
#define MQTT_PASS "654321" // Password - connect to MQTT broker

//#define MQTT_USER "mqttnode"       // User - connect to MQTT broker
//#define MQTT_PASS "congamo"		// Password - connect to MQTT broker

#define MQTT_TOPIC_ACTUATOR_STATUS FARM "/sn/" SERIAL_NUMBER // "/as/"
// MQTT topic to subscribe and get command to switch on/off actuator
#define MQTT_TOPIC_ACTUATOR_CONTROL FARM "/snac/" SERIAL_NUMBER "/"
/** MQTT
 * Global broker: mqtt.agriconnect.vn`
 */
#define MQTT_HOST "tcp://mqtt.agriconnect.vn" // MQTT broker

#define MQTT_CLIENT_ID SERIAL_NUMBER
#define MQTT_PORT 1883

#define TIME_PERIOD ((2000000*TIME_SLEEP)/60000)-1

#endif /* INC_CONFIG_H_ */
