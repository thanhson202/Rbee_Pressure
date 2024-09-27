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

#define SIMCOM_MODEL a7670sa // #default is a7670c if you use model other please choose enter your model
// Serial number. Must be lower case.
#ifndef SERIAL_NUMBER
  #define SERIAL_NUMBER "rb000032"
#endif

#define MQTT_USER "node" 		// User - connect to MQTT broker
#define MQTT_PASS "654321"		// Password - connect to MQTT broker

//#define MQTT_USER "mqttnode"       // User - connect to MQTT broker
//#define MQTT_PASS "congamo"		// Password - connect to MQTT broker

#define MQTT_TOPIC_ACTUATOR_STATUS FARM "/sn/" SERIAL_NUMBER // "/as/"
// MQTT topic to subscribe and get command to switch on/off actuator
#define MQTT_TOPIC_ACTUATOR_CONTROL FARM "/snac/" SERIAL_NUMBER "/"
/** MQTT
 * Global broker: mqtt.agriconnect.vn`
 */
#define MQTT_HOST "tcp://mqtt.agriconnect.vn"           		// MQTT broker

#define MQTT_PORT 1883




#define NUMBER_LOADS 4
#define MQTT_CLIENT_ID  SERIAL_NUMBER
#define LENGTH_STATUS_PAYLOAD_0_9  6*NUMBER_LOADS + 1
#define LENGTH_STATUS_PAYLOAD_10_18  6*NUMBER_LOADS + 2 + NUMBER_LOADS%10


#endif /* INC_CONFIG_H_ */
