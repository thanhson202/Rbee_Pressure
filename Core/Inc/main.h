/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                       uint16_t Size);
extern TIM_HandleTypeDef htim6;
extern float read_ss(void);
extern void enable_simcom(void);
extern void sleep_stm32(void);
extern int connectMQTT(void);
extern void send_to_simcom_a76xx(char *cmd);
extern void create_JSON(void);
extern float fn_check_signal_simcom(void);
extern int acquire_gsm_mqtt_client(void);
extern int enable_mqtt_on_gsm_modem(void);
extern int connect_mqtt_server_by_gsm(void);
extern int subscribe_mqtt_via_gsm(void);
extern int publish_mqtt_via_gsm(void);
extern int read_signal_quality(void);
extern int stop_mqtt_via_gsm(void);
extern int check_error_mqtt_via_gsm(void);
extern int update_status(void);
extern void restart_stm32(void);
extern int event_wait_function(void);
extern int check_active_payload(void);
extern void read_flash_payload(void);
extern void init_flash(void);
extern float Level_Pin(void);
extern char rx_buffer[150];
extern char rx_data_sim[150];
extern char at_command[300];
extern int is_pb_done;
extern int previousTick;
extern int timeOutConnectMQTT;
extern int payLoadPin;
extern char array_json[300];
extern float data_percentage_pin;
extern float signal_strength;
extern int rssi;
extern int is_connect_simcom;
extern bool is_fn_enable_mqtt;
extern bool is_fn_connect_mqtt;
extern bool is_fn_check_sim;
extern bool is_fn_subcribe_mqtt;
extern bool is_fn_publish_mqtt;
extern bool is_fn_acquier_mqtt;
extern bool is_fn_update_status;
extern float float_sensor_pressre;
extern GPIO_TypeDef *GPIO_LOAD_PORT[4];
extern unsigned int GPIO_LOAD_PIN[4];
enum GmsModemState {
  Off,
  On,
  InternetReady,
  MqttReady,
  Subscribed,
  UpdateToServer,
  SleepRbee
};
extern enum GmsModemState CurrentStatusSimcom;

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENABLE_SENSOR_Pin GPIO_PIN_4
#define ENABLE_SENSOR_GPIO_Port GPIOA
#define LED_STATUS_Pin GPIO_PIN_12
#define LED_STATUS_GPIO_Port GPIOB
#define A76XX_PWRKEY_Pin GPIO_PIN_11
#define A76XX_PWRKEY_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
