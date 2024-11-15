/*
 * connectivity.c
 *
 *  Created on: Aug 27, 2024
 *      Author: thuanphat7
 */
#include "cJSON.h"
#include "config.h"
#include "main.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
extern UART_HandleTypeDef huart1;
char array_json[300];

void send_to_simcom_a76xx(char *cmd) {
  printf("STM32 Write: %s", cmd);
  HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 1000);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    printf("\r\nSIMCOM Response:");
    printf(rx_buffer);
    static int times;
        times = strlen(rx_buffer);
        for (int i = 0; i < times; i++) {
          rx_data_sim[i] = rx_buffer[i];
        }
    if ((strstr((char *)rx_buffer, "+CMQTTCONNLOST") != NULL) &&
        is_pb_done == true) {
      printf(
          "-----------------Client Disconnect passively!------------------\n");
      CurrentStatusSimcom = On;
    }
    memset(rx_buffer, '\0', 150);
  }
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)rx_buffer, 150);
}

