/*
 * DHT11.h
 *
 *  Created on: Apr 30, 2020
 *      Author: Controllerstech
 */

#include "stm32f3xx_hal.h"


void DHT_Delay (uint16_t time);

void DHT_Start (uint8_t DHT);

uint8_t DHT_Check_Response (void);

uint8_t DHT_Read (void);

uint8_t DHT_Get_Data (float *Temperature, float *Humidity, uint8_t DHT);

