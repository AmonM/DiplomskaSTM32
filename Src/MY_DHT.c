/*
 * DHT11.c
 *
 *  Created on: Apr 30, 2020
 *      Author: Controllerstech
 */

#include <MY_DHT.h>
#include "stm32f3xx_hal.h"

extern TIM_HandleTypeDef htim2;
#define DHT_TIMER &htim2

#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_5



/********************* NO CHANGES AFTER THIS *************************************/

void DHT_Delay (uint16_t time)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(DHT_TIMER, 0);
	while ((__HAL_TIM_GET_COUNTER(DHT_TIMER))<time);
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM;

void DHT_Start (uint8_t DHT)
{
	Set_Pin_Output (DHT11_PORT, DHT11_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	if(DHT == 22){
		DHT_Delay (1300);   // wait for >1ms
	}else{
		DHT_Delay (18000);   // wait for 18ms
	}

    HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
    DHT_Delay (20);   // wait for 30us
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);    // set as input
}

uint8_t DHT_Check_Response (void)
{
	uint8_t Response = 0;
	DHT_Delay (40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		DHT_Delay (80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
		else Response = 0;
	}else{
		return 0;
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
		DHT_Delay (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));  // wait for the pin to go low
	}
	return i;
}

uint8_t DHT_Get_Data (float *Temperature, float *Humidity, uint8_t DHT)
{
	float TEMP, RH;
    DHT_Start (DHT);
	if (DHT_Check_Response ())
	{
		Rh_byte1 = DHT_Read ();
		Rh_byte2 = DHT_Read ();
		Temp_byte1 = DHT_Read ();
		Temp_byte2 = DHT_Read ();
		SUM = DHT_Read();
		if((DHT & 4) == 4){
			TEMP = (float)(((Temp_byte1<<8) | Temp_byte2)/10.0f);
		    RH =  (float)(((Rh_byte1<<8) | Rh_byte2)/10.0f);
		    *Temperature = TEMP;
		    *Humidity = RH;
		}else{
			if (SUM == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))
			{
				TEMP = (float)Temp_byte1*1.0f;
				RH =  (float)Rh_byte1*1.0f;
			    *Temperature = TEMP;
			    *Humidity = RH;
			}
			else return -1;
		}
	}
	else return -1;


    //*Temperature = (int *)TEMP;
	//*Humidity = (int *)RH;

    return 1;
}

