/*
 * dth11.h
 *
 *  Created on: May 27, 2023
 *      Author: Embedded Wala
 */

#ifndef MK_DHT11_H
#define MK_DHT11_H

#include "main.h"

/**
 * @brief Structure representing the DHT11 sensor.
 */
typedef struct dht11_t
{
    GPIO_TypeDef *port;         /**< GPIO port used for communication with the sensor. */
    uint16_t pin;               /**< GPIO pin used for communication with the sensor. */
    TIM_HandleTypeDef *htim;    /**< Timer handler for timing purposes. */
    uint8_t temperature;        /**< Last read temperature value from the sensor. */
    uint8_t humidity;           /**< Last read humidity value from the sensor. */
} dht11_t;

void DHT11_Init(dht11_t *dht, TIM_HandleTypeDef *htim, GPIO_TypeDef *port, uint16_t pin);
void DHT11_Set_GPIO_Mode(dht11_t *dht, uint32_t pMode);
uint8_t DHT11_Read_Bit(dht11_t *dht, uint32_t timeout);
uint8_t DHT11_Read_Byte(uint8_t bitArray[]);
uint16_t DHT11_Read_Signal(dht11_t *dht, uint32_t timeout, GPIO_PinState state);
uint8_t DHT11_Start(dht11_t *dht, uint16_t readTimeoutUs);
uint8_t DHT11_Read(dht11_t *dht);

#endif /* MK_DHT11_H */
