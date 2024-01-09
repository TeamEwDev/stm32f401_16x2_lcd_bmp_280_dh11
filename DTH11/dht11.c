/*
 * dth11.c
 *
 *  Created on: May 27, 2023
 *      Author: Embedded Wala
 */

/* Includes ------------------------------------------------------------------*/
#include "dht11.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"

/**
 * @brief configure dht11 struct with given parameter
 * @param htim TIMER for calculate delays ex:&htim2
 * @param port GPIO port ex:GPIOA
 * @param pin GPIO pin ex:GPIO_PIN_2
 * @param dht struct to configure ex:&dht
 */
void DHT11_Init(dht11_t *dht, TIM_HandleTypeDef *htim, GPIO_TypeDef *port, uint16_t pin)
{
    dht->htim = htim;
    dht->port = port;
    dht->pin = pin;
}

/**
 * @brief Set the direction of the DHT11 sensor pin with the specified mode.
 * @param dht Pointer to the DHT11 structure.
 * @param pMode GPIO mode to be set (e.g., GPIO_MODE_INPUT or GPIO_MODE_OUTPUT_PP).
 */
void DHT11_Set_GPIO_Mode(dht11_t *dht, uint32_t pMode)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = dht->pin;
    GPIO_InitStruct.Mode = pMode;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(dht->port, &GPIO_InitStruct);
}

/**
 * @brief Read a single bit from the DHT11 sensor.
 * @param dht Pointer to the DHT11 structure.
 * @param timeout Maximum timeout value for reading the bit.
 * @return The read bit (0 or 1) or TIMEOUT_ERROR if the bit reading times out.
 */
uint8_t DHT11_Read_Bit(dht11_t *dht, uint32_t timeout)
{
    uint16_t mTimeUs;
    uint8_t mBit;

    __HAL_TIM_SET_COUNTER(dht->htim, 0);
    while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_RESET)
    {
        if ((uint16_t)__HAL_TIM_GET_COUNTER(dht->htim) > timeout)
        {
            return 0;
        }
    }
    __HAL_TIM_SET_COUNTER(dht->htim, 0);
    while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_SET)
    {
        if ((uint16_t)__HAL_TIM_GET_COUNTER(dht->htim) > timeout)
        {
            return 0;
        }
    }

    mTimeUs  = (uint16_t)__HAL_TIM_GET_COUNTER(dht->htim);

    if (mTimeUs > 20 && mTimeUs < 30)
    {
        mBit = 0;
    }
    else if (mTimeUs > 60 && mTimeUs < 80) //if pass time 70 uS set as HIGH
    {
        mBit = 1;
    }
    return mBit;
}

/**
 * @brief Read a byte of data from the DHT11 sensor.
 * @param bitArray Array holding the individual bits to be assembled into a byte.
 * @return The assembled byte of data.
 */
uint8_t DHT11_Read_Byte(uint8_t bitArray[])
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++)
    {
        if (bitArray[i])
            byte |= (1 << (7 - i));
        else
            byte &= ~(1 << (7 - i));
    }
    return byte;
}

/**
 * @brief Read the duration of a signal (either high or low) from the DHT11 sensor.
 * @param dht Pointer to the DHT11 structure.
 * @param timeout Maximum timeout value for reading the signal.
 * @param state Desired GPIO_PinState to read (GPIO_PIN_SET for high, GPIO_PIN_RESET for low).
 * @return The duration of the signal in microseconds or TIMEOUT_ERROR if the signal reading times out.
 */
uint16_t DHT11_Read_Signal(dht11_t *dht, uint32_t timeout, GPIO_PinState state)
{
    uint16_t signalTimeUs = 0;
    uint32_t delayCnt = 0;

    while (HAL_GPIO_ReadPin(dht->port, dht->pin) == state)
    {
        if (delayCnt > timeout)
        {
            return timeout;
        }
        Delay_Us(1);
        delayCnt++;
        signalTimeUs++;
    }

    return signalTimeUs;
}

/**
 * @brief Start communication with the DHT11 sensor by initiating the data transmission.
 * @param dht Pointer to the DHT11 structure.
 * @param readTimeoutUs Maximum timeout value for reading sensor responses in microseconds.
 */
uint8_t DHT11_Start(dht11_t *dht, uint16_t readTimeoutUs)
{
    uint16_t mTimeUs1 = 0, mTimeUs2 = 0;

    DHT11_Set_GPIO_Mode(dht, GPIO_MODE_OUTPUT_PP);
    HAL_GPIO_WritePin(dht->port, dht->pin, GPIO_PIN_RESET);
    HAL_Delay(18);
    DHT11_Set_GPIO_Mode(dht, GPIO_MODE_INPUT);

    DHT11_Read_Signal(dht, readTimeoutUs, GPIO_PIN_SET);
    mTimeUs1 = DHT11_Read_Signal(dht, readTimeoutUs, GPIO_PIN_RESET);
    mTimeUs2 = DHT11_Read_Signal(dht, readTimeoutUs, GPIO_PIN_SET);
    if (mTimeUs1 >= 75 && mTimeUs1 <= 85 && mTimeUs2 >= 75 && mTimeUs2 <= 85)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief Read data from the DHT11 sensor, including temperature and humidity values.
 * @param dht Pointer to the DHT11 structure.
 * @return DHT11_OK if data reading is successful, or DHT11_ERROR if an error occurs.
 */
uint8_t DHT11_Read(dht11_t *dht)
{
    uint8_t humidity = 0, temperature = 0, parity = 0, genParity = 0;
    uint8_t mBitsData[40];
    uint16_t readTimeOutUs = 500;

    DHT11_Start(dht, readTimeOutUs);

    for (int j = 0; j < 40; j++)
    {
        mBitsData[j] = DHT11_Read_Bit(dht, readTimeOutUs);
    }

    humidity = DHT11_Read_Byte(&mBitsData[0]);
    temperature = DHT11_Read_Byte(&mBitsData[16]);
    parity = DHT11_Read_Byte(&mBitsData[32]);
    genParity = humidity + temperature + DHT11_Read_Byte(&mBitsData[8]) + DHT11_Read_Byte(&mBitsData[24]);

    dht->temperature = -1;
    dht->humidity = -1;
    if (genParity == parity)
    {
        dht->temperature = temperature;
        dht->humidity = humidity;
        return 0;
    }

    return 1;
}
