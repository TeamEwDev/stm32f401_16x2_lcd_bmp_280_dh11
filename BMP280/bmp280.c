/*
 * bmp280.c
 *
 *  Created on: May 27, 2023
 *      Author: Embedded Wala
 */

/* Includes ------------------------------------------------------------------*/
#include "BMP280.h"
#include "math.h"

/**
 * BMP280 registers
 */
#define BMP280_REG_TEMP_XLSB   0xFC /* bits: 7-4 */
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_TEMP        (BMP280_REG_TEMP_MSB)
#define BMP280_REG_PRESS_XLSB  0xF9 /* bits: 7-4 */
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_PRESSURE    (BMP280_REG_PRESS_MSB)
#define BMP280_REG_CONFIG      0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define BMP280_REG_CTRL        0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define BMP280_REG_STATUS      0xF3 /* bits: 3 measuring; 0 im_update */
#define BMP280_REG_CTRL_HUM    0xF2 /* bits: 2-0 osrs_h; */
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_ID          0xD0
#define BMP280_REG_CALIB       0x88
#define BMP280_REG_HUM_CALIB   0x88
#define BMP280_RESET_VALUE     0xB6

/**
 * @brief Initialize BMP280 parameters with default values.
 * @param params Pointer to BMP280 parameters structure.
 */
void BMP280_Init_Default_Params(BMP280_params_t *params)
{
    params->mode = BMP280_MODE_NORMAL;
    params->filter = BMP280_FILTER_OFF;
    params->oversampling_pressure = BMP280_STANDARD;
    params->oversampling_temperature = BMP280_STANDARD;
    params->oversampling_humidity = BMP280_STANDARD;
    params->standby = BMP280_STANDBY_250;
}

/**
 * @brief Read a 16-bit register value from BMP280.
 * @param dev Pointer to BMP280 device structure.
 * @param addr Register address.
 * @param value Pointer to store the read value.
 * @return True if the read operation is successful, false otherwise.
 */
static bool Read_Register16(BMP280_HandleTypedef *dev, uint8_t addr, uint16_t *value)
{
    uint16_t tx_buff;
    uint8_t rx_buff[2];
    tx_buff = (dev->addr << 1);

    if (HAL_I2C_Mem_Read(dev->i2c, tx_buff, addr, 1, rx_buff, 2, 5000)
        == HAL_OK)
    {
        *value = (uint16_t)((rx_buff[1] << 8) | rx_buff[0]);
        return true;
    }
    else
        return false;

}

/**
 * @brief Read data from BMP280.
 * @param dev Pointer to BMP280 device structure.
 * @param addr Register address.
 * @param value Pointer to store the read data.
 * @param len Number of bytes to read.
 * @return 0 on success, 1 on failure.
 */
static inline int Read_Data(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t *value,
                            uint8_t len)
{
    uint16_t tx_buff;
    tx_buff = (dev->addr << 1);
    if (HAL_I2C_Mem_Read(dev->i2c, tx_buff, addr, 1, value, len, 5000) == HAL_OK)
        return 0;
    else
        return 1;

}

/**
 * @brief Read calibration data from BMP280.
 * @param dev Pointer to BMP280 device structure.
 * @return True if calibration data reading is successful, false otherwise.
 */
static bool Read_Calibration_Data(BMP280_HandleTypedef *dev)
{

    if (Read_Register16(dev, 0x88, &dev->dig_T1)
        && Read_Register16(dev, 0x8a, (uint16_t *) &dev->dig_T2)
        && Read_Register16(dev, 0x8c, (uint16_t *) &dev->dig_T3)
        && Read_Register16(dev, 0x8e, &dev->dig_P1)
        && Read_Register16(dev, 0x90, (uint16_t *) &dev->dig_P2)
        && Read_Register16(dev, 0x92, (uint16_t *) &dev->dig_P3)
        && Read_Register16(dev, 0x94, (uint16_t *) &dev->dig_P4)
        && Read_Register16(dev, 0x96, (uint16_t *) &dev->dig_P5)
        && Read_Register16(dev, 0x98, (uint16_t *) &dev->dig_P6)
        && Read_Register16(dev, 0x9a, (uint16_t *) &dev->dig_P7)
        && Read_Register16(dev, 0x9c, (uint16_t *) &dev->dig_P8)
        && Read_Register16(dev, 0x9e,
                           (uint16_t *) &dev->dig_P9))
    {

        return true;
    }

    return false;
}

/**
 * @brief Read humidity calibration data from BMP280 device.
 *
 * This function reads the humidity calibration data from the BMP280 device's registers
 * and stores them in the provided BMP280 device structure. The humidity calibration
 * data is required for accurate humidity measurements.
 *
 * @param dev Pointer to the BMP280 device structure.
 * @return True if the humidity calibration data reading is successful, false otherwise.
 */
static bool Read_Hum_Calibration_Data(BMP280_HandleTypedef *dev)
{
    uint16_t h4, h5;

    if (!Read_Data(dev, 0xa1, &dev->dig_H1, 1)
        && Read_Register16(dev, 0xe1, (uint16_t *) &dev->dig_H2)
        && !Read_Data(dev, 0xe3, &dev->dig_H3, 1)
        && Read_Register16(dev, 0xe4, &h4)
        && Read_Register16(dev, 0xe5, &h5)
        && !Read_Data(dev, 0xe7, (uint8_t *) &dev->dig_H6, 1))
    {
        dev->dig_H4 = (h4 & 0x00ff) << 4 | (h4 & 0x0f00) >> 8;
        dev->dig_H5 = h5 >> 4;

        return true;
    }

    return false;
}

/**
 * @brief Write an 8-bit value to a register in the BMP280 device.
 *
 * This function writes an 8-bit value to a specific register address in the BMP280 device.
 * It is used to configure various settings and parameters of the device.
 *
 * @param dev Pointer to the BMP280 device structure.
 * @param addr Register address to write to.
 * @param value The 8-bit value to be written to the register.
 * @return True if the write operation is successful, false otherwise.
 */
static int Write_Register8(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t value)
{
    uint16_t tx_buff;

    tx_buff = (dev->addr << 1);

    if (HAL_I2C_Mem_Write(dev->i2c, tx_buff, addr, 1, &value, 1, 10000) == HAL_OK)
        return false;
    else
        return true;
}

/**
 * @brief Initialize BMP280 device with provided parameters.
 * @param dev Pointer to BMP280 device structure.
 * @param params Pointer to BMP280 parameters structure.
 * @return True if initialization is successful, false otherwise.
 */
bool BMP280_Init(BMP280_HandleTypedef *dev, BMP280_params_t *params)
{

    if (dev->addr != BMP280_I2C_ADDRESS_0
        && dev->addr != BMP280_I2C_ADDRESS_1)
    {

        return false;
    }

    if (Read_Data(dev, BMP280_REG_ID, &dev->id, 1))
    {
        return false;
    }

    if (dev->id != BMP280_CHIP_ID && dev->id != BME280_CHIP_ID)
    {

        return false;
    }

    // Soft reset.
    if (Write_Register8(dev, BMP280_REG_RESET, BMP280_RESET_VALUE))
    {
        return false;
    }

    // Wait until finished copying over the NVP data.
    while (1)
    {
        uint8_t status;
        if (!Read_Data(dev, BMP280_REG_STATUS, &status, 1)
            && (status & 1) == 0)
            break;
    }

    if (!Read_Calibration_Data(dev))
    {
        return false;
    }

    if (dev->id == BME280_CHIP_ID && !Read_Hum_Calibration_Data(dev))
    {
        return false;
    }

    uint8_t config = (params->standby << 5) | (params->filter << 2);
    if (Write_Register8(dev, BMP280_REG_CONFIG, config))
    {
        return false;
    }

    if (params->mode == BMP280_MODE_FORCED)
    {
        params->mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
    }

    uint8_t ctrl = (params->oversampling_temperature << 5)
                   | (params->oversampling_pressure << 2) | (params->mode);

    if (dev->id == BME280_CHIP_ID)
    {
        // Write crtl hum reg first, only active after write to BMP280_REG_CTRL.
        uint8_t ctrl_hum = params->oversampling_humidity;
        if (Write_Register8(dev, BMP280_REG_CTRL_HUM, ctrl_hum))
        {
            return false;
        }
    }

    if (Write_Register8(dev, BMP280_REG_CTRL, ctrl))
    {
        return false;
    }

    return true;
}

/**
 * @brief Initiate a forced measurement in the BMP280 device.
 *
 * This function triggers a forced measurement operation in the BMP280 device.
 * When forced mode is used, the device performs a single measurement of temperature and pressure
 * and then returns to sleep mode after the measurement is complete.
 *
 * @param dev Pointer to the BMP280 device structure.
 * @return True if the forced measurement initiation is successful, false otherwise.
 */
bool BMP280_Force_Measurement(BMP280_HandleTypedef *dev)
{
    uint8_t ctrl;
    if (Read_Data(dev, BMP280_REG_CTRL, &ctrl, 1))
        return false;
    ctrl &= ~0b11;  // clear two lower bits
    ctrl |= BMP280_MODE_FORCED;
    if (Write_Register8(dev, BMP280_REG_CTRL, ctrl))
    {
        return false;
    }
    return true;
}

/**
 * @brief Check if the BMP280 device is currently performing a measurement.
 *
 * This function checks the status register of the BMP280 device to determine if
 * a measurement is currently in progress.
 *
 * @param dev Pointer to the BMP280 device structure.
 * @return True if the device is actively measuring, false otherwise.
 */
bool BMP280_Is_Measuring(BMP280_HandleTypedef *dev)
{
    uint8_t status;
    if (Read_Data(dev, BMP280_REG_STATUS, &status, 1))
        return false;
    if (status & (1 << 3))
    {
        return true;
    }
    return false;
}

/**
 * @brief Compensate temperature reading using calibration data.
 * @param dev Pointer to BMP280 device structure.
 * @param adc_temp Raw temperature reading from the sensor.
 * @param fine_temp Pointer to store the calculated fine temperature.
 * @return Compensated temperature value in degrees Celsius.
 */
static inline int32_t Compensate_Temperature(BMP280_HandleTypedef *dev, int32_t adc_temp,
        int32_t *fine_temp)
{
    int32_t var1, var2;

    var1 = ((((adc_temp >> 3) - ((int32_t) dev->dig_T1 << 1)))
            * (int32_t) dev->dig_T2) >> 11;
    var2 = (((((adc_temp >> 4) - (int32_t) dev->dig_T1)
              * ((adc_temp >> 4) - (int32_t) dev->dig_T1)) >> 12)
            * (int32_t) dev->dig_T3) >> 14;

    *fine_temp = var1 + var2;
    return (*fine_temp * 5 + 128) >> 8;
}

/**
 * @brief Read fixed-point temperature, pressure, and optionally humidity from BMP280.
 * @param dev Pointer to BMP280 device structure.
 * @param temperature Pointer to store the compensated temperature in degrees Celsius.
 * @param pressure Pointer to store the compensated pressure in Pa.
 * @param humidity Pointer to store the compensated humidity in %RH.
 * @return True if reading is successful, false otherwise.
 */
static inline uint32_t Compensate_Pressure(BMP280_HandleTypedef *dev, int32_t adc_press,
        int32_t fine_temp)
{
    int64_t var1, var2, p;

    var1 = (int64_t) fine_temp - 128000;
    var2 = var1 * var1 * (int64_t) dev->dig_P6;
    var2 = var2 + ((var1 * (int64_t) dev->dig_P5) << 17);
    var2 = var2 + (((int64_t) dev->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t) dev->dig_P3) >> 8)
           + ((var1 * (int64_t) dev->dig_P2) << 12);
    var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) dev->dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0;  // avoid exception caused by division by zero
    }

    p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t) dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t) dev->dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t) dev->dig_P7 << 4);
    return p;
}

/**
 * @brief Compensate raw humidity reading using calibration data.
 *
 * This function calculates the compensated humidity value based on the provided raw
 * humidity reading and the previously calculated fine temperature value.
 *
 * @param dev Pointer to the BMP280 device structure.
 * @param adc_hum Raw humidity reading from the sensor.
 * @param fine_temp Calculated fine temperature value.
 * @return Compensated humidity value in %RH (percentage relative humidity).
 */
static inline uint32_t Compensate_Humidity(BMP280_HandleTypedef *dev, int32_t adc_hum,
        int32_t fine_temp)
{
    int32_t v_x1_u32r;

    v_x1_u32r = fine_temp - (int32_t) 76800;
    v_x1_u32r = ((((adc_hum << 14) - ((int32_t) dev->dig_H4 << 20)
                   - ((int32_t) dev->dig_H5 * v_x1_u32r)) + (int32_t) 16384) >> 15)
                * (((((((v_x1_u32r * (int32_t) dev->dig_H6) >> 10)
                       * (((v_x1_u32r * (int32_t) dev->dig_H3) >> 11)
                          + (int32_t) 32768)) >> 10) + (int32_t) 2097152)
                    * (int32_t) dev->dig_H2 + 8192) >> 14);
    v_x1_u32r = v_x1_u32r
                - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
                    * (int32_t) dev->dig_H1) >> 4);
    v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
    v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
    return v_x1_u32r >> 12;
}

/**
 * @brief Read compensated temperature, pressure, and optionally humidity values.
 *
 * This function reads the raw temperature and pressure values from the BMP280 sensor,
 * then calculates the compensated temperature and pressure values using calibration
 * data. If the sensor is a BME280, the function also calculates the compensated humidity
 * value. The calculated values are returned through the provided pointers.
 *
 * @param dev Pointer to the BMP280 device structure.
 * @param temperature Pointer to store the compensated temperature in degrees Celsius.
 * @param pressure Pointer to store the compensated pressure in Pa.
 * @param humidity Pointer to store the compensated humidity in %RH (only for BME280).
 * @return True if the reading is successful, false otherwise.
 */
bool BMP280_Read_Fixed(BMP280_HandleTypedef *dev, int32_t *temperature, uint32_t *pressure,
                       uint32_t *humidity)
{
    int32_t adc_pressure;
    int32_t adc_temp;
    uint8_t data[8];

    // Only the BME280 supports reading the humidity.
    if (dev->id != BME280_CHIP_ID)
    {
        if (humidity)
            *humidity = 0;
        humidity = NULL;
    }

    // Need to read in one sequence to ensure they match.
    size_t size = humidity ? 8 : 6;
    if (Read_Data(dev, 0xf7, data, size))
    {
        return false;
    }

    adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
    adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;

    int32_t fine_temp;
    *temperature = Compensate_Temperature(dev, adc_temp, &fine_temp);
    *pressure = Compensate_Pressure(dev, adc_pressure, fine_temp);
    return true;
}

/**
 * Calculates the altitude based on the given atmospheric pressure.
 * This function takes a pressure value as input and calculates the corresponding altitude
 * using the standard atmosphere model. It assumes a constant temperature and pressure lapse
 * rate, and provides a reasonable estimation of altitude in typical weather conditions.
 * @param pressure The atmospheric pressure in pascals.
 * @return The altitude in meters above sea level.
*/
float BMP280_Read_Altitude(float pressure)
{
    float altitude, seaLevelhPa = 1013.25;
    pressure /= 100;
    altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
    return altitude;
}

/**
 * @brief Read temperature, pressure, and optionally humidity as floating-point values.
 * @param dev Pointer to BMP280 device structure.
 * @param temperature Pointer to store the temperature in degrees Celsius.
 * @param pressure Pointer to store the pressure in Pa.
 * @param humidity Pointer to store the humidity in %RH.
 * @param The altitude in meters above sea level.
 * @return True if reading is successful, false otherwise.
 */
bool BMP280_Read_Float(BMP280_HandleTypedef *dev, float *temperature, float *pressure, float *altitude)
{
    int32_t fixed_temperature;
    uint32_t fixed_pressure;

    if (BMP280_Read_Fixed(dev, &fixed_temperature, &fixed_pressure,
                          NULL))
    {
        *temperature = (float) fixed_temperature / 100;
        *pressure = (float) fixed_pressure / 256;
        *altitude = BMP280_Read_Altitude(*pressure);
        return true;
    }

    return false;
}


