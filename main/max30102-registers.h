/**
 * @file max30102-registers.h
 * 
 * @author
 * Joshua D John 
 * email: joshdumo@live.com
 * 
 *
 * 
 * @brief This is the "private" headers file for the MAX30102 ESP32 library. This is where there 
 * are significant differences between the register structures with the MAX30100.
 * 
 * Please do NOT include in your code.
 * 
 * This work is adapted from Angelo Elias Dalzotto's library for the MAX30100
*/

#ifndef MAX30102_REGISTERS_H
#define MAX30102_REGISTERS_H

#include "max30102.h"

/**
 * MAX30102 internal registers definitions.
 */
#define MAX30102_DEVICE                   0x57
// Part ID
#define MAX30102_REV_ID                   0xFE
#define MAX30102_PART_ID                  0xFF
// Status
// - Interrupts
#define MAX30102_INT_STATUS               0x00
#define MAX30102_INT_STATUS_2             0x01  
#define MAX30102_INT_ENABLE               0x02
#define MAX30102_INT_ENABLE_2             0x03
// - FIFO  
#define MAX30102_FIFO_WRITE               0x04
#define MAX30102_FIFO_OVERFLOW_COUNTER    0x05
#define MAX30102_FIFO_READ                0x06
#define MAX30102_FIFO_DATA                0x07
// Configuration
#define MAX30102_FIFO_CONF                0x08  
#define MAX30102_MODE_CONF                0x09
#define MAX30102_SPO2_CONF                0x0A
#define MAX30102_LED_CONF                 0x0C  
#define MAX30102_LED_CONF_2               0x0D
#define MAX30102_MULTILED_REG             0x11
#define MAX30102_MULTILED_REG_2           0x12
// Die Temperature    
#define MAX30102_TEMP_INT                 0x1F
#define MAX30102_TEMP_FRACTION            0x20
#define MAX30102_TEMP_CONFIG              0x21  

/**
 * Bit defines for mode configuration.
 */
#define MAX30102_MODE_SHDN                (1<<7)
#define MAX30102_MODE_RESET               (1<<6)
#define MAX30102_MODE_TEMP_EN             (1<<1)
#define MAX30102_SPO2_HI_RES_EN           (1<<6)
#define MAX30102_FIFO_ROLL_OVER_EN        (1<<4)  

/**
 * Pulse state machine Enum.
 */
typedef enum _pulse_state_machine {
    MAX30102_PULSE_IDLE,
    MAX30102_PULSE_TRACE_UP,
    MAX30102_PULSE_TRACE_DOWN
} pulse_state_machine;

/**
 * "Private" functions declarations.
 * These functions will only be called by the library, and not by the user.
 */

/**
 * @brief Pulse detection algorithm.
 * 
 * @details Called by the update function.
 * 
 * @param this is the address of the configuration structure.
 * @param sensor_value is the value read from the sensor after the filters
 * 
 * @returns true if detected.
 */
bool max30102_detect_pulse(max30102_config_t* this, float sensor_value);

/**
 * @brief Balance intensities filter.
 * 
 * @details DC filter for the raw values.
 * 
 * @param this is the address of the configuration structure.
 * @param red_dc is the w in red led dc filter structure.
 * @param ir_dc is the w in ir led dc filter structure.
 * 
 * @returns status of execution.
 */
esp_err_t max30102_balance_intensities(max30102_config_t* this, float red_dc, float ir_dc);

/**
 * @brief Write to the MAX30102 register.
 * 
 * @param this is the address of the configuration structure.
 * @param address is the register address.
 * @param val is the byte to write.
 * 
 * @returns status of execution.
 */
esp_err_t max30102_write_register(max30102_config_t* this, uint8_t address, uint8_t val);

/**
 * @brief Read from MAX30102 register.
 * 
 * @param this is the address of the configuration structure.
 * @param address is the register address.
 * @param reg is the address to save the byte read.
 * 
 * @returns status of execution.
 */
esp_err_t max30102_read_register(max30102_config_t* this, uint8_t address, uint8_t* reg);

/**
 * @brief Read set of MAX30102 registers.
 * 
 * @param this is the address of the configuration structure.
 * @param address is the register address.
 * @param data is the initial address to save.
 * @param size is the size of the register to read.
 * 
 * @returns status of execution.
 */
esp_err_t max30102_read_from( max30102_config_t* this, uint8_t address, 
                              uint8_t* data, uint8_t size );

/**
 * @brief Read from MAX30102 FIFO.
 * 
 * @param this is the address of the configuration structure.
 * @param fifo is the address to a FIFO structure.
 * 
 * @returns status of execution.
 */
esp_err_t max30102_read_fifo(max30102_config_t* this, max30102_fifo_t* fifo);

/**
 * @brief Removes the DC offset of the sensor value.
 * 
 * @param this is the address of the configuration structure.
 * @param x is the raw value read from the led.
 * @param prev_w is the previous filtered w from the dc filter structure.
 * @param alpha is the dc filter alpha parameter.
 * 
 * @returns a dc filter structure.
 */
max30102_dc_filter_t max30102_dc_removal(float x, float prev_w, float alpha);

/**
 * @brief Applies the mean diff filter.
 * 
 * @param this is the address of the configuration structure.
 * @param M is the filtered DC result of the dc filter structure.
 * 
 * @returns a filtered value.
 */
float max30102_mean_diff(max30102_config_t* this, float M);

/**
 * @brief Applies a low-pass butterworth filter.
 * 
 * @param this is the address of the configuration structure.
 * @param x is the mean diff filtered value.
 */
void max30102_lpb_filter(max30102_config_t* this, float x);

#endif

/**
 * MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/