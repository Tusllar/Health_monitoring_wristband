
#ifndef MAX30100_H
#define MAX30100_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* MAX30100 I2C Address */
#define MAX30100_I2C_ADDR            0x57

/* MAX30100 Register Map */
#define MAX30100_REG_INT_STATUS      0x00
#define MAX30100_REG_INT_ENABLE      0x01
#define MAX30100_REG_FIFO_WR_PTR     0x02
#define MAX30100_REG_OVRFLOW_CTR     0x03
#define MAX30100_REG_FIFO_RD_PTR     0x04
#define MAX30100_REG_FIFO_DATA       0x05
#define MAX30100_REG_MODE_CONFIG     0x06
#define MAX30100_REG_SPO2_CONFIG     0x07
#define MAX30100_REG_LED_CONFIG      0x09
#define MAX30100_REG_TEMP_INT        0x16
#define MAX30100_REG_TEMP_FRAC       0x17
#define MAX30100_REG_REV_ID          0xFE
#define MAX30100_REG_PART_ID         0xFF

/* User output struct */
typedef struct {
    float heart_rate;
    float spo2;
    bool valid;
} max30100_reading_t;

/* Public API */
esp_err_t max30100_init(void);
esp_err_t max30100_get_reading(max30100_reading_t *out);
esp_err_t max30100_set_mode(bool enable);
esp_err_t max30100_reset_fifo(void);

#endif


