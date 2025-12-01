#include "max30100.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "MAX30100";

/* ---------------- BUFFER ---------------- */
#define BUFFER_SIZE   200
static float ir_ac_buf[BUFFER_SIZE];
static float red_ac_buf[BUFFER_SIZE];
static int buf_index = 0;

/* ---------------- FILTER ---------------- */
typedef struct {
    float dc;
    float ac;
} dc_filter_t;

static dc_filter_t ir_filter = {0};
static dc_filter_t red_filter = {0};

static float dc_filter(dc_filter_t *f, float sample)
{
    f->dc += 0.01f * (sample - f->dc);
    float ac = sample - f->dc;
    f->ac = ac;
    return ac;
}

/* ---------------- BPM VARS ---------------- */
static uint32_t last_peak_time = 0;
static float bpm = 0;

static inline int detect_peak(float cur, float prev, float th)
{
    return (cur > th && prev <= th);
}

/* ------------------------------------------------------------------ */
/*                           INIT SENSOR                              */
/* ------------------------------------------------------------------ */
esp_err_t max30100_init(void)
{
    uint8_t part, rev;

    if (i2c_bus_read_byte(MAX30100_I2C_ADDR, MAX30100_REG_PART_ID, &part) != ESP_OK)
        return ESP_FAIL;

    i2c_bus_read_byte(MAX30100_I2C_ADDR, MAX30100_REG_REV_ID, &rev);

    ESP_LOGI(TAG, "MAX30100 detected PART=0x%02X REV=0x%02X", part, rev);

    /* Reset */
    i2c_bus_write_byte(MAX30100_I2C_ADDR, MAX30100_REG_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* SPO2 config (Hi-res + 100 Hz + 1600 us) */
    uint8_t spo2_cfg = (1 << 6) | (2 << 2) | 3;
    i2c_bus_write_byte(MAX30100_I2C_ADDR, MAX30100_REG_SPO2_CONFIG, spo2_cfg);

    /* LED: IR = 27 mA, Red = 27 mA */
    uint8_t led_cfg = (0x0C << 4) | 0x0C;
    i2c_bus_write_byte(MAX30100_I2C_ADDR, MAX30100_REG_LED_CONFIG, led_cfg);

    /* SPO2 mode */
    i2c_bus_write_byte(MAX30100_I2C_ADDR, MAX30100_REG_MODE_CONFIG, 0x03);

    max30100_reset_fifo();
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*                           FIFO READING                             */
/* ------------------------------------------------------------------ */
static esp_err_t max30100_read_fifo(uint16_t *ir, uint16_t *red)
{
    uint8_t raw[4];

    if (i2c_bus_read_bytes(MAX30100_I2C_ADDR, MAX30100_REG_FIFO_DATA, raw, 4) != ESP_OK)
        return ESP_FAIL;

    *ir  = (raw[0] << 8) | raw[1];
    *red = (raw[2] << 8) | raw[3];
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*                       MAIN PROCESS FUNCTION                         */
/* ------------------------------------------------------------------ */
esp_err_t max30100_get_reading(max30100_reading_t *r)
{
    uint16_t ir_raw, red_raw;
    if (max30100_read_fifo(&ir_raw, &red_raw) != ESP_OK)
    {
        r->valid = false;
        return ESP_FAIL;
    }

    /* DC + AC Filter */
    float ir_ac  = dc_filter(&ir_filter,  ir_raw);
    float red_ac = dc_filter(&red_filter, red_raw);

    /* Save AC for SpO2 calculation */
    ir_ac_buf[buf_index]  = ir_ac;
    red_ac_buf[buf_index] = red_ac;
    buf_index = (buf_index + 1) % BUFFER_SIZE;

    /* Finger detection */
    if (ir_filter.dc < 3000) {
        r->valid = false;
        r->heart_rate = 0;
        r->spo2 = 0;
        return ESP_OK;
    }

    /* --------------------- BPM Peak Detection --------------------- */
    static float prev_ir_ac = 0;
    float th = ir_filter.dc * 0.015f;

    int peak = detect_peak(ir_ac, prev_ir_ac, th);
    prev_ir_ac = ir_ac;

    uint32_t now = esp_timer_get_time() / 1000; // ms

    if (peak) {
        if (last_peak_time > 0) {
            uint32_t dt = now - last_peak_time;

            if (dt > 300 && dt < 2000) {       // 30–200 BPM
                float inst_bpm = 60000.0f / dt;
                bpm = bpm * 0.7f + inst_bpm * 0.3f;
            }
        }
        last_peak_time = now;
    }

    r->heart_rate = bpm;

    /* --------------------- SpO2 Computation ----------------------- */
    float ir_rms = 0, red_rms = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        ir_rms  += fabsf(ir_ac_buf[i]);
        red_rms += fabsf(red_ac_buf[i]);
    }
    ir_rms  /= BUFFER_SIZE;
    red_rms /= BUFFER_SIZE;

    if (ir_rms < 0.5f || red_rms < 0.5f) {
        r->valid = false;
        return ESP_OK;
    }

    /* Ratio AC/DC */
    float ratio = (red_rms / red_filter.dc) / (ir_rms / ir_filter.dc);

    /* Empirical best-fit curve (from Maxim AN6409) */
    float spo2 = -45.060f * ratio * ratio + 30.354f * ratio + 94.845f;
    if (spo2 > 100) spo2 = 100;
    if (spo2 < 70)  spo2 = 70;

    r->spo2 = spo2;
    r->valid = true;

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*                           EXTRA FUNCTIONS                          */
/* ------------------------------------------------------------------ */
esp_err_t max30100_set_mode(bool enable)
{
    return i2c_bus_write_byte(
        MAX30100_I2C_ADDR,
        MAX30100_REG_MODE_CONFIG,
        enable ? 0x03 : 0x80
    );
}

esp_err_t max30100_reset_fifo(void)
{
    i2c_bus_write_byte(MAX30100_I2C_ADDR, MAX30100_REG_FIFO_WR_PTR, 0x00);
    i2c_bus_write_byte(MAX30100_I2C_ADDR, MAX30100_REG_FIFO_RD_PTR, 0x00);
    i2c_bus_write_byte(MAX30100_I2C_ADDR, MAX30100_REG_OVRFLOW_CTR, 0x00);
    return ESP_OK;
}
