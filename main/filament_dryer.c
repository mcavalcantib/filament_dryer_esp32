/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <driver/adc.h>
#include <driver/ledc.h>
#include <driver/timer.h>
#include <esp_adc_cal.h>
#include <esp_log.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lcd1602_i2c.h"
#include "sdkconfig.h"

#define ADC_CHANNEL ADC1_CHANNEL_4
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_11
#define BETA 3950
#define R_REF 10000.0
#define R_NTC 100000.0
#define T0 298.15 // 25 + 273.15

#define SAMPLING_FREQUENCY 20 // HZ
#define PID_MAXOUTPUT \
    1023                                        // Maximum control output magnitude (change this if your
                                                // microcontroller has a greater max value)
#define SAMPLING_TIME 1000 / SAMPLING_FREQUENCY // Sampling time (seconds)
#define CONSTANT_Kp 20.0f                       // Proportional gain
#define CONSTANT_Ki 0.0f                        // Integral gain times SAMPLING_TIME
#define CONSTANT_Kd 10.0f                       // Derivative gain divided by SAMPLING_TIME
#define MOVING_AVERAGE_SIZE 10

static const char *TAG = "Filament_Dryer";

static volatile float currentTemperature = 0.0;
static volatile uint32_t targetTemperature = 0.0;
static volatile bool isHeating = false, dataPulse = false;
volatile uint16_t output = 0;
uint8_t movingAveragePosition = 0;
int16_t temperatureArray[MOVING_AVERAGE_SIZE];
int32_t integral = 0;
char buffer[16];

esp_adc_cal_characteristics_t adc_chars;
static ledc_channel_config_t ledc_channel;

uint16_t PID_update(uint16_t currentTemperature)
{
    // e[k] = r[k] - y[k], error between setpoint and true position
    int16_t error = targetTemperature - currentTemperature;
    // e_d[k] = (e_f[k] - e_f[k-1]) / Tₛ, filtered derivative
    int16_t derivative = round(error / SAMPLING_TIME);
    // e_i[k+1] = e_i[k] + Tₛ e[k], integral
    int16_t new_integral = round(integral + error * SAMPLING_TIME);

    // PID formula:
    // u[k] = Kp e[k] + Ki e_i[k] + Kd e_d[k], control signal
    int16_t control_u = round(CONSTANT_Kp * error + CONSTANT_Ki * integral +
                              CONSTANT_Kd * derivative);
    // Clamp the output
    if (control_u > PID_MAXOUTPUT)
        control_u = PID_MAXOUTPUT;
    else if (control_u < 0)
        control_u = 0;
    else // Anti-windup
        integral = new_integral;

    return (uint16_t)control_u;
}

float ThermistorReading()
{
    float voltage_read =
        esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC_CHANNEL), &adc_chars);
    float resistance = (R_REF * voltage_read) / (3300 - voltage_read);
    float logR = log(resistance / R_NTC);
    float output = 1 / (logR / BETA + 1 / T0) - 273.15;
    return output;
}

// ISR handler
void TemperatureReadTask(void *arg)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    // BaseType_t xWasDelayed;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        temperatureArray[movingAveragePosition] = round(ThermistorReading());
        movingAveragePosition =
            (movingAveragePosition + 1) % MOVING_AVERAGE_SIZE;
        int temperatureSum = 0;
        for (int i = 0; i < MOVING_AVERAGE_SIZE; i++)
        {
            temperatureSum += temperatureArray[i];
        }
        currentTemperature = round(temperatureSum / MOVING_AVERAGE_SIZE);
        // xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);
        output = PID_update(currentTemperature);
        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, output);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void IRAM_ATTR gpio_isr_handler()
{
    if (gpio_get_level(GPIO_NUM_19) == 1)
    {
        if (gpio_get_level(GPIO_NUM_19) == gpio_get_level(GPIO_NUM_18))
        {
            targetTemperature += 10;
        }
        else
        {
            targetTemperature -= 10;
        }
    }
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void app_main()
{
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, 0, &adc_chars);

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.duty = 0;
    ledc_channel.gpio_num = GPIO_NUM_23;
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.hpoint = 0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&ledc_channel);

    // #18
    gpio_config_t io_config = {.intr_type = GPIO_INTR_NEGEDGE,
                               .pin_bit_mask = (1ULL << GPIO_NUM_18),
                               .mode = GPIO_MODE_INPUT,
                               .pull_up_en = 1,
                               .pull_down_en = 0};

    gpio_config(&io_config);

    gpio_config_t io_config2 = {.pin_bit_mask = (1ULL << GPIO_NUM_19),
                                .mode = GPIO_MODE_INPUT,
                                .pull_up_en = 1,
                                .pull_down_en = 0};

    gpio_config(&io_config2);

    // gpio_set_intr_type(GPIO_NUM_18, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    esp_err_t result =
        gpio_isr_handler_add(GPIO_NUM_18, gpio_isr_handler, (void *)1);
    if (result == ESP_OK)
    {
        ESP_LOGI(TAG, "Handler cadastrado");
    }
    else
    {
        ESP_LOGI(TAG, "Handler não cadastrado");
    }

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    lcd_init();
    lcd_clear();

    // sprintf(buffer, "val=%.2f", num);
    // lcd_put_cur(0, 0);
    // lcd_send_string(buffer);

    for (int i = 0; i < MOVING_AVERAGE_SIZE; i++)
    {
        temperatureArray[i] = round(ThermistorReading());
    }
    char message[16];
    xTaskCreate(&TemperatureReadTask, "TemperatureReadTask", 2048, NULL, 5,
                NULL);
    while (true)
    {
        memset(message, 0, 16);
        sprintf(message, "%d/%d   ", (int)currentTemperature,
                (int)targetTemperature);

        lcd_put_cur(0, 0);
        lcd_send_string("Heater: off");

        lcd_put_cur(1, 0);
        lcd_send_string(message);
        ESP_LOGI(TAG, "Temperature: %d°C/%d°C | Output: %d | Heat: %s",
                 (int)currentTemperature, (int)targetTemperature, output,
                 isHeating ? "ON" : "OFF");

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}