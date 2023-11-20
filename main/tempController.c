static volatile float currentTemperature = 0.0, relativeHumidity = 0.0, absoluteHumidity = 0.0, airTemperature = 0.0;
uint8_t movingAveragePosition = 0;
int16_t temperatureArray[MOVING_AVERAGE_SIZE];
int32_t integral = 0;

//Custom Main functions
float calculateAbsoluteHumidity(float hum, float temp)
{
    float UA = ((6.112 * (pow(e, ((17.67 * temp) / (temp + 243.5)))) * hum * 2.1674) / (273.15 + temp));
    return UA;
}

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

uint8_t TargetTemperatureCalc(int currentTemperature){
    uint8_t error = targetTemperature - currentTemperature;
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
    const TickType_t xFrequency = pdMS_TO_TICKS(1000/SAMPLING_FREQUENCY);
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        temperatureArray[movingAveragePosition] = round(ThermistorReading());
        movingAveragePosition = (movingAveragePosition + 1) % MOVING_AVERAGE_SIZE;
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

void AirStatusReadTask(void *arg)
{

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(2500);
    xLastWakeTime = xTaskGetTickCount();

    setDHTgpio(GPIO_NUM_32);
    while (1)
    {
        int ret = readDHT();
        errorHandler(ret);
        relativeHumidity = getHumidity();
        airTemperature = getTemperature();
        absoluteHumidity = calculateAbsoluteHumidity(relativeHumidity, airTemperature);
        ESP_LOGI(TAG, "rel Hum: %.1f abs Hum: %.1f Tmp: %.1f\n", relativeHumidity, absoluteHumidity, airTemperature);
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}