/*
  Original work from:

    SparkFun Electronics
    Date: 8/2019
    Author: Elias Santistevan

    This example displays a more manual method of adjusting the way in which the
    MAX30101 gathers data. Specifically we'll look at how to modify the pulse
    length of the LEDs within the MAX30101 which impacts the number of samples
    that can be gathered, so we'll adjust this value as well. In addition we
    gather additional data from the bioData type: LED samples. This data gives
    the number of samples gathered by the MAX30101 for both the red and IR LEDs.
    As a side note you can also choose MODE_ONE and MODE_TWO for configSensorBpm
    as well.

    If you run into an error code check the following table to help diagnose your
    problem:
    1 = Unavailable Command
    2 = Unavailable Function
    3 = Data Format Error
    4 = Input Value Error
    5 = Try Again
    255 = Error Unknown

  This is an example using pure ESP-IDF library.
  OBS: Be patient, wait for the sensor be ready, keep your finger on the sensor and wait...
  The first value is the BPM, the second is the SpO2. The values will be different from zero
  when the sensor is sure about the values, so many zeros will be shown. If takes too long
  try switch fingers or move your finger a little bit. Don't press too hard.

  One issue that happened to me was that the sensor was not receiving proper power from the
  3.3v pin. Try to use another power source if you get lots of timeouts.

  Finally, check if `CONFIG_FREERTOS_HZ` is set to 1000 in menuconfig. If not, set it to 1000!

*/

#include "max32664.h"

#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char TAG[] = "main";
// Reset pin, MFIO pin
gpio_num_t resPin = (gpio_num_t)I2C_MAX32664_RESET_PIN;
gpio_num_t mfioPin = (gpio_num_t)I2C_MAX32664_MFIO_PIN;
gpio_num_t I2C_SDA_IO = (gpio_num_t)I2C_MASTER_SDA; /*!< gpio number for I2C master data  */
gpio_num_t I2C_SCL_IO = (gpio_num_t)I2C_MASTER_SCL; /*!< gpio number for I2C master clock */

// Possible widths: 69, 118, 215, 411us
uint16_t width = I2C_MAX32664_PULSE_WIDTH;
// Possible samples: 50, 100, 200, 400, 800, 1000, 1600, 3200 samples/second
// Not every sample amount is possible with every width; check out our hookup
// guide for more information.
uint16_t samplrate = I2C_MAX32664_SAMPLE_RATE;
uint16_t pulseWidthVal;
uint16_t sampleVal;

// Takes address, reset pin, and MFIO pin.
Max32664_Hub bioHub(resPin, mfioPin);

bioData body;

// ^^^^^^^^^
// What's this!? This is a type (like "int", "byte", "long") unique to the SparkFun
// Pulse Oximeter and Heart Rate Monitor. Unlike those other types it holds
// specific information on the LED count values of the sensor and ALSO the
// biometric data: heart rate, oxygen levels, and confidence. "bioLedData" is
// actually a specific kind of type, known as a "struct". I chose the name
// "body" but you could use another variable name like "blood", "readings",
// "ledBody" or whatever. Using the variable in the following way gives the
// following data:
// body.irLed      - Infrared LED counts.
// body.redLed     - Red LED counts.
// body.heartrate  - Heartrate
// body.confidence - Confidence in the heartrate value
// body.oxygen     - Blood oxygen level
// body.status     - Has a finger been sensed?

// The familiar setup() function used on Arduino. But you can use directly in app_main()
void setup()
{
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);        // Change this for less or more logging
    esp_log_level_set("max32664", ESP_LOG_VERBOSE); // Change this for less or more logging

    if (CONFIG_FREERTOS_HZ != 1000) {
        ESP_LOGE(TAG, "CONFIG_FREERTOS_HZ must be set to 1000!");
        esp_system_abort("CONFIG_FREERTOS_HZ must be set to 1000!");
    }

    // Initialize I2C bus before anything else
    esp_err_t res = bioHub.i2c_bus_init(I2C_SDA_IO, I2C_SCL_IO);
    ESP_ERROR_CHECK(res);

    res = (esp_err_t)bioHub.begin();
    if (res == ESP_OK) { // Zero errors!
        ESP_LOGI(TAG, "Sensor started!");
    }

    ESP_LOGI(TAG, "Configuring Sensor....");
    ESP_LOGI(TAG, "Keep your finger out of the sensor until you see the first readings.");
    // Configure Sensor and BPM mode , MODE_TWO also available
    uint8_t error = bioHub.configSensorBpm(MODE_ONE);
    if (error == 0) { // Zero errors.
        ESP_LOGI(TAG, "Sensor configured.");
    } else {
        ESP_LOGE(TAG, "Error configuring sensor.");
        ESP_LOGE(TAG, "Error: %u", error);
    }

    // Set pulse width.
    error = bioHub.setPulseWidth(width);
    if (error == 0) { // Zero errors.
        ESP_LOGI(TAG, "Pulse Width Set.");
    } else {
        ESP_LOGE(TAG, "Could not set Pulse Width.");
        ESP_LOGE(TAG, "Error: %u", error);
    }

    // Check that the pulse width was set.
    pulseWidthVal = bioHub.readPulseWidth();
    ESP_LOGI(TAG, "Pulse Width: %u", pulseWidthVal);

    // Set sample rate per second. Remember that not every sample rate is
    // available with every pulse width. Check hookup guide for more information.
    error = bioHub.setSampleRate(samplrate);
    if (error == 0) { // Zero errors.
        ESP_LOGI(TAG, "Sample Rate Set to %u.", samplrate);
    } else {
        ESP_LOGE(TAG, "Could not set Sample Rate!");
        ESP_LOGE(TAG, "Error: %u", error);
    }

    // bioHub.set_report_period(100);

    // Check sample rate.
    sampleVal = bioHub.readSampleRate();

    if (samplrate != sampleVal) {
        ESP_LOGE(TAG, "Sample Rate not set correctly!");
        ESP_LOGE(TAG, "Sample Rate retrieved: %u", sampleVal);
    } else {
        ESP_LOGI(TAG, "Sample rate is set to: %u", sampleVal);
    }

    // Data lags a bit behind the sensor, if you're finger is on the sensor when
    // it's being configured this delay will give some time for the data to catch
    // up.
    ESP_LOGI(TAG, "Loading up the buffer with data....");
    vTaskDelay((portTICK_PERIOD_MS * 4000));
}

void loop()
{
    uint8_t samples = bioHub.numSamplesOutFifo();

    // Information from the readSensor function will be saved to our "body"
    // variable.

    // read all samples in fifo and use most recent one
    while (samples) {
        body = bioHub.readSensorBpm();
        samples--;
    }

    ESP_LOGI(TAG, "%u, %u", body.heartRate, body.oxygen);

    vTaskDelay((portTICK_PERIOD_MS * 10));
}

#ifdef __cplusplus
extern "C" {
#endif
/// @brief main function
void app_main(void)
{
    setup();

    for (;;) {
        loop();
    }
}
#ifdef __cplusplus
}
#endif
