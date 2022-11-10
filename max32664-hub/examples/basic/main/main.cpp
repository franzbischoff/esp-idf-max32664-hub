/*
  Original work from:

    SparkFun Electronics
    Date: 8/2019
    Author: Elias Santistevan

    This example displays a more manual method of adjusting the way in which the
    MAX30101 gathers data. Specifically we'll look at how to modify the pulse
    length of the LEDs within the MAX30101 which impacts the number of samples
    that can be gathered, so we'll adjust this value as well. In addition we
    gather additional data from the BioData type: LED samples. This data gives
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

#include "max32664.hpp"

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char TAG[] = "main";

// Takes address, reset pin, and MFIO pin.
static maxim::Max32664Hub BIO_HUB(I2C_MAX32664_RESET_PIN, // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
                                  I2C_MAX32664_MFIO_PIN);

// The familiar setup() function used on Arduino. But you can use directly in app_main()
void setup()
{
    // Possible widths: 69, 118, 215, 411us
    uint16_t width = I2C_MAX32664_PULSE_WIDTH;
    // Possible samples: 50, 100, 200, 400, 800, 1000, 1600, 3200 samples/second
    // Not every sample amount is possible with every width; check out our hookup
    // guide for more information.
    uint16_t sample_rate = I2C_MAX32664_SAMPLE_RATE;
    uint16_t pulse_width_val;
    uint16_t sample_val;

    esp_log_level_set(TAG, ESP_LOG_VERBOSE);        // Change this for less or more logging
    esp_log_level_set("max32664", ESP_LOG_VERBOSE); // Change this for less or more logging

    if (CONFIG_FREERTOS_HZ != 1000) {
        ESP_LOGE(TAG, "CONFIG_FREERTOS_HZ must be set to 1000!");
        esp_system_abort("CONFIG_FREERTOS_HZ must be set to 1000!");
    }

    // Initialize I2C bus before anything else
    esp_err_t res = BIO_HUB.i2c_bus_init(I2C_MASTER_SDA, I2C_MASTER_SCL);
    ESP_ERROR_CHECK(res);

    res = (esp_err_t)BIO_HUB.begin();
    if (res == ESP_OK) { // Zero errors!
        ESP_LOGI(TAG, "Sensor started!");
    }

    ESP_LOGI(TAG, "Configuring Sensor....");
    ESP_LOGI(TAG, "Keep your finger out of the sensor until you see the first readings.");
    // Configure Sensor and BPM mode , MODE_TWO also available
    uint8_t error = BIO_HUB.config_sensor_bpm(MODE_ONE);
    if (error == 0) { // Zero errors.
        ESP_LOGI(TAG, "Sensor configured.");
    } else {
        ESP_LOGE(TAG, "Error configuring sensor.");
        ESP_LOGE(TAG, "Error: %u", error);
    }

    // Set pulse width.
    error = BIO_HUB.set_pulse_width(width);
    if (error == 0) { // Zero errors.
        ESP_LOGI(TAG, "Pulse Width Set.");
    } else {
        ESP_LOGE(TAG, "Could not set Pulse Width.");
        ESP_LOGE(TAG, "Error: %u", error);
    }

    // Check that the pulse width was set.
    pulse_width_val = BIO_HUB.read_pulse_width();
    ESP_LOGI(TAG, "Pulse Width: %u", pulse_width_val);

    // Set sample rate per second. Remember that not every sample rate is
    // available with every pulse width. Check hookup guide for more information.
    error = BIO_HUB.set_sample_rate(sample_rate);
    if (error == 0) { // Zero errors.
        ESP_LOGI(TAG, "Sample Rate Set to %u.", sample_rate);
    } else {
        ESP_LOGE(TAG, "Could not set Sample Rate!");
        ESP_LOGE(TAG, "Error: %u", error);
    }

    // BIO_HUB.set_report_period(100);

    // Check sample rate.
    sample_val = BIO_HUB.read_sample_rate();

    if (sample_rate != sample_val) {
        ESP_LOGE(TAG, "Sample Rate not set correctly!");
        ESP_LOGE(TAG, "Sample Rate retrieved: %u", sample_val);
    } else {
        ESP_LOGI(TAG, "Sample rate is set to: %u", sample_val);
    }

    // Data lags a bit behind the sensor, if you're finger is on the sensor when
    // it's being configured this delay will give some time for the data to catch
    // up.
    ESP_LOGI(TAG, "Loading up the buffer with data....");
    vTaskDelay((portTICK_PERIOD_MS * 4000));
}

void loop()
{
    uint8_t samples = BIO_HUB.num_samples_out_fifo();
    maxim::BioData body;

    // Information from the readSensor function will be saved to our "body"
    // variable.

    // read all samples in fifo and use most recent one
    while (samples) {
        body = BIO_HUB.read_sensor_bpm();
        --samples;
    }

    ESP_LOGI(TAG, "%u, %u", body.heart_rate, body.oxygen);

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
