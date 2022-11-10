/*
  Original work from:

    SparkFun Electronics
    Date: June, 2019
    Author: Elias Santistevan

    This is an Arduino Library written for the MAXIM 32664 Biometric Sensor Hub
    The MAX32664 Biometric Sensor Hub is in actuality a small Cortex M4 microcontroller
    with pre-loaded firmware and algorithms used to interact with the a number of MAXIM
    sensors; specifically the MAX30101 Pulse Oximter and Heart Rate Monitor and
    the KX122 Accelerometer. With that in mind, this library is built to
    communicate with a middle-person and so has a unique method of communication
    (family, index, and write bytes) that is more simplistic than writing and reading to
    registers, but includes a larger set of definable values.

    License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

    Feel like supporting our work? Buy a board from SparkFun!

  Adapted as ESP-IDF component by:

    Francisco Bischoff
    Date: 2022-11-02

    This code was mainly rewritten to use directly the i2c driver from ESP-IDF.
    The original code was using the Wire library from Arduino.
    Since the time is short, I transcrypted much of the Wire code, so this can
    improved by using the i2c functions even more directly.
    The Wire library uses semaphores everywhere, and I don't know if this is
    really needed here, since the i2c driver may be using semaphores itself.
    These code chunks are disabled by default, but can be enabled by defining
    the macro CONFIG_ENABLE_HAL_LOCKS in the component config.
*/

#include "max32664.hpp"
#include "esp_log.h"

namespace maxim {

static const char TAG[] = "max32664";

Max32664Hub::Max32664Hub(gpio_num_t reset_pin, gpio_num_t mfio_pin, uint8_t address)
    : _reset_pin(reset_pin), _mfio_pin(mfio_pin), _address(address), _user_selected_mode(MODE_ONE)
{
    if (_mfio_pin >= GPIO_NUM_0) {
        gpio_set_direction(_mfio_pin, GPIO_MODE_OUTPUT);
    }

    if (_reset_pin >= GPIO_NUM_0) {
        gpio_set_direction(_reset_pin, GPIO_MODE_OUTPUT); // Set these pins as output
    }
}

Max32664Hub::~Max32664Hub()
{
#if defined(CONFIG_ENABLE_HAL_LOCKS)
    if (_lock != nullptr) {
        // acquire lock
        if (xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "could not acquire lock");
            return;
        }
#endif
        if (_i2c_started) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_driver_delete(I2C_NUM_0));
        }
        _free_wire_buffer();
#if defined(CONFIG_ENABLE_HAL_LOCKS)
        // release lock
        xSemaphoreGive(_lock);
    }
#endif

#if defined(CONFIG_ENABLE_HAL_LOCKS)
    if (_lock != nullptr) {
        vSemaphoreDelete(_lock);
    }
#endif
}

/**
 * @brief i2c master initialization
 */
esp_err_t Max32664Hub::i2c_bus_init(gpio_num_t sda, gpio_num_t scl)
{
    //  A call to i2c_bus_init should occur in sketch
    //  to avoid multiple begins with other sketches.

    esp_err_t res = ESP_OK;

    if (sda == GPIO_NUM_NC) {
        sda = (gpio_num_t)I2C_MASTER_SDA;
    }

    if (scl == GPIO_NUM_NC) {
        scl = (gpio_num_t)I2C_MASTER_SCL;
    }

#if defined(CONFIG_ENABLE_HAL_LOCKS)
    if (_lock == nullptr) {
        _lock = xSemaphoreCreateMutex();
        if (_lock == nullptr) {
            ESP_LOGE(TAG, "xSemaphoreCreateMutex failed");
            return ESP_FAIL;
        }
    }

    // acquire lock
    if (xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "could not acquire lock");
        return ESP_FAIL;
    }
#endif

    if (_i2c_started) {
        ESP_LOGW(TAG, "Bus already started in Master Mode.");
        return ESP_OK;
    }

    if (!_allocate_wire_buffer()) {
        ESP_LOGE(TAG, "_allocateWireBuffer failed");
        return ESP_FAIL;
    }

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (int)sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (int)scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    res = i2c_param_config(I2C_NUM_0, &conf);

    if (res != ESP_OK) {
        return res;
    }

    res = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    if (res == ESP_OK) {
        _i2c_started = true;
    }

    if (!_i2c_started) {
        _free_wire_buffer();
    }
#if defined(CONFIG_ENABLE_HAL_LOCKS)
    // release lock
    xSemaphoreGive(_lock);
#endif

    return res;
}

bool Max32664Hub::_allocate_wire_buffer()
{
    // or both buffer can be allocated or none will be
    if (_rx_buffer == nullptr) {
        _rx_buffer = static_cast<uint8_t *>(malloc(_buffer_size));
        if (_rx_buffer == nullptr) {
            ESP_LOGE(TAG, "Can't allocate memory for I2C_%d rxBuffer", I2C_NUM_0);
            return false;
        }
    }
    if (_tx_buffer == nullptr) {
        _tx_buffer = static_cast<uint8_t *>(malloc(_buffer_size));
        if (_tx_buffer == nullptr) {
            ESP_LOGE(TAG, "Can't allocate memory for I2C_%d txBuffer", I2C_NUM_0);
            _free_wire_buffer(); // free rxBuffer for safety!
            return false;
        }
    }
    // in case both were allocated before, they must have the same size. All good.
    return true;
}

void Max32664Hub::_free_wire_buffer()
{
    if (_rx_buffer != nullptr) {
        free(_rx_buffer);
        _rx_buffer = nullptr;
    }
    if (_tx_buffer != nullptr) {
        free(_tx_buffer);
        _tx_buffer = nullptr;
    }
}

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function initializes the sensor. To place the MAX32664 into
// application mode, the MFIO pin must be pulled High while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be
// in application mode and will return two bytes, the first 0x00 is a
// successful communcation byte, followed by 0x00 which is the byte indicating
// which mode the IC is in.
uint8_t Max32664Hub::begin(gpio_num_t reset_pin, gpio_num_t mfio_pin)
{
    if (reset_pin >= GPIO_NUM_0) {
        _reset_pin = reset_pin;
        gpio_set_direction(_reset_pin, GPIO_MODE_OUTPUT); // Set the pin as output
    }

    if (mfio_pin >= GPIO_NUM_0) {
        _mfio_pin = mfio_pin;
        gpio_set_direction(_mfio_pin, GPIO_MODE_OUTPUT); // Set the pin as output
    }

    if ((_reset_pin == GPIO_NUM_NC) || (_mfio_pin == GPIO_NUM_NC)) { // Bail if the pins have still not been defined
        return ERR_UNKNOWN;                                          // Return ERR_UNKNOWN
    }

    gpio_set_level(_reset_pin, GPIO_HIGH);
    vTaskDelay(portTICK_PERIOD_MS * 30);

    gpio_set_level(_mfio_pin, GPIO_HIGH);
    gpio_set_level(_reset_pin, GPIO_LOW);
    vTaskDelay(portTICK_PERIOD_MS * 30);
    gpio_set_level(_reset_pin, GPIO_HIGH);
    vTaskDelay(portTICK_PERIOD_MS * 1000);
    gpio_set_direction(_mfio_pin, GPIO_MODE_INPUT);  // To be used as an interrupt later
    gpio_set_pull_mode(_mfio_pin, GPIO_PULLUP_ONLY); // To be used as an interrupt later

    uint8_t const response_byte = _read_byte(READ_DEVICE_MODE, 0x00); // 0x00 only possible Index Byte.

    return response_byte;
}

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function puts the MAX32664 into bootloader mode. To place the MAX32664 into
// bootloader mode, the MFIO pin must be pulled LOW while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be
// in bootloader mode and will return two bytes, the first 0x00 is a
// successful communcation byte, followed by 0x08 which is the byte indicating
// that the board is in bootloader mode.
uint8_t Max32664Hub::begin_bootloader(gpio_num_t reset_pin, gpio_num_t mfio_pin)
{
    if (reset_pin >= GPIO_NUM_0) {
        _reset_pin = reset_pin;
        gpio_set_direction(_reset_pin, GPIO_MODE_OUTPUT); // Set the pin as output
    }

    if (mfio_pin >= GPIO_NUM_0) {
        _mfio_pin = mfio_pin;
        gpio_set_direction(_mfio_pin, GPIO_MODE_OUTPUT); // Set the pin as output
    }

    if ((_reset_pin == GPIO_NUM_NC) || (_mfio_pin == GPIO_NUM_NC)) { // Bail if the pins have still not been defined
        return ERR_UNKNOWN;                                          // Return ERR_UNKNOWN
    }

    gpio_set_level(_mfio_pin, GPIO_LOW);
    gpio_set_level(_reset_pin, GPIO_LOW);
    vTaskDelay(portTICK_PERIOD_MS * 10);
    gpio_set_level(_reset_pin, GPIO_HIGH);
    vTaskDelay(portTICK_PERIOD_MS * 50); // Bootloader mode is enabled when this ends.
    gpio_set_direction(_reset_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(_mfio_pin, GPIO_MODE_OUTPUT);

    // Let's check to see if the device made it into bootloader mode.
    uint8_t const response_byte = _read_byte(READ_DEVICE_MODE, 0x00); // 0x00 only possible Index Byte
    return response_byte;
}

// Family Byte: HUB_STATUS (0x00), Index Byte: 0x00, No Write Byte.
// The following function checks the status of the FIFO.
uint8_t Max32664Hub::read_sensor_hub_status()
{
#if defined(READBYTE_FAST)
    uint8_t const status = _read_byte_fast(0x00, 0x00); // Just family and index byte.
#else
    uint8_t const status = _read_byte(0x00, 0x00); // Just family and index byte.
#endif

    return status; // Will return 0x00
}

// This function sets very basic settings to get sensor and biometric data.
// The biometric data includes data about heartrate, the confidence
// level, SpO2 levels, and whether the sensor has detected a finger or not.
uint8_t Max32664Hub::config_bpm(uint8_t mode)
{
    uint8_t status_chauf = 0;
    if (mode == MODE_ONE || mode == MODE_TWO) {
    } else {
        return INCORR_PARAM;
    }

    status_chauf = set_output_mode(ALGO_DATA); // Just the data
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    status_chauf = set_fifo_threshold(0x01); // One sample before interrupt is fired.
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    status_chauf = agc_algo_control(ENABLE); // One sample before interrupt is fired.
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    status_chauf = max30101_control(ENABLE);
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    status_chauf = maxim_fast_algo_control(mode);
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    _user_selected_mode = mode;
    _sample_rate = read_algo_samples();

    vTaskDelay(portTICK_PERIOD_MS * 1000);
    return SUCCESS;
}

// This function sets very basic settings to get LED count values from the MAX30101.
// Sensor data includes 24 bit LED values for the three LED channels: Red, IR,
// and Green.
uint8_t Max32664Hub::config_sensor()
{
    uint8_t status_chauf; // Our status chauffeur

    status_chauf = set_output_mode(SENSOR_DATA); // Just the sensor data (LED)
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    status_chauf = set_fifo_threshold(0x01); // One sample before interrupt is fired to the MAX32664
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    status_chauf = max30101_control(ENABLE); // Enable Sensor.
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    status_chauf = maxim_fast_algo_control(MODE_ONE); // Enable algorithm
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    vTaskDelay(portTICK_PERIOD_MS * 1000);
    return SUCCESS;
}

// This function sets very basic settings to get sensor and biometric data.
// Sensor data includes 24 bit LED values for the two LED channels: Red and IR.
// The biometric data includes data about heartrate, the confidence
// level, SpO2 levels, and whether the sensor has detected a finger or not.
// Of note, the number of samples is set to one.
uint8_t Max32664Hub::config_sensor_bpm(uint8_t mode)
{
    uint8_t status_chauf; // Our status chauffeur
    if (mode == MODE_ONE || mode == MODE_TWO) {
    } else {
        return INCORR_PARAM;
    }

    status_chauf = set_output_mode(SENSOR_AND_ALGORITHM); // Data and sensor data
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    status_chauf = set_fifo_threshold(0x01); // One sample before interrupt is fired to the MAX32664
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    status_chauf = max30101_control(ENABLE); // Enable Sensor.
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    status_chauf = maxim_fast_algo_control(mode); // Enable algorithm
    if (status_chauf != SUCCESS) {
        return status_chauf;
    }

    _user_selected_mode = mode;
    _sample_rate = read_algo_samples();

    vTaskDelay(portTICK_PERIOD_MS * 1000);
    return SUCCESS;
}

// This function takes the 8 bytes from the FIFO buffer related to the wrist
// heart rate algortihm: heart rate (uint16_t), confidence (uint8_t) , SpO2 (uint16_t),
// and the finger detected status (uint8_t). Note that the the algorithm is stated as
// "wrist" though the sensor only works with the finger. The data is loaded
// into the whrmFifo and returned.
BioData Max32664Hub::read_bpm()
{
    BioData lib_bpm;
    uint8_t status_chauf; // The status chauffeur captures return values.

    status_chauf = read_sensor_hub_status();

    if (status_chauf == 1) { // Communication Error
        lib_bpm.heart_rate = 0;
        lib_bpm.confidence = 0;
        lib_bpm.oxygen = 0;
        return lib_bpm;
    }

    num_samples_out_fifo();

    if (_user_selected_mode == MODE_ONE) {
        _read_fill_array(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE, bpm_arr);

        // Heart Rate formatting
        lib_bpm.heart_rate = (uint16_t(bpm_arr[0]) << 8);
        lib_bpm.heart_rate |= (bpm_arr[1]);
        lib_bpm.heart_rate /= 10;

        // Confidence formatting
        lib_bpm.confidence = bpm_arr[2];

        // Blood oxygen level formatting
        lib_bpm.oxygen = uint16_t(bpm_arr[3]) << 8;
        lib_bpm.oxygen |= bpm_arr[4];
        lib_bpm.oxygen /= 10;

        //"Machine State" - has a finger been detected?
        lib_bpm.status = bpm_arr[5];

        return lib_bpm;
    }

    else if (_user_selected_mode == MODE_TWO) {
        _read_fill_array(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA, bpm_arr_two);

        // Heart Rate formatting
        lib_bpm.heart_rate = (uint16_t(bpm_arr_two[0]) << 8);
        lib_bpm.heart_rate |= (bpm_arr_two[1]);
        lib_bpm.heart_rate /= 10;

        // Confidence formatting
        lib_bpm.confidence = bpm_arr_two[2];

        // Blood oxygen level formatting
        lib_bpm.oxygen = uint16_t(bpm_arr_two[3]) << 8;
        lib_bpm.oxygen |= bpm_arr_two[4];
        lib_bpm.oxygen /= 10;

        //"Machine State" - has a finger been detected?
        lib_bpm.status = bpm_arr_two[5];

        // Sp02 r Value formatting
        uint16_t temp_val = uint16_t(bpm_arr_two[6]) << 8;
        temp_val |= bpm_arr_two[7];
        lib_bpm.r_value = (float)temp_val;
        lib_bpm.r_value /= 10.0;

        // Extended Machine State formatting
        lib_bpm.ext_status = (int8_t)bpm_arr_two[8];

        // There are two additional bytes of data that were requested but that
        // have not been implemented in firmware 10.1 so will not be saved to
        // user's data.
        return lib_bpm;
    }

    else {
        lib_bpm.heart_rate = 0;
        lib_bpm.confidence = 0;
        lib_bpm.oxygen = 0;
        return lib_bpm;
    }
}

// This function takes 9 bytes of LED values from the MAX30101 associated with
// the RED, IR, and GREEN LEDs. In addition it gets the 8 bytes from the FIFO buffer
// related to the wrist heart rate algortihm: heart rate (uint16_t), confidence (uint8_t),
// SpO2 (uint16_t), and the finger detected status (uint8_t). Note that the the algorithm
// is stated as "wrist" though the sensor only works with the finger. The data is loaded
// into the whrmFifo and returned.
BioData Max32664Hub::read_sensor()
{
    BioData lib_led_fifo;
    _read_fill_array(READ_DATA_OUTPUT, READ_DATA, MAX30101_LED_ARRAY, sen_arr);

    // Value of LED one....
    lib_led_fifo.ir_led = uint32_t(sen_arr[0]) << 16;
    lib_led_fifo.ir_led |= uint32_t(sen_arr[1]) << 8;
    lib_led_fifo.ir_led |= sen_arr[2];

    // Value of LED two...
    lib_led_fifo.red_led = uint32_t(sen_arr[3]) << 16;
    lib_led_fifo.red_led |= uint32_t(sen_arr[4]) << 8;
    lib_led_fifo.red_led |= sen_arr[5];

    // Value of LED three...
    lib_led_fifo.green_led = uint32_t(sen_arr[6]) << 16;
    lib_led_fifo.green_led |= uint32_t(sen_arr[7]) << 8;
    lib_led_fifo.green_led |= sen_arr[8];

    return lib_led_fifo;
}

// This function takes the information of both the LED value and the biometric
// data from the MAX32664's FIFO. In essence it combines the two functions
// above into a single function call.
BioData Max32664Hub::read_sensor_bpm()
{
    BioData lib_led_bpm;

    if (_user_selected_mode == MODE_ONE) {
        _read_fill_array(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAX30101_LED_ARRAY, bpm_sen_arr);

        // Value of LED one....
        lib_led_bpm.ir_led = uint32_t(bpm_sen_arr[0]) << 16;
        lib_led_bpm.ir_led |= uint32_t(bpm_sen_arr[1]) << 8;
        lib_led_bpm.ir_led |= bpm_sen_arr[2];

        // Value of LED two...
        lib_led_bpm.red_led = uint32_t(bpm_sen_arr[3]) << 16;
        lib_led_bpm.red_led |= uint32_t(bpm_sen_arr[4]) << 8;
        lib_led_bpm.red_led |= bpm_sen_arr[5];

        // Value of LED three...
        lib_led_bpm.green_led = uint32_t(bpm_sen_arr[6]) << 16;
        lib_led_bpm.green_led |= uint32_t(bpm_sen_arr[7]) << 8;
        lib_led_bpm.green_led |= bpm_sen_arr[8];

        // -- What happened here? -- There are two uint32_t values that are given by
        // the sensor for LEDs that do not exists on the MAX30101. So we have to
        // request those empty values because they occupy the buffer:
        // bpmSenArr[6-11].

        // Heart rate formatting
        lib_led_bpm.heart_rate = (uint16_t(bpm_sen_arr[12]) << 8);
        lib_led_bpm.heart_rate |= (bpm_sen_arr[13]);
        lib_led_bpm.heart_rate /= 10;

        // Confidence formatting
        lib_led_bpm.confidence = bpm_sen_arr[14];

        // Blood oxygen level formatting
        lib_led_bpm.oxygen = uint16_t(bpm_sen_arr[15]) << 8;
        lib_led_bpm.oxygen |= bpm_sen_arr[16];
        lib_led_bpm.oxygen /= 10;

        //"Machine State" - has a finger been detected?
        lib_led_bpm.status = bpm_sen_arr[17];
        return lib_led_bpm;
    }

    else if (_user_selected_mode == MODE_TWO) {
        _read_fill_array(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAX30101_LED_ARRAY + MAXFAST_EXTENDED_DATA,
                         bpm_sen_arr_two);

        // Value of LED one....
        lib_led_bpm.ir_led = uint32_t(bpm_sen_arr_two[0]) << 16;
        lib_led_bpm.ir_led |= uint32_t(bpm_sen_arr_two[1]) << 8;
        lib_led_bpm.ir_led |= bpm_sen_arr_two[2];

        // Value of LED two...
        lib_led_bpm.red_led = uint32_t(bpm_sen_arr_two[3]) << 16;
        lib_led_bpm.red_led |= uint32_t(bpm_sen_arr_two[4]) << 8;
        lib_led_bpm.red_led |= bpm_sen_arr_two[5];

        // -- What happened here? -- There are two uint32_t values that are given by
        // the sensor for LEDs that do not exists on the MAX30101. So we have to
        // request those empty values because they occupy the buffer:
        // bpmSenArrTwo[6-11].

        // Heart rate formatting
        lib_led_bpm.heart_rate = (uint16_t(bpm_sen_arr_two[12]) << 8);
        lib_led_bpm.heart_rate |= (bpm_sen_arr_two[13]);
        lib_led_bpm.heart_rate /= 10;

        // Confidence formatting
        lib_led_bpm.confidence = bpm_sen_arr_two[14];

        // Blood oxygen level formatting
        lib_led_bpm.oxygen = uint16_t(bpm_sen_arr_two[15]) << 8;
        lib_led_bpm.oxygen |= bpm_sen_arr_two[16];
        lib_led_bpm.oxygen /= 10;

        //"Machine State" - has a finger been detected?
        lib_led_bpm.status = bpm_sen_arr_two[17];

        // Sp02 r Value formatting
        uint16_t temp_val = uint16_t(bpm_sen_arr_two[18]) << 8;
        temp_val |= bpm_sen_arr_two[19];
        lib_led_bpm.r_value = (float)temp_val;
        lib_led_bpm.r_value /= 10.0;

        // Extended Machine State formatting
        lib_led_bpm.ext_status = (int8_t)bpm_sen_arr_two[20];

        // There are two additional bytes of data that were requested but that
        // have not been implemented in firmware 10.1 so will not be saved to
        // user's data.
        //
        return lib_led_bpm;

    }

    else {
        lib_led_bpm.ir_led = 0;
        lib_led_bpm.red_led = 0;
        lib_led_bpm.heart_rate = 0;
        lib_led_bpm.confidence = 0;
        lib_led_bpm.oxygen = 0;
        lib_led_bpm.status = 0;
        lib_led_bpm.r_value = 0.0;
        lib_led_bpm.ext_status = 0;
        return lib_led_bpm;
    }
}

// This function modifies the pulse width of the MAX30101 LEDs. All of the LEDs
// are modified to the same width. This will affect the number of samples that
// can be collected and will also affect the ADC resolution.
// Default: 69us - 15 resolution - 50 samples per second.
// Register: 0x0A, bits [1:0]
// Width(us) - Resolution -  Sample Rate
//  69us     -    15      -   <= 3200 (fastest - least resolution)
//  118us    -    16      -   <= 1600
//  215us    -    17      -   <= 1600
//  411us    -    18      -   <= 1000 (slowest - GPIO_HIGHest resolution)
uint8_t Max32664Hub::set_pulse_width(uint16_t width)
{
    uint8_t bits;
    uint8_t reg_val;

    // Make sure the correct pulse width is selected.
    if (width == 69) {
        bits = 0;
    } else if (width == 118) {
        bits = 1;
    } else if (width == 215) {
        bits = 2;
    } else if (width == 411) {
        bits = 3;
    } else {
        return INCORR_PARAM;
    }

    // Get current register value so that nothing is overwritten.
    reg_val = read_register_max30101(CONFIGURATION_REGISTER);
    reg_val &= PULSE_MASK;                                    // Mask bits to change.
    reg_val |= bits;                                          // Add bits
    write_register_max30101(CONFIGURATION_REGISTER, reg_val); // Write Register

    return SUCCESS;
}

// This function reads the CONFIGURATION_REGISTER (0x0A), bits [1:0] from the
// MAX30101 Sensor. It returns one of the four settings in microseconds.
uint16_t Max32664Hub::read_pulse_width()
{
    uint8_t reg_val;

    reg_val = read_register_max30101(CONFIGURATION_REGISTER);
    reg_val &= READ_PULSE_MASK;

    if (reg_val == 0) {
        return 69;
    } else if (reg_val == 1) {
        return 118;
    } else if (reg_val == 2) {
        return 215;
    } else /*if (regVal == 3)*/ {
        return 411;
    } /*else {
      return ERR_UNKNOWN; // this should never happen.
    }*/
}

// This function changes the sample rate of the MAX30101 sensor. The sample
// rate is affected by the set pulse width of the MAX30101 LEDs.
// Default: 69us - 15 resolution - 50 samples per second.
// Register: 0x0A, bits [4:2]
// Width(us) - Resolution -  Sample Rate
//  69us     -    15      -   <= 3200 (fastest - least resolution)
//  118us    -    16      -   <= 1600
//  215us    -    17      -   <= 1600
//  411us    -    18      -   <= 1000 (slowest - GPIO_HIGHest resolution)
uint8_t Max32664Hub::set_sample_rate(uint16_t samp_rate)
{
    uint8_t bits;
    uint8_t reg_val;

    // Make sure the correct sample rate was picked
    if (samp_rate == 50) {
        bits = 0;
    } else if (samp_rate == 100) {
        bits = 1;
    } else if (samp_rate == 200) {
        bits = 2;
    } else if (samp_rate == 400) {
        bits = 3;
    } else if (samp_rate == 800) {
        bits = 4;
    } else if (samp_rate == 1000) {
        bits = 5;
    } else if (samp_rate == 1600) {
        bits = 6;
    } else if (samp_rate == 3200) {
        bits = 7;
    } else {
        return INCORR_PARAM;
    }

    // Get current register value so that nothing is overwritten.
    reg_val = read_register_max30101(CONFIGURATION_REGISTER);
    reg_val &= SAMP_MASK;                                     // Mask bits to change.
    reg_val |= (bits << 2);                                   // Add bits but shift them first to correct position.
    write_register_max30101(CONFIGURATION_REGISTER, reg_val); // Write Register

    return SUCCESS;
}

// This function reads the CONFIGURATION_REGISTER (0x0A), bits [4:2] from the
// MAX30101 Sensor. It returns one of the 8 possible sample rates.
uint16_t Max32664Hub::read_sample_rate()
{
    uint8_t reg_val;

    reg_val = read_register_max30101(CONFIGURATION_REGISTER);
    reg_val &= READ_SAMP_MASK;
    reg_val = (reg_val >> 2);

    if (reg_val == 0) {
        return 50;
    } else if (reg_val == 1) {
        return 100;
    } else if (reg_val == 2) {
        return 200;
    } else if (reg_val == 3) {
        return 400;
    } else if (reg_val == 4) {
        return 800;
    } else if (reg_val == 5) {
        return 1000;
    } else if (reg_val == 6) {
        return 1600;
    } else /*if (regVal == 7)*/ {
        return 3200;
    } /*else {
      return ERR_UNKNOWN; // this should never happen.
    }*/
}

// MAX30101 Register: CONFIGURATION_REGISTER (0x0A), bits [6:5]
// This functions sets the dynamic range of the MAX30101's ADC. The function
// accepts the GPIO_HIGHer range as a parameter.
// Default Range: 7.81pA - 2048nA
// Possible Ranges:
// 7.81pA  - 2048nA
// 15.63pA - 4096nA
// 32.25pA - 8192nA
// 62.5pA  - 16384nA
uint8_t Max32664Hub::set_adc_range(uint16_t adc_val)
{
    uint8_t reg_val;
    uint8_t bits;

    if (adc_val <= 2048) {
        bits = 0;
    } else if (adc_val <= 4096) {
        bits = 1;
    } else if (adc_val <= 8192) {
        bits = 2;
    } else if (adc_val <= 16384) {
        bits = 3;
    } else {
        return INCORR_PARAM;
    }

    reg_val = read_register_max30101(CONFIGURATION_REGISTER);
    reg_val &= ADC_MASK;
    reg_val |= bits << 5;

    write_register_max30101(CONFIGURATION_REGISTER, reg_val);

    return SUCCESS;
}

// MAX30101 Register: CONFIGURATION_REGISTER (0x0A), bits [6:5]
// This function returns the set ADC range of the MAX30101 sensor.
uint16_t Max32664Hub::read_adc_range()
{
    uint8_t reg_val;
    reg_val = read_register_max30101(CONFIGURATION_REGISTER);
    reg_val &= READ_ADC_MASK;
    reg_val = (reg_val >> 5); // Shift our bits to the front of the line.

    if (reg_val == 0) {
        return 2048;
    } else if (reg_val == 1) {
        return 4096;
    } else if (reg_val == 2) {
        return 8192;
    } else /* if (regVal == 3) */ {
        return 16384;
    } /*else {
      return ERR_UNKNOWN; // this should never happen.
    }*/
}

// Family Byte: SET_DEVICE_MODE (0x01), Index Byte: 0x00, Write Byte: 0x00; 0x02; 0x08
// The following function is an alternate way to set the mode of the of
// MAX32664. It can take three parameters: Enter and Exit Bootloader Mode, as
// well as reset.
// Page 14
uint8_t Max32664Hub::set_operating_mode(uint8_t selection)
{
    // Must be one of the three....
    if (selection == EXIT_BOOTLOADER || selection == RESET || selection == ENTER_BOOTLOADER) {
    } else {
        return INCORR_PARAM;
    }

    uint8_t const status_byte = _write_byte(SET_DEVICE_MODE, 0x00, selection);
    if (status_byte != SUCCESS) {
        return status_byte;
    }

    // Here we'll check if the board made it into Bootloader mode...
    // responses are 0x00: Application operating mode. 0x08: Bootloader operating mode
    uint8_t const response_byte = _read_byte(READ_DEVICE_MODE, 0x00);
    return response_byte; // This is in fact the status byte, need second returned byte - bootloader mode
}

// Family Byte: IDENTITY (0x01), Index Byte: READ_MCU_TYPE, Write Byte: NONE
// The following function returns a byte that signifies the microcontoller that
// is in communcation with your host microcontroller. Returns 0x00 for the
// MAX32625 and 0x01 for the MAX32660/MAX32664.
uint8_t Max32664Hub::get_mcu_type()
{
    uint8_t const return_byte = _read_byte(IDENTITY, READ_MCU_TYPE, NO_WRITE);
    if (return_byte != SUCCESS) {
        return ERR_UNKNOWN;
    } else {
        return return_byte;
    }
}

// Family Byte: BOOTLOADER_INFO (0x80), Index Byte: BOOTLOADER_VERS (0x00)
// This function checks the version number of the bootloader on the chip and
// returns a four bytes: Major version Byte, Minor version Byte, Space Byte,
// and the Revision Byte.
int32_t Max32664Hub::get_bootloader_inf()
{
    const size_t size_of_rev = 4;
    int32_t rev_num[size_of_rev] = {};
    uint8_t const status = _read_multiple_bytes(BOOTLOADER_INFO, BOOTLOADER_VERS, 0x00, 4, rev_num);

    if (status == 0) {
        return ERR_UNKNOWN;
    } else {
        int32_t boot_vers = 0;
        boot_vers |= (int32_t(rev_num[1]) << 16);
        boot_vers |= (int32_t(rev_num[2]) << 8);
        boot_vers |= rev_num[3];
        return boot_vers;
    }
}

// Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30101 (0x03), Write
// Byte: senSwitch  (parameter - 0x00 or 0x01).
// This function enables the MAX30101.
uint8_t Max32664Hub::max30101_control(uint8_t sen_switch)
{
    if (sen_switch == 0 || sen_switch == 1) {
    } else {
        return INCORR_PARAM;
    }

    // Check that communication was successful, not that the sensor is enabled.
    uint8_t const status_byte = _enable_write(ENABLE_SENSOR, ENABLE_MAX30101, sen_switch);
    if (status_byte != SUCCESS) {
        return status_byte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: READ_SENSOR_MODE (0x45), Index Byte: READ_ENABLE_MAX30101 (0x03)
// This function checks if the MAX30101 is enabled or not.
uint8_t Max32664Hub::read_max30101_state()
{
    uint8_t const state = _read_byte(READ_SENSOR_MODE, READ_ENABLE_MAX30101);
    return state;
}

// Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_ACCELEROMETER (0x04), Write
// Byte: accepts (parameter - 0x00 or 0x01).
// This function enables the Accelerometer.
uint8_t Max32664Hub::accel_control(uint8_t accel_switch)
{
    if (accel_switch != 0 && accel_switch != 1) {
    } else {
        return INCORR_PARAM;
    }

    // Check that communication was successful, not that the sensor is enabled.
    uint8_t const status_byte = _enable_write(ENABLE_SENSOR, ENABLE_ACCELEROMETER, accel_switch);
    if (status_byte != SUCCESS) {
        return status_byte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: OUTPUT_MODE (0x10), Index Byte: SET_FORMAT (0x00),
// Write Byte : outputType (Parameter values in OUTPUT_MODE_WRITE_BYTE)
uint8_t Max32664Hub::set_output_mode(uint8_t output_type)
{
    if (output_type > SENSOR_ALGO_COUNTER) { // Bytes between 0x00 and 0x07
        return INCORR_PARAM;
    }

    // Check that communication was successful, not that the IC is outputting
    // correct format.
    uint8_t const status_byte = _write_byte(OUTPUT_MODE, SET_FORMAT, output_type);
    if (status_byte != SUCCESS) {
        return status_byte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: OUTPUT_MODE(0x10), Index Byte: WRITE_SET_THRESHOLD (0x01), Write byte: intThres
// (parameter - value betwen 0 and 0xFF).
// This function changes the threshold for the FIFO interrupt bit/pin. The
// interrupt pin is the MFIO pin which is set to INPUT after IC initialization
// (begin).
uint8_t Max32664Hub::set_fifo_threshold(uint8_t int_thresh)
{
    // Checks that there was succesful communcation, not that the threshold was
    // set correctly.
    uint8_t const status_byte = _write_byte(OUTPUT_MODE, WRITE_SET_THRESHOLD, int_thresh);
    if (status_byte != SUCCESS) {
        return status_byte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: NUM_SAMPLES (0x00), Write
// Byte: NONE
// This function returns the number of samples available in the FIFO.
uint8_t Max32664Hub::num_samples_out_fifo()
{
#if defined(READBYTE_FAST)
    uint8_t const samp_avail = _read_byte_fast(READ_DATA_OUTPUT, NUM_SAMPLES);
#else
    uint8_t const sampAvail = _read_byte(READ_DATA_OUTPUT, NUM_SAMPLES);
#endif

    return samp_avail;
}

// Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_DATA (0x01), Write
// Byte: NONE
// This function returns the data in the FIFO.
uint8_t *Max32664Hub::get_data_out_fifo(uint8_t data[])
{
    uint8_t const samples = num_samples_out_fifo();
    _read_fill_array(READ_DATA_OUTPUT, READ_DATA, samples, data);
    return data;
}

// Family Byte: READ_DATA_INPUT (0x13), Index Byte: FIFO_EXTERNAL_INDEX_BYTE (0x00), Write
// Byte: NONE
// This function adds support for the acceleromter that is NOT included on
// SparkFun's product, The Family Registery of 0x13 and 0x14 is skipped for now.
uint8_t Max32664Hub::num_samples_external_sensor()
{
    uint8_t const samp_avail = _read_byte(READ_DATA_INPUT, SAMPLE_SIZE, WRITE_ACCELEROMETER);
    return samp_avail;
}

// Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX30101 (0x03), Write Bytes:
// Register Address and Register Value
// This function writes the given register value at the given register address
// for the MAX30101 sensor and returns a boolean indicating a successful or
// non-successful write.
void Max32664Hub::write_register_max30101(uint8_t reg_addr, uint8_t reg_val)
{
    _write_byte(WRITE_REGISTER, WRITE_MAX30101, reg_addr, reg_val);
}

// Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_ACCELEROMETER (0x04), Write Bytes:
// Register Address and Register Value
// This function writes the given register value at the given register address
// for the Accelerometer and returns a boolean indicating a successful or
// non-successful write.
void Max32664Hub::write_register_accel(uint8_t reg_addr, uint8_t reg_val)
{
    _write_byte(WRITE_REGISTER, WRITE_ACCELEROMETER, reg_addr, reg_val);
}

// Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30101 (0x03), Write Byte:
// Register Address
// This function reads the given register address for the MAX30101 Sensor and
// returns the values at that register.
uint8_t Max32664Hub::read_register_max30101(uint8_t reg_addr)
{
    uint8_t const reg_cont = _read_byte(READ_REGISTER, READ_MAX30101, reg_addr);
    return reg_cont;
}

// Family Byte: READ_REGISTER (0x41), Index Byte: READ_ACCELEROMETER (0x04), Write Byte:
// Register Address
// This function reads the given register address for the MAX30101 Sensor and
// returns the values at that register.
uint8_t Max32664Hub::read_register_accel(uint8_t reg_addr)
{
    uint8_t const reg_cont = _read_byte(READ_REGISTER, READ_ACCELEROMETER, reg_addr);
    return reg_cont;
}

// Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX30101 (0x03)
// This function retrieves the attributes of the AFE (Analog Front End) of the
// MAX30101 sensor. It returns the number of bytes in a word for the sensor
// and the number of registers available.
SensorAttr Max32664Hub::get_afe_attributes_max30101()
{
    SensorAttr max_attr;
    uint8_t temp_array[2] {};

    _read_fill_array(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_MAX30101, 2, temp_array);

    max_attr.byte_word = temp_array[0];
    max_attr.avail_registers = temp_array[1];

    return max_attr;
}

// Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte:
// RETRIEVE_AFE_ACCELEROMETER (0x04)
// This function retrieves the attributes of the AFE (Analog Front End) of the
// Accelerometer. It returns the number of bytes in a word for the sensor
// and the number of registers available.
SensorAttr Max32664Hub::get_afe_attributes_accelerometer()
{
    SensorAttr max_attr;
    uint8_t temp_array[2] {};

    _read_fill_array(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_ACCELEROMETER, 2, temp_array);

    max_attr.byte_word = temp_array[0];
    max_attr.avail_registers = temp_array[1];

    return max_attr;
}

// Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX30101 (0x03)
// This function returns all registers and register values sequentially of the
// MAX30101 sensor: register zero and register value zero to register n and
// register value n. There are 36 registers in this case.
uint8_t Max32664Hub::dump_register_max30101(uint8_t reg_array[])
{
    uint8_t const num_of_bytes = 36;
    uint8_t const status = _read_fill_array(DUMP_REGISTERS, DUMP_REGISTER_MAX30101, num_of_bytes, reg_array);
    return status;
}

// Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_ACCELEROMETER (0x04)
// This function returns all registers and register values sequentially of the
// Accelerometer: register zero and register value zero to register n and
// register value n.
uint8_t Max32664Hub::dump_register_accelerometer(uint8_t num_reg, uint8_t reg_array[])
{
    uint8_t const status =
        _read_fill_array(DUMP_REGISTERS, DUMP_REGISTER_ACCELEROMETER, num_reg, reg_array); // Fake read amount
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_TARG_PERC (0x00), Write Byte: AGC_GAIN_ID (0x00)
// This function sets the target percentage of the full-scale ADC range that
// the automatic gain control algorithm uses. It takes a paramater of zero to
// 100 percent.
uint8_t Max32664Hub::set_algo_range(uint8_t perc)
{
    if (perc > 100) {
        return INCORR_PARAM;
    }

    // Successful communication or no?
    uint8_t const status_byte = _write_byte(CHANGE_ALGORITHM_CONFIG, SET_TARG_PERC, AGC_GAIN_ID, perc);
    if (status_byte != SUCCESS) {
        return status_byte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_STEP_SIZE (0x00), Write Byte: AGC_STEP_SIZE_ID (0x01)
// This function changes the step size toward the target for the AGC algorithm.
// It takes a paramater of zero to 100 percent.
uint8_t Max32664Hub::set_algo_step_size(uint8_t step)
{
    if (step > 100) {
        return INCORR_PARAM;
    }

    // Successful communication or no?
    uint8_t const status_byte = _write_byte(CHANGE_ALGORITHM_CONFIG, SET_STEP_SIZE, AGC_STEP_SIZE_ID, step);
    if (status_byte != SUCCESS) {
        return status_byte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_SENSITIVITY (0x00), Write Byte: AGC_SENSITIVITY_ID (0x02)
// This function changes the sensitivity of the AGC algorithm.
uint8_t Max32664Hub::set_algo_sensitivity(uint8_t sense)
{
    if (sense > 100) {
        return INCORR_PARAM;
    }

    // Successful communication or no?
    uint8_t const status_byte = _write_byte(CHANGE_ALGORITHM_CONFIG, SET_SENSITIVITY, AGC_SENSITIVITY_ID, sense);
    if (status_byte != SUCCESS) {
        return status_byte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_AVG_SAMPLES (0x00), Write Byte: AGC_NUM_SAMP_ID (0x03)
// This function changes the number of samples that are averaged.
// It takes a paramater of zero to 255.
uint8_t Max32664Hub::set_algo_samples(uint8_t avg)
{
    // Successful communication or no?
    uint8_t const status_byte = _write_byte(CHANGE_ALGORITHM_CONFIG, SET_AVG_SAMPLES, AGC_NUM_SAMP_ID, avg);
    if (status_byte != SUCCESS) {
        return status_byte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// WHRM_CONFIG (0x02), Write Byte: MAXIMFAST_COEF_ID (0x0B)
// This function takes three values that are used as the Sp02 coefficients.
// These three values are multiplied by 100,000;
// default values are in order: 159584, -3465966, and 11268987.
uint8_t Max32664Hub::set_maxim_fast_coef(int32_t coef1, int32_t coef2, int32_t coef3)
{
    const size_t num_coef_vals = 3;
    int32_t coef_arr[num_coef_vals] = {coef1, coef2, coef3};

    uint8_t const status_byte =
        _write_long_bytes(CHANGE_ALGORITHM_CONFIG, WHRM_CONFIG, MAXIMFAST_COEF_ID, coef_arr, num_coef_vals);
    if (status_byte != SUCCESS) {
        return status_byte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_PERCENTAGE (0x00), Write Byte: READ_AGC_PERC_ID (0x00)
// This function reads and returns the currently set target percentage
// of the full-scale ADC range that the Automatic Gain Control algorithm is using.
uint8_t Max32664Hub::read_algo_range()
{
    uint8_t const range = _read_byte(READ_ALGORITHM_CONFIG, READ_AGC_PERCENTAGE, READ_AGC_PERC_ID);
    return range;
}

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_STEP_SIZE (0x00), Write Byte: READ_AGC_STEP_SIZE_ID (0x01)
// This function returns the step size toward the target for the AGC algorithm.
// It returns a value between zero and 100 percent.
uint8_t Max32664Hub::read_algo_step_size()
{
    uint8_t const step_size = _read_byte(READ_ALGORITHM_CONFIG, READ_AGC_STEP_SIZE, READ_AGC_STEP_SIZE_ID);
    return step_size;
}

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_SENSITIVITY_ID (0x00), Write Byte: READ_AGC_SENSITIVITY_ID (0x02)
// This function returns the sensitivity (percentage) of the automatic gain control.
uint8_t Max32664Hub::read_algo_sensitivity()
{
    uint8_t const sensitivity = _read_byte(READ_ALGORITHM_CONFIG, READ_AGC_SENSITIVITY, READ_AGC_SENSITIVITY_ID);
    return sensitivity;
}

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_NUM_SAMPLES (0x00), Write Byte: READ_AGC_NUM_SAMPLES_ID (0x03)
// This function changes the number of samples that are averaged.
// It takes a paramater of zero to 255.
uint8_t Max32664Hub::read_algo_samples()
{
    uint8_t const samples = _read_byte(READ_ALGORITHM_CONFIG, READ_AGC_NUM_SAMPLES, READ_AGC_NUM_SAMPLES_ID);
    return samples;
}

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_MAX_FAST_COEF (0x02), Write Byte: READ_MAX_FAST_COEF_ID (0x0B)
// This function reads the maximum age for the wrist heart rate monitor
// (WHRM) algorithm. It returns three uint32_t integers that are
// multiplied by 100,000.
uint8_t Max32664Hub::read_maxim_fast_coef(int32_t coef_arr[3])
{
    const size_t num_of_reads = 3;
    uint8_t const status =
        _read_multiple_bytes(READ_ALGORITHM_CONFIG, READ_MAX_FAST_COEF, READ_MAX_FAST_COEF_ID, num_of_reads, coef_arr);
    coef_arr[0] = coef_arr[0] * 100000;
    coef_arr[1] = coef_arr[1] * 100000;
    coef_arr[2] = coef_arr[2] * 100000;
    return status;
}

// Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
// ENABLE_AGC_ALGO (0x00)
// This function enables (one) or disables (zero) the automatic gain control algorithm.
uint8_t Max32664Hub::agc_algo_control(uint8_t enable)
{
    if (enable == 0 || enable == 1) {
    } else {
        return INCORR_PARAM;
    }

    uint8_t const status_byte = _enable_write(ENABLE_ALGORITHM, ENABLE_AGC_ALGO, enable);
    if (status_byte != SUCCESS) {
        return status_byte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
// ENABLE_WHRM_ALGO (0x02)
// This function enables (one) or disables (zero) the wrist heart rate monitor
// algorithm.
uint8_t Max32664Hub::maxim_fast_algo_control(uint8_t mode)
{
    if (mode == 0 || mode == 1 || mode == 2) {
    } else {
        return INCORR_PARAM;
    }

    uint8_t const status_byte = _enable_write(ENABLE_ALGORITHM, ENABLE_WHRM_ALGO, mode);
    if (status_byte != SUCCESS) {
        return status_byte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_INIT_VECTOR_BYTES (0x00)
// void Max32664Hub::setInitBytes

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_AUTH_BYTES (0x01)

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_NUM_PAGES (0x02),
// Write Bytes: 0x00 - Number of pages at byte 0x44 from .msbl file.
bool Max32664Hub::set_num_pages(uint8_t total_pages)
{
    uint8_t const status_byte = _write_byte(BOOTLOADER_FLASH, SET_NUM_PAGES, 0x00, total_pages);
    return status_byte;
}

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: ERASE_FLASH (0x03)
// Returns true on successful communication.
bool Max32664Hub::erase_flash()
{
    // This is a unique write in that it does not have a relevant write byte.
    _begin_transmission(_address);
    _write(BOOTLOADER_FLASH);
    _write(ERASE_FLASH);
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _request_from(_address, static_cast<uint8_t>(1));
    uint8_t const status_byte = _read();
    return (status_byte == 0);
}

// Family Byte: BOOTLOADER_INFO (0x81), Index Byte: BOOTLOADER_VERS (0x00)
Version Max32664Hub::read_bootloader_vers()
{
    Version boo_vers; // BOO!
    _begin_transmission(_address);
    _write(BOOTLOADER_INFO);
    _write(BOOTLOADER_VERS);
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _request_from(_address, static_cast<uint8_t>(4));
    uint8_t const status_byte = _read();
    if (status_byte) { // Pass through if SUCCESS (0x00).
        boo_vers.major = 0;
        boo_vers.minor = 0;
        boo_vers.revision = 0;
        return boo_vers;
    }

    boo_vers.major = _read();
    boo_vers.minor = _read();
    boo_vers.revision = _read();

    return boo_vers;
}

// Family Byte: IDENTITY (0xFF), Index Byte: READ_SENSOR_HUB_VERS (0x03)
Version Max32664Hub::read_sensor_hub_version()
{
    Version bio_hub_vers;
    _begin_transmission(_address);
    _write(IDENTITY);
    _write(READ_SENSOR_HUB_VERS);
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _request_from(_address, static_cast<uint8_t>(4));
    uint8_t const status_byte = _read();
    if (status_byte) { // Pass through if SUCCESS (0x00).
        bio_hub_vers.major = 0;
        bio_hub_vers.minor = 0;
        bio_hub_vers.revision = 0;
        return bio_hub_vers;
    }

    bio_hub_vers.major = _read();
    bio_hub_vers.minor = _read();
    bio_hub_vers.revision = _read();

    return bio_hub_vers;
}

// Family Byte: IDENTITY (0xFF), Index Byte: READ_ALGO_VERS (0x07)
Version Max32664Hub::read_algorithm_version()
{
    Version lib_algo_vers;
    _begin_transmission(_address);
    _write(IDENTITY);
    _write(READ_ALGO_VERS);
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _request_from(_address, static_cast<uint8_t>(4));
    uint8_t const status_byte = _read();
    if (status_byte) { // Pass through if SUCCESS (0x00).
        lib_algo_vers.major = 0;
        lib_algo_vers.minor = 0;
        lib_algo_vers.revision = 0;
        return lib_algo_vers;
    }

    lib_algo_vers.major = _read();
    lib_algo_vers.minor = _read();
    lib_algo_vers.revision = _read();

    return lib_algo_vers;
}

// ------------------Function Below for MAX32664 Version D (Blood Pressure) ----

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_MEDICATION (0x00)
uint8_t Max32664Hub::is_patient_bp_medication(uint8_t medication)
{
    if (medication != 0x00 && medication != 0x01) {
        return INCORR_PARAM;
    }

    uint8_t const status = _write_byte(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_MEDICATION, medication);
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_MEDICATION (0x00)
uint8_t Max32664Hub::is_patient_bp_medication()
{
    uint8_t const medication = _read_byte(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_MEDICATION);
    return medication;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: SYSTOLIC_VALUE (0x02)
uint8_t Max32664Hub::write_systolic_vals(uint8_t sys_val1, uint8_t sys_val2, uint8_t sys_val3)
{
    const size_t num_sys_vals = 3;
    uint8_t sys_vals[num_sys_vals] = {sys_val1, sys_val2, sys_val3};
    uint8_t const status = _write_bytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, SYSTOLIC_VALUE, sys_vals, num_sys_vals);

    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: SYSTOLIC_VALUE (0x02)
uint8_t Max32664Hub::read_systolic_vals(uint8_t user_array[])
{
    const size_t num_sys_vals = 3;
    uint8_t const status =
        _read_multiple_bytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, SYSTOLIC_VALUE, num_sys_vals, user_array);

    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: DIASTOLIC_VALUE (0x01)
uint8_t Max32664Hub::write_diastolic_vals(uint8_t dias_val1, uint8_t dias_val2, uint8_t dias_val3)
{
    const size_t num_dias_vals = 3;
    uint8_t dias_vals[num_dias_vals] = {dias_val1, dias_val2, dias_val3};
    uint8_t const status = _write_bytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, DIASTOLIC_VALUE, dias_vals, num_dias_vals);

    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: DIASTOLIC_VALUE (0x01)
uint8_t Max32664Hub::read_diastolic_vals(uint8_t user_array[])
{
    const size_t num_dias_vals = 3;
    uint8_t const status =
        _read_multiple_bytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, DIASTOLIC_VALUE, num_dias_vals, user_array);
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_CALIB_DATA (0x03)
uint8_t Max32664Hub::write_bpt_algo_data(const uint8_t bpt_calib_data[])
{
    const size_t num_calib_vals = 824;
    uint8_t const status =
        _write_bytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_CALIB_DATA, bpt_calib_data, num_calib_vals);
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_CALIB_DATA (0x03)
uint8_t Max32664Hub::read_bpt_algo_data(uint8_t user_array[])
{
    const size_t num_calib_vals = 824;
    uint8_t const status =
        _read_multiple_bytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_CALIB_DATA, num_calib_vals, user_array);
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: PATIENT_RESTING (0x05)
uint8_t Max32664Hub::is_patient_resting(uint8_t resting)
{ //

    if (resting != 0x00 && resting != 0x01) {
        return INCORR_PARAM;
    }

    uint8_t const status = _write_byte(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, PATIENT_RESTING, resting);
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: PATIENT_RESTING (0x05)
uint8_t Max32664Hub::is_patient_resting()
{
    uint8_t const resting = _write_byte(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, PATIENT_RESTING);
    return resting;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_SP02_COEFS (0x06)
uint8_t Max32664Hub::write_spo2_algo_coef(int32_t int_a, int32_t int_b, int32_t int_c)
{
    const size_t num_coef_vals = 3;
    int32_t coef_vals[num_coef_vals] = {int_a, int_b, int_c};
    uint8_t const status =
        _write_long_bytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_SP02_COEFS, coef_vals, num_coef_vals);
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_SP02_COEFS (0x06)
uint8_t Max32664Hub::read_spo2_algo_coef(int32_t user_array[])
{ // Have the user provide their own array here and pass the pointer to it

    const size_t num_of_reads = 3;
    uint8_t const status =
        _read_multiple_bytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_SP02_COEFS, num_of_reads, user_array);
    return status;
}

//-------------------Private Functions-----------------------

void Max32664Hub::_begin_transmission(uint16_t address)
{
#if defined(CONFIG_ENABLE_HAL_LOCKS)
    if (_nonStop && _nonStopTask == xTaskGetCurrentTaskHandle()) {
        ESP_LOGE(TAG,
                 "Unfinished Repeated Start transaction! Expected requestFrom, not beginTransmission! Clearing...");
        // release lock
        xSemaphoreGive(_lock);
    }
    // acquire lock
    if (_lock == nullptr || xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "could not acquire lock");
        return;
    }
#endif
    _non_stop = false;
    _tx_address = address;
    _tx_length = 0;
}

void Max32664Hub::_begin_transmission(int16_t address) { _begin_transmission(static_cast<uint16_t>(address)); }

void Max32664Hub::_begin_transmission(uint8_t address) { _begin_transmission(static_cast<uint16_t>(address)); }

uint8_t Max32664Hub::_end_transmission() { return _end_transmission(true); }

/*
https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
endTransmission() returns:
0: success.
1: data too long to fit in transmit buffer.
2: received NACK on transmit of address.
3: received NACK on transmit of data.
4: other error.
5: timeout
*/
uint8_t Max32664Hub::_end_transmission(bool send_stop)
{
    if (_tx_buffer == nullptr) {
        ESP_LOGE(TAG, "NULL TX buffer pointer");
        return 4;
    }
    esp_err_t err = ESP_OK;
    if (send_stop) {
        err = i2c_master_write_to_device(I2C_NUM_0, _tx_address, _tx_buffer, _tx_length, _time_out_millis);
#if defined(CONFIG_ENABLE_HAL_LOCKS)
        // release lock
        xSemaphoreGive(_lock);
#endif
    } else {
        // mark as non-stop
        _non_stop = true;
#if defined(CONFIG_ENABLE_HAL_LOCKS)
        _nonStopTask = xTaskGetCurrentTaskHandle();
#endif
    }
    switch (err) {
    case ESP_OK:
        return 0;
    case ESP_FAIL:
        return 2;
    case ESP_ERR_TIMEOUT:
        return 5;
    default:
        break;
    }
    return 4;
}

size_t Max32664Hub::_write(uint8_t data)
{
    if (_tx_buffer == nullptr) {
        ESP_LOGE(TAG, "NULL TX buffer pointer");
        return 0;
    }
    if (_tx_length >= _buffer_size) {
        return 0;
    }
    _tx_buffer[_tx_length++] = data;
    return 1;
}

size_t Max32664Hub::_write(const uint8_t *data, size_t quantity)
{
    for (size_t i = 0; i < quantity; ++i) {
        if (_write(data[i]) == 0) {
            return i;
        }
    }
    return quantity;
}

int16_t Max32664Hub::_read()
{
    int16_t value = -1;
    if (_rx_buffer == nullptr) {
        ESP_LOGE(TAG, "NULL RX buffer pointer");
        return value;
    }
    if (_rx_index < _rx_length) {
        value = _rx_buffer[_rx_index++];
    }
    return value;
}

size_t Max32664Hub::_request_from(uint16_t address, size_t size)
{
    if (_rx_buffer == nullptr || _tx_buffer == nullptr) {
        ESP_LOGE(TAG, "NULL buffer pointer");
        return 0;
    }
    esp_err_t err = ESP_OK;
    if (_non_stop
#if defined(CONFIG_ENABLE_HAL_LOCKS)
        && _nonStopTask == xTaskGetCurrentTaskHandle()
#endif
    ) {
        if (address != _tx_address) {
            ESP_LOGE(TAG, "Unfinished Repeated Start transaction! Expected address do not match! %u != %u", address,
                     _tx_address);
            return 0;
        }
        _non_stop = false;
        _rx_index = 0;
        _rx_length = 0;
        err = i2c_master_write_read_device(I2C_NUM_0, address, _tx_buffer, _tx_length, _rx_buffer, size,
                                           _time_out_millis);
        if (err) {
            ESP_LOGE(TAG, "i2c_master_write_read_device returned Error %d", err);
        }
    } else {
#if defined(CONFIG_ENABLE_HAL_LOCKS)
        // acquire lock
        if (_lock == nullptr || xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "could not acquire lock");
            return 0;
        }
#endif
        _rx_index = 0;
        _rx_length = 0;
        err = i2c_master_read_from_device(I2C_NUM_0, address, _rx_buffer, size, _time_out_millis);
        if (err) {
            if (err == ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "i2c_master_read_from_device returned Timeout");
            } else {
                ESP_LOGE(TAG, "i2c_master_read_from_device returned Error %d", err);
            }
        } else {
            _rx_length = size;
        }
    }
#if defined(CONFIG_ENABLE_HAL_LOCKS)
    // release lock
    xSemaphoreGive(_lock);
#endif
    return _rx_length;
}

size_t Max32664Hub::_request_from(uint8_t address, size_t len)
{
    return _request_from(static_cast<uint16_t>(address), static_cast<size_t>(len));
}

uint8_t Max32664Hub::_request_from(uint8_t address, uint8_t len)
{
    return _request_from(static_cast<uint16_t>(address), static_cast<size_t>(len));
}

uint8_t Max32664Hub::_request_from(uint16_t address, uint8_t len)
{
    return _request_from(address, static_cast<size_t>(len));
}

uint8_t Max32664Hub::_request_from(int16_t address, int16_t len)
{
    return _request_from(static_cast<uint16_t>(address), static_cast<size_t>(len));
}

// This function uses the given family, index, and write byte to enable
// the given sensor.
uint8_t Max32664Hub::_enable_write(uint8_t family_byte, uint8_t index_byte, uint8_t enable_byte)
{
    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _write(enable_byte);
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * ENABLE_CMD_DELAY); // delayMicroseconds(ENABLE_CMD_DELAY * 1000U);

    // Status Byte, success or no? 0x00 is a successful transmit
    _request_from(_address, static_cast<uint8_t>(1));
    uint8_t const status_byte = _read();
    return status_byte;
}

// This function uses the given family, index, and write byte to communicate
// with the MAX32664 which in turn communicates with downward sensors. There
// are two steps demonstrated in this function. First a write to the MCU
// indicating what you want to do, a delay, and then a read to confirm positive
// transmission.
uint8_t Max32664Hub::_write_byte(uint8_t family_byte, uint8_t index_byte, uint8_t write_byte)
{
    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _write(write_byte);
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    // Status Byte, success or no? 0x00 is a successful transmit
    _request_from(_address, static_cast<uint8_t>(1));
    uint8_t const status_byte = _read();
    return status_byte;
}

// This function is the same as the function above and uses the given family,
// index, and write byte, but also takes a 16 bit integer as a paramter to communicate
// with the MAX32664 which in turn communicates with downward sensors. There
// are two steps demonstrated in this function. First a write to the MCU
// indicating what you want to do, a delay, and then a read to confirm positive
// transmission.
uint8_t Max32664Hub::_write_byte(uint8_t family_byte, uint8_t index_byte, uint8_t write_byte, uint16_t val)
{
    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _write(write_byte);
    _write((val >> 8)); // MSB
    _write(val);        // LSB
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    // Status Byte, success or no? 0x00 is a successful transmit
    _request_from(_address, static_cast<uint8_t>(1));
    uint8_t const status_byte = _read();
    return status_byte;
}

// This function sends information to the MAX32664 to specifically write values
// to the registers of downward sensors and so also requires a
// register address and register value as parameters. Again there is the write
// of the specific bytes followed by a read to confirm positive transmission.
uint8_t Max32664Hub::_write_byte(uint8_t family_byte, uint8_t index_byte, uint8_t write_byte, uint8_t write_val)
{
    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _write(write_byte);
    _write(write_val);
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    // Status Byte, 0x00 is a successful transmit.
    _request_from(_address, static_cast<uint8_t>(1));
    uint8_t const status_byte = _read();
    return status_byte;
}

// This function sends information to the MAX32664 to specifically write values
// to the registers of downward sensors and so also requires a
// register address and register value as parameters. Again there is the write
// of the specific bytes followed by a read to confirm positive transmission.
uint8_t Max32664Hub::_write_long_bytes(uint8_t family_byte, uint8_t index_byte, uint8_t write_byte,
                                       const int32_t write_val[], const size_t size)
{
    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _write(write_byte);

    for (size_t i = 0; i < size; ++i) {
        _write(write_val[i] >> 24);
        _write(write_val[i] >> 16);
        _write(write_val[i] >> 8);
        _write(write_val[i]);
    }

    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    // Status Byte, 0x00 is a successful transmit.
    _request_from(_address, static_cast<uint8_t>(1));
    uint8_t const status_byte = _read();
    return status_byte;
}

// This function sends information to the MAX32664 to specifically write values
// to the registers of downward sensors and so also requires a
// register address and register value as parameters. Again there is the write
// of the specific bytes followed by a read to confirm positive transmission.
uint8_t Max32664Hub::_write_bytes(uint8_t family_byte, uint8_t index_byte, uint8_t write_byte,
                                  const uint8_t write_val[], const size_t size)
{
    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _write(write_byte);

    for (size_t i = 0; i < size; ++i) {
        _write(write_val[i]);
    }

    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY);

    // Status Byte, 0x00 is a successful transmit.
    _request_from(_address, static_cast<uint8_t>(1));
    uint8_t const status_byte = _read();
    return status_byte;
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte an index byte, and
// then delays 60 microseconds, during which the MAX32664 retrieves the requested
// information. An I-squared-C request is then issued, and the information is read.
uint8_t Max32664Hub::_read_byte(uint8_t family_byte, uint8_t index_byte)
{
    uint8_t return_byte;
    uint8_t status_byte;

    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _request_from(_address, static_cast<uint8_t>(sizeof(return_byte) + sizeof(status_byte)));
    status_byte = _read();
    if (status_byte) {      // SUCCESS (0x00) - how do I know its
        return status_byte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    return_byte = _read();
    return return_byte; // If good then return the actual byte.
}

uint8_t Max32664Hub::_read_byte_fast(uint8_t family_byte, uint8_t index_byte)
{
    uint8_t return_byte;
    uint8_t status_byte;

    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _end_transmission();
    // delayMicroseconds(CMD_DELAY*1000U);

    _request_from(_address, static_cast<uint8_t>(sizeof(return_byte) + sizeof(status_byte)));
    status_byte = _read();
    if (status_byte) {      // SUCCESS (0x00) - how do I know its
        return status_byte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    return_byte = _read();
    return return_byte; // If good then return the actual byte.
}

uint8_t Max32664Hub::_read_byte_fast(uint8_t family_byte, uint8_t index_byte, uint8_t write_byte)
{
    uint8_t return_byte;
    uint8_t status_byte;

    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _write(write_byte);
    _end_transmission();
    // delayMicroseconds(CMD_DELAY*1000U);

    _request_from(_address, static_cast<uint8_t>(sizeof(return_byte) + sizeof(status_byte)));
    status_byte = _read();
    if (status_byte) {      // SUCCESS (0x00) - how do I know its
        return status_byte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    return_byte = _read();
    return return_byte; // If good then return the actual byte.
}

// This function is exactly as the one above except it accepts also receives a
// Write Byte as a parameter. It starts a request by writing the family byte, index byte, and
// write byte to the MAX32664 and then delays 60 microseconds, during which
// the MAX32664 retrieves the requested information. A I-squared-C request is
// then issued, and the information is read.
uint8_t Max32664Hub::_read_byte(uint8_t family_byte, uint8_t index_byte, uint8_t write_byte)
{
    uint8_t return_byte;
    uint8_t status_byte;

    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _write(write_byte);
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _request_from(_address, static_cast<uint8_t>(sizeof(return_byte) + sizeof(status_byte)));
    status_byte = _read();
    if (status_byte) {      // SUCCESS (0x00)
        return status_byte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    return_byte = _read();
    return return_byte; // If good then return the actual byte.
}

// This functions is similar to _read_multiple_bytes() except it doesn't send a write byte
// and all the information is read at once and returned in a uint8_t array by reference.
uint8_t Max32664Hub::_read_fill_array(uint8_t family_byte, uint8_t index_byte, uint8_t num_of_reads, uint8_t array[])
{
    uint8_t status_byte;

    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _end_transmission();

    // vTaskDelay(portTICK_PERIOD_MS * 1);

    _request_from(_address, static_cast<uint8_t>(num_of_reads + sizeof(status_byte)));
    status_byte = _read();
    // if( (statusByte!=0) && (statusByte!=255) ){// SUCCESS (0x00)
    if (status_byte != 0) { // SUCCESS (0x00)
        for (uint8_t i = 0; i < num_of_reads; ++i) {
            array[i] = 0;
        }
        return status_byte;
    }

    for (size_t i = 0; i < num_of_reads; ++i) {
        array[i] = _read();
    }
    return status_byte;
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, an index byte, and
// a write byte and then then delays 60 microseconds, during which the MAX32664
// retrieves the requested information. An I-squared-C request is then issued,
// and the information is read. This differs from the above read commands in
// that it returns a 16 bit integer instead of a single byte.
uint16_t Max32664Hub::_read_int_byte(uint8_t family_byte, uint8_t index_byte, uint8_t write_byte)
{
    uint16_t return_byte;
    uint8_t status_byte;

    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _write(write_byte);
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _request_from(_address, static_cast<uint8_t>(sizeof(return_byte) + sizeof(status_byte)));
    status_byte = _read();
    if (status_byte) {      // Pass through if SUCCESS (0x00).
        return status_byte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    return_byte = (static_cast<uint16_t>(_read()) << 8);
    return_byte |= _read();

    return return_byte;
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, an index byte, and
// a write byte and then then delays 60 microseconds, during which the MAX32664
// retrieves the requested information. An I-squared-C request is then issued,
// and the information is read. This function is very similar to the one above
// except it returns three uint32_t bytes instead of one.
uint8_t Max32664Hub::_read_multiple_bytes(uint8_t family_byte, uint8_t index_byte, uint8_t write_byte,
                                          const size_t num_of_reads, int32_t user_array[])
{
    uint8_t status_byte;

    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _write(write_byte);
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _request_from(_address, static_cast<uint8_t>(sizeof(int32_t) * num_of_reads + sizeof(status_byte)));
    status_byte = _read();
    if (status_byte) { // Pass through if SUCCESS (0x00).
        return status_byte;
    } else {
        for (size_t i = 0; i < (sizeof(int32_t) * num_of_reads); ++i) {
            user_array[i] = _read() << 24;
            user_array[i] |= _read() << 16;
            user_array[i] |= _read() << 8;
            user_array[i] |= _read();
        }
        return status_byte;
    }
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, an index byte, and
// a write byte and then then delays 60 microseconds, during which the MAX32664
// retrieves the requested information. An I-squared-C request is then issued,
// and the information is read. This function is very similar to the one above
// except it returns three uint32_t bytes instead of one.
uint8_t Max32664Hub::_read_multiple_bytes(uint8_t family_byte, uint8_t index_byte, uint8_t write_byte,
                                          const size_t num_of_reads, uint8_t user_array[])
{
    uint8_t status_byte;

    _begin_transmission(_address);
    _write(family_byte);
    _write(index_byte);
    _write(write_byte);
    _end_transmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _request_from(_address, static_cast<uint8_t>(num_of_reads + sizeof(status_byte)));
    status_byte = _read();
    if (status_byte) { // Pass through if SUCCESS (0x00).
        return status_byte;
    } else {
        for (size_t i = 0; i < num_of_reads; ++i) {
            user_array[i] = _read();
        }
        return status_byte;
    }
}

} // namespace maxim
