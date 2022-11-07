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

#include "max32664.h"
#include <esp_log.h>

static const char TAG[] = "max32664";

Max32664_Hub::Max32664_Hub(gpio_num_t resetPin, gpio_num_t mfioPin, uint8_t address)
    : _resetPin(resetPin), _mfioPin(mfioPin), _address(address), _userSelectedMode(MODE_ONE)
{
    if (_mfioPin >= GPIO_NUM_0) {
        gpio_set_direction(_mfioPin, GPIO_MODE_OUTPUT);
    }

    if (_resetPin >= GPIO_NUM_0) {
        gpio_set_direction(_resetPin, GPIO_MODE_OUTPUT); // Set these pins as output
    }
}

Max32664_Hub::~Max32664_Hub()
{
#if defined(CONFIG_ENABLE_HAL_LOCKS)
    if (_lock != nullptr) {
        // acquire lock
        if (xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "could not acquire lock");
            return;
        }
#endif
        if (_i2cStarted) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_driver_delete(I2C_NUM_0));
        }
        _freeWireBuffer();
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
esp_err_t Max32664_Hub::i2c_bus_init(gpio_num_t sda, gpio_num_t scl)
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

    if (_i2cStarted) {
        ESP_LOGW(TAG, "Bus already started in Master Mode.");
        return ESP_OK;
    }

    if (!_allocateWireBuffer()) {
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
        _i2cStarted = true;
    }

    if (!_i2cStarted) {
        _freeWireBuffer();
    }
#if defined(CONFIG_ENABLE_HAL_LOCKS)
    // release lock
    xSemaphoreGive(_lock);
#endif

    return res;
}

bool Max32664_Hub::_allocateWireBuffer()
{
    // or both buffer can be allocated or none will be
    if (_rxBuffer == nullptr) {
        _rxBuffer = static_cast<uint8_t *>(malloc(_bufferSize));
        if (_rxBuffer == nullptr) {
            ESP_LOGE(TAG, "Can't allocate memory for I2C_%d rxBuffer", I2C_NUM_0);
            return false;
        }
    }
    if (_txBuffer == nullptr) {
        _txBuffer = static_cast<uint8_t *>(malloc(_bufferSize));
        if (_txBuffer == nullptr) {
            ESP_LOGE(TAG, "Can't allocate memory for I2C_%d txBuffer", I2C_NUM_0);
            _freeWireBuffer(); // free rxBuffer for safety!
            return false;
        }
    }
    // in case both were allocated before, they must have the same size. All good.
    return true;
}

void Max32664_Hub::_freeWireBuffer()
{
    if (_rxBuffer != nullptr) {
        free(_rxBuffer);
        _rxBuffer = nullptr;
    }
    if (_txBuffer != nullptr) {
        free(_txBuffer);
        _txBuffer = nullptr;
    }
}

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function initializes the sensor. To place the MAX32664 into
// application mode, the MFIO pin must be pulled High while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be
// in application mode and will return two bytes, the first 0x00 is a
// successful communcation byte, followed by 0x00 which is the byte indicating
// which mode the IC is in.
uint8_t Max32664_Hub::begin(gpio_num_t resetPin, gpio_num_t mfioPin)
{
    if (resetPin >= GPIO_NUM_0) {
        _resetPin = resetPin;
        gpio_set_direction(_resetPin, GPIO_MODE_OUTPUT); // Set the pin as output
    }

    if (mfioPin >= GPIO_NUM_0) {
        _mfioPin = mfioPin;
        gpio_set_direction(_mfioPin, GPIO_MODE_OUTPUT); // Set the pin as output
    }

    if ((_resetPin == GPIO_NUM_NC) || (_mfioPin == GPIO_NUM_NC)) { // Bail if the pins have still not been defined
        return ERR_UNKNOWN;                                        // Return ERR_UNKNOWN
    }

    gpio_set_level(_resetPin, GPIO_HIGH);
    vTaskDelay(portTICK_PERIOD_MS * 30);

    gpio_set_level(_mfioPin, GPIO_HIGH);
    gpio_set_level(_resetPin, GPIO_LOW);
    vTaskDelay(portTICK_PERIOD_MS * 30);
    gpio_set_level(_resetPin, GPIO_HIGH);
    vTaskDelay(portTICK_PERIOD_MS * 1000);
    gpio_set_direction(_mfioPin, GPIO_MODE_INPUT);  // To be used as an interrupt later
    gpio_set_pull_mode(_mfioPin, GPIO_PULLUP_ONLY); // To be used as an interrupt later

    uint8_t const responseByte = _readByte(READ_DEVICE_MODE, 0x00); // 0x00 only possible Index Byte.

    return responseByte;
}

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function puts the MAX32664 into bootloader mode. To place the MAX32664 into
// bootloader mode, the MFIO pin must be pulled LOW while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be
// in bootloader mode and will return two bytes, the first 0x00 is a
// successful communcation byte, followed by 0x08 which is the byte indicating
// that the board is in bootloader mode.
uint8_t Max32664_Hub::beginBootloader(gpio_num_t resetPin, gpio_num_t mfioPin)
{
    if (resetPin >= GPIO_NUM_0) {
        _resetPin = resetPin;
        gpio_set_direction(_resetPin, GPIO_MODE_OUTPUT); // Set the pin as output
    }

    if (mfioPin >= GPIO_NUM_0) {
        _mfioPin = mfioPin;
        gpio_set_direction(_mfioPin, GPIO_MODE_OUTPUT); // Set the pin as output
    }

    if ((_resetPin == GPIO_NUM_NC) || (_mfioPin == GPIO_NUM_NC)) { // Bail if the pins have still not been defined
        return ERR_UNKNOWN;                                        // Return ERR_UNKNOWN
    }

    gpio_set_level(_mfioPin, GPIO_LOW);
    gpio_set_level(_resetPin, GPIO_LOW);
    vTaskDelay(portTICK_PERIOD_MS * 10);
    gpio_set_level(_resetPin, GPIO_HIGH);
    vTaskDelay(portTICK_PERIOD_MS * 50); // Bootloader mode is enabled when this ends.
    gpio_set_direction(_resetPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(_mfioPin, GPIO_MODE_OUTPUT);

    // Let's check to see if the device made it into bootloader mode.
    uint8_t const responseByte = _readByte(READ_DEVICE_MODE, 0x00); // 0x00 only possible Index Byte
    return responseByte;
}

// Family Byte: HUB_STATUS (0x00), Index Byte: 0x00, No Write Byte.
// The following function checks the status of the FIFO.
uint8_t Max32664_Hub::readSensorHubStatus()
{
#if defined(READBYTE_FAST)
    uint8_t const status = _readByteFast(0x00, 0x00); // Just family and index byte.
#else
    uint8_t const status = _readByte(0x00, 0x00); // Just family and index byte.
#endif

    return status; // Will return 0x00
}

// This function sets very basic settings to get sensor and biometric data.
// The biometric data includes data about heartrate, the confidence
// level, SpO2 levels, and whether the sensor has detected a finger or not.
uint8_t Max32664_Hub::configBpm(uint8_t mode)
{
    uint8_t statusChauf = 0;
    if (mode == MODE_ONE || mode == MODE_TWO) {
    } else {
        return INCORR_PARAM;
    }

    statusChauf = setOutputMode(ALGO_DATA); // Just the data
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    statusChauf = setFifoThreshold(0x01); // One sample before interrupt is fired.
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    statusChauf = agcAlgoControl(ENABLE); // One sample before interrupt is fired.
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    statusChauf = max30101Control(ENABLE);
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    statusChauf = maximFastAlgoControl(mode);
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    _userSelectedMode = mode;
    _sampleRate = readAlgoSamples();

    vTaskDelay(portTICK_PERIOD_MS * 1000);
    return SUCCESS;
}

// This function sets very basic settings to get LED count values from the MAX30101.
// Sensor data includes 24 bit LED values for the three LED channels: Red, IR,
// and Green.
uint8_t Max32664_Hub::configSensor()
{
    uint8_t statusChauf; // Our status chauffeur

    statusChauf = setOutputMode(SENSOR_DATA); // Just the sensor data (LED)
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    statusChauf = setFifoThreshold(0x01); // One sample before interrupt is fired to the MAX32664
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    statusChauf = max30101Control(ENABLE); // Enable Sensor.
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    statusChauf = maximFastAlgoControl(MODE_ONE); // Enable algorithm
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    vTaskDelay(portTICK_PERIOD_MS * 1000);
    return SUCCESS;
}

// This function sets very basic settings to get sensor and biometric data.
// Sensor data includes 24 bit LED values for the two LED channels: Red and IR.
// The biometric data includes data about heartrate, the confidence
// level, SpO2 levels, and whether the sensor has detected a finger or not.
// Of note, the number of samples is set to one.
uint8_t Max32664_Hub::configSensorBpm(uint8_t mode)
{
    uint8_t statusChauf; // Our status chauffeur
    if (mode == MODE_ONE || mode == MODE_TWO) {
    } else {
        return INCORR_PARAM;
    }

    statusChauf = setOutputMode(SENSOR_AND_ALGORITHM); // Data and sensor data
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    statusChauf = setFifoThreshold(0x01); // One sample before interrupt is fired to the MAX32664
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    statusChauf = max30101Control(ENABLE); // Enable Sensor.
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    statusChauf = maximFastAlgoControl(mode); // Enable algorithm
    if (statusChauf != SUCCESS) {
        return statusChauf;
    }

    _userSelectedMode = mode;
    _sampleRate = readAlgoSamples();

    vTaskDelay(portTICK_PERIOD_MS * 1000);
    return SUCCESS;
}

// This function takes the 8 bytes from the FIFO buffer related to the wrist
// heart rate algortihm: heart rate (uint16_t), confidence (uint8_t) , SpO2 (uint16_t),
// and the finger detected status (uint8_t). Note that the the algorithm is stated as
// "wrist" though the sensor only works with the finger. The data is loaded
// into the whrmFifo and returned.
bioData Max32664_Hub::readBpm()
{
    bioData libBpm;
    uint8_t statusChauf; // The status chauffeur captures return values.

    statusChauf = readSensorHubStatus();

    if (statusChauf == 1) { // Communication Error
        libBpm.heartRate = 0;
        libBpm.confidence = 0;
        libBpm.oxygen = 0;
        return libBpm;
    }

    numSamplesOutFifo();

    if (_userSelectedMode == MODE_ONE) {
        _readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE, bpmArr);

        // Heart Rate formatting
        libBpm.heartRate = (uint16_t(bpmArr[0]) << 8);
        libBpm.heartRate |= (bpmArr[1]);
        libBpm.heartRate /= 10;

        // Confidence formatting
        libBpm.confidence = bpmArr[2];

        // Blood oxygen level formatting
        libBpm.oxygen = uint16_t(bpmArr[3]) << 8;
        libBpm.oxygen |= bpmArr[4];
        libBpm.oxygen /= 10;

        //"Machine State" - has a finger been detected?
        libBpm.status = bpmArr[5];

        return libBpm;
    }

    else if (_userSelectedMode == MODE_TWO) {
        _readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA, bpmArrTwo);

        // Heart Rate formatting
        libBpm.heartRate = (uint16_t(bpmArrTwo[0]) << 8);
        libBpm.heartRate |= (bpmArrTwo[1]);
        libBpm.heartRate /= 10;

        // Confidence formatting
        libBpm.confidence = bpmArrTwo[2];

        // Blood oxygen level formatting
        libBpm.oxygen = uint16_t(bpmArrTwo[3]) << 8;
        libBpm.oxygen |= bpmArrTwo[4];
        libBpm.oxygen /= 10;

        //"Machine State" - has a finger been detected?
        libBpm.status = bpmArrTwo[5];

        // Sp02 r Value formatting
        uint16_t tempVal = uint16_t(bpmArrTwo[6]) << 8;
        tempVal |= bpmArrTwo[7];
        libBpm.rValue = (float)tempVal;
        libBpm.rValue /= 10.0;

        // Extended Machine State formatting
        libBpm.extStatus = (int8_t)bpmArrTwo[8];

        // There are two additional bytes of data that were requested but that
        // have not been implemented in firmware 10.1 so will not be saved to
        // user's data.
        return libBpm;
    }

    else {
        libBpm.heartRate = 0;
        libBpm.confidence = 0;
        libBpm.oxygen = 0;
        return libBpm;
    }
}

// This function takes 9 bytes of LED values from the MAX30101 associated with
// the RED, IR, and GREEN LEDs. In addition it gets the 8 bytes from the FIFO buffer
// related to the wrist heart rate algortihm: heart rate (uint16_t), confidence (uint8_t),
// SpO2 (uint16_t), and the finger detected status (uint8_t). Note that the the algorithm
// is stated as "wrist" though the sensor only works with the finger. The data is loaded
// into the whrmFifo and returned.
bioData Max32664_Hub::readSensor()
{
    bioData libLedFifo;
    _readFillArray(READ_DATA_OUTPUT, READ_DATA, MAX30101_LED_ARRAY, senArr);

    // Value of LED one....
    libLedFifo.irLed = uint32_t(senArr[0]) << 16;
    libLedFifo.irLed |= uint32_t(senArr[1]) << 8;
    libLedFifo.irLed |= senArr[2];

    // Value of LED two...
    libLedFifo.redLed = uint32_t(senArr[3]) << 16;
    libLedFifo.redLed |= uint32_t(senArr[4]) << 8;
    libLedFifo.redLed |= senArr[5];

    // Value of LED three...
    libLedFifo.greenLed = uint32_t(senArr[6]) << 16;
    libLedFifo.greenLed |= uint32_t(senArr[7]) << 8;
    libLedFifo.greenLed |= senArr[8];

    return libLedFifo;
}

// This function takes the information of both the LED value and the biometric
// data from the MAX32664's FIFO. In essence it combines the two functions
// above into a single function call.
bioData Max32664_Hub::readSensorBpm()
{
    bioData libLedBpm;

    if (_userSelectedMode == MODE_ONE) {
        _readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAX30101_LED_ARRAY, bpmSenArr);

        // Value of LED one....
        libLedBpm.irLed = uint32_t(bpmSenArr[0]) << 16;
        libLedBpm.irLed |= uint32_t(bpmSenArr[1]) << 8;
        libLedBpm.irLed |= bpmSenArr[2];

        // Value of LED two...
        libLedBpm.redLed = uint32_t(bpmSenArr[3]) << 16;
        libLedBpm.redLed |= uint32_t(bpmSenArr[4]) << 8;
        libLedBpm.redLed |= bpmSenArr[5];

        // Value of LED three...
        libLedBpm.greenLed = uint32_t(bpmSenArr[6]) << 16;
        libLedBpm.greenLed |= uint32_t(bpmSenArr[7]) << 8;
        libLedBpm.greenLed |= bpmSenArr[8];

        // -- What happened here? -- There are two uint32_t values that are given by
        // the sensor for LEDs that do not exists on the MAX30101. So we have to
        // request those empty values because they occupy the buffer:
        // bpmSenArr[6-11].

        // Heart rate formatting
        libLedBpm.heartRate = (uint16_t(bpmSenArr[12]) << 8);
        libLedBpm.heartRate |= (bpmSenArr[13]);
        libLedBpm.heartRate /= 10;

        // Confidence formatting
        libLedBpm.confidence = bpmSenArr[14];

        // Blood oxygen level formatting
        libLedBpm.oxygen = uint16_t(bpmSenArr[15]) << 8;
        libLedBpm.oxygen |= bpmSenArr[16];
        libLedBpm.oxygen /= 10;

        //"Machine State" - has a finger been detected?
        libLedBpm.status = bpmSenArr[17];
        return libLedBpm;
    }

    else if (_userSelectedMode == MODE_TWO) {
        _readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE + MAX30101_LED_ARRAY + MAXFAST_EXTENDED_DATA,
                       bpmSenArrTwo);

        // Value of LED one....
        libLedBpm.irLed = uint32_t(bpmSenArrTwo[0]) << 16;
        libLedBpm.irLed |= uint32_t(bpmSenArrTwo[1]) << 8;
        libLedBpm.irLed |= bpmSenArrTwo[2];

        // Value of LED two...
        libLedBpm.redLed = uint32_t(bpmSenArrTwo[3]) << 16;
        libLedBpm.redLed |= uint32_t(bpmSenArrTwo[4]) << 8;
        libLedBpm.redLed |= bpmSenArrTwo[5];

        // -- What happened here? -- There are two uint32_t values that are given by
        // the sensor for LEDs that do not exists on the MAX30101. So we have to
        // request those empty values because they occupy the buffer:
        // bpmSenArrTwo[6-11].

        // Heart rate formatting
        libLedBpm.heartRate = (uint16_t(bpmSenArrTwo[12]) << 8);
        libLedBpm.heartRate |= (bpmSenArrTwo[13]);
        libLedBpm.heartRate /= 10;

        // Confidence formatting
        libLedBpm.confidence = bpmSenArrTwo[14];

        // Blood oxygen level formatting
        libLedBpm.oxygen = uint16_t(bpmSenArrTwo[15]) << 8;
        libLedBpm.oxygen |= bpmSenArrTwo[16];
        libLedBpm.oxygen /= 10;

        //"Machine State" - has a finger been detected?
        libLedBpm.status = bpmSenArrTwo[17];

        // Sp02 r Value formatting
        uint16_t tempVal = uint16_t(bpmSenArrTwo[18]) << 8;
        tempVal |= bpmSenArrTwo[19];
        libLedBpm.rValue = (float)tempVal;
        libLedBpm.rValue /= 10.0;

        // Extended Machine State formatting
        libLedBpm.extStatus = (int8_t)bpmSenArrTwo[20];

        // There are two additional bytes of data that were requested but that
        // have not been implemented in firmware 10.1 so will not be saved to
        // user's data.
        //
        return libLedBpm;

    }

    else {
        libLedBpm.irLed = 0;
        libLedBpm.redLed = 0;
        libLedBpm.heartRate = 0;
        libLedBpm.confidence = 0;
        libLedBpm.oxygen = 0;
        libLedBpm.status = 0;
        libLedBpm.rValue = 0.0;
        libLedBpm.extStatus = 0;
        return libLedBpm;
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
uint8_t Max32664_Hub::setPulseWidth(uint16_t width)
{
    uint8_t bits;
    uint8_t regVal;

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
    regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= PULSE_MASK;                                  // Mask bits to change.
    regVal |= bits;                                        // Add bits
    writeRegisterMAX30101(CONFIGURATION_REGISTER, regVal); // Write Register

    return SUCCESS;
}

// This function reads the CONFIGURATION_REGISTER (0x0A), bits [1:0] from the
// MAX30101 Sensor. It returns one of the four settings in microseconds.
uint16_t Max32664_Hub::readPulseWidth()
{
    uint8_t regVal;

    regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= READ_PULSE_MASK;

    if (regVal == 0) {
        return 69;
    } else if (regVal == 1) {
        return 118;
    } else if (regVal == 2) {
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
uint8_t Max32664_Hub::setSampleRate(uint16_t sampRate)
{
    uint8_t bits;
    uint8_t regVal;

    // Make sure the correct sample rate was picked
    if (sampRate == 50) {
        bits = 0;
    } else if (sampRate == 100) {
        bits = 1;
    } else if (sampRate == 200) {
        bits = 2;
    } else if (sampRate == 400) {
        bits = 3;
    } else if (sampRate == 800) {
        bits = 4;
    } else if (sampRate == 1000) {
        bits = 5;
    } else if (sampRate == 1600) {
        bits = 6;
    } else if (sampRate == 3200) {
        bits = 7;
    } else {
        return INCORR_PARAM;
    }

    // Get current register value so that nothing is overwritten.
    regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= SAMP_MASK;                                   // Mask bits to change.
    regVal |= (bits << 2);                                 // Add bits but shift them first to correct position.
    writeRegisterMAX30101(CONFIGURATION_REGISTER, regVal); // Write Register

    return SUCCESS;
}

// This function reads the CONFIGURATION_REGISTER (0x0A), bits [4:2] from the
// MAX30101 Sensor. It returns one of the 8 possible sample rates.
uint16_t Max32664_Hub::readSampleRate()
{
    uint8_t regVal;

    regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= READ_SAMP_MASK;
    regVal = (regVal >> 2);

    if (regVal == 0) {
        return 50;
    } else if (regVal == 1) {
        return 100;
    } else if (regVal == 2) {
        return 200;
    } else if (regVal == 3) {
        return 400;
    } else if (regVal == 4) {
        return 800;
    } else if (regVal == 5) {
        return 1000;
    } else if (regVal == 6) {
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
uint8_t Max32664_Hub::setAdcRange(uint16_t adcVal)
{
    uint8_t regVal;
    uint8_t bits;

    if (adcVal <= 2048) {
        bits = 0;
    } else if (adcVal <= 4096) {
        bits = 1;
    } else if (adcVal <= 8192) {
        bits = 2;
    } else if (adcVal <= 16384) {
        bits = 3;
    } else {
        return INCORR_PARAM;
    }

    regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= ADC_MASK;
    regVal |= bits << 5;

    writeRegisterMAX30101(CONFIGURATION_REGISTER, regVal);

    return SUCCESS;
}

// MAX30101 Register: CONFIGURATION_REGISTER (0x0A), bits [6:5]
// This function returns the set ADC range of the MAX30101 sensor.
uint16_t Max32664_Hub::readAdcRange()
{
    uint8_t regVal;
    regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
    regVal &= READ_ADC_MASK;
    regVal = (regVal >> 5); // Shift our bits to the front of the line.

    if (regVal == 0) {
        return 2048;
    } else if (regVal == 1) {
        return 4096;
    } else if (regVal == 2) {
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
uint8_t Max32664_Hub::setOperatingMode(uint8_t selection)
{
    // Must be one of the three....
    if (selection == EXIT_BOOTLOADER || selection == RESET || selection == ENTER_BOOTLOADER) {
    } else {
        return INCORR_PARAM;
    }

    uint8_t const statusByte = _writeByte(SET_DEVICE_MODE, 0x00, selection);
    if (statusByte != SUCCESS) {
        return statusByte;
    }

    // Here we'll check if the board made it into Bootloader mode...
    // responses are 0x00: Application operating mode. 0x08: Bootloader operating mode
    uint8_t const responseByte = _readByte(READ_DEVICE_MODE, 0x00);
    return responseByte; // This is in fact the status byte, need second returned byte - bootloader mode
}

// Family Byte: IDENTITY (0x01), Index Byte: READ_MCU_TYPE, Write Byte: NONE
// The following function returns a byte that signifies the microcontoller that
// is in communcation with your host microcontroller. Returns 0x00 for the
// MAX32625 and 0x01 for the MAX32660/MAX32664.
uint8_t Max32664_Hub::getMcuType()
{
    uint8_t const returnByte = _readByte(IDENTITY, READ_MCU_TYPE, NO_WRITE);
    if (returnByte != SUCCESS) {
        return ERR_UNKNOWN;
    } else {
        return returnByte;
    }
}

// Family Byte: BOOTLOADER_INFO (0x80), Index Byte: BOOTLOADER_VERS (0x00)
// This function checks the version number of the bootloader on the chip and
// returns a four bytes: Major version Byte, Minor version Byte, Space Byte,
// and the Revision Byte.
int32_t Max32664_Hub::getBootloaderInf()
{
    const size_t sizeOfRev = 4;
    int32_t revNum[sizeOfRev] = {};
    uint8_t const status = _readMultipleBytes(BOOTLOADER_INFO, BOOTLOADER_VERS, 0x00, 4, revNum);

    if (status == 0) {
        return ERR_UNKNOWN;
    } else {
        int32_t bootVers = 0;
        bootVers |= (int32_t(revNum[1]) << 16);
        bootVers |= (int32_t(revNum[2]) << 8);
        bootVers |= revNum[3];
        return bootVers;
    }
}

// Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30101 (0x03), Write
// Byte: senSwitch  (parameter - 0x00 or 0x01).
// This function enables the MAX30101.
uint8_t Max32664_Hub::max30101Control(uint8_t senSwitch)
{
    if (senSwitch == 0 || senSwitch == 1) {
    } else {
        return INCORR_PARAM;
    }

    // Check that communication was successful, not that the sensor is enabled.
    uint8_t const statusByte = _enableWrite(ENABLE_SENSOR, ENABLE_MAX30101, senSwitch);
    if (statusByte != SUCCESS) {
        return statusByte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: READ_SENSOR_MODE (0x45), Index Byte: READ_ENABLE_MAX30101 (0x03)
// This function checks if the MAX30101 is enabled or not.
uint8_t Max32664_Hub::readMAX30101State()
{
    uint8_t const state = _readByte(READ_SENSOR_MODE, READ_ENABLE_MAX30101);
    return state;
}

// Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_ACCELEROMETER (0x04), Write
// Byte: accepts (parameter - 0x00 or 0x01).
// This function enables the Accelerometer.
uint8_t Max32664_Hub::accelControl(uint8_t accelSwitch)
{
    if (accelSwitch != 0 && accelSwitch != 1) {
    } else {
        return INCORR_PARAM;
    }

    // Check that communication was successful, not that the sensor is enabled.
    uint8_t const statusByte = _enableWrite(ENABLE_SENSOR, ENABLE_ACCELEROMETER, accelSwitch);
    if (statusByte != SUCCESS) {
        return statusByte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: OUTPUT_MODE (0x10), Index Byte: SET_FORMAT (0x00),
// Write Byte : outputType (Parameter values in OUTPUT_MODE_WRITE_BYTE)
uint8_t Max32664_Hub::setOutputMode(uint8_t outputType)
{
    if (outputType > SENSOR_ALGO_COUNTER) { // Bytes between 0x00 and 0x07
        return INCORR_PARAM;
    }

    // Check that communication was successful, not that the IC is outputting
    // correct format.
    uint8_t const statusByte = _writeByte(OUTPUT_MODE, SET_FORMAT, outputType);
    if (statusByte != SUCCESS) {
        return statusByte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: OUTPUT_MODE(0x10), Index Byte: WRITE_SET_THRESHOLD (0x01), Write byte: intThres
// (parameter - value betwen 0 and 0xFF).
// This function changes the threshold for the FIFO interrupt bit/pin. The
// interrupt pin is the MFIO pin which is set to INPUT after IC initialization
// (begin).
uint8_t Max32664_Hub::setFifoThreshold(uint8_t intThresh)
{
    // Checks that there was succesful communcation, not that the threshold was
    // set correctly.
    uint8_t const statusByte = _writeByte(OUTPUT_MODE, WRITE_SET_THRESHOLD, intThresh);
    if (statusByte != SUCCESS) {
        return statusByte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: NUM_SAMPLES (0x00), Write
// Byte: NONE
// This function returns the number of samples available in the FIFO.
uint8_t Max32664_Hub::numSamplesOutFifo()
{
#if defined(READBYTE_FAST)
    uint8_t const sampAvail = _readByteFast(READ_DATA_OUTPUT, NUM_SAMPLES);
#else
    uint8_t const sampAvail = _readByte(READ_DATA_OUTPUT, NUM_SAMPLES);
#endif

    return sampAvail;
}

// Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_DATA (0x01), Write
// Byte: NONE
// This function returns the data in the FIFO.
uint8_t *Max32664_Hub::getDataOutFifo(uint8_t data[])
{
    uint8_t const samples = numSamplesOutFifo();
    _readFillArray(READ_DATA_OUTPUT, READ_DATA, samples, data);
    return data;
}

// Family Byte: READ_DATA_INPUT (0x13), Index Byte: FIFO_EXTERNAL_INDEX_BYTE (0x00), Write
// Byte: NONE
// This function adds support for the acceleromter that is NOT included on
// SparkFun's product, The Family Registery of 0x13 and 0x14 is skipped for now.
uint8_t Max32664_Hub::numSamplesExternalSensor()
{
    uint8_t const sampAvail = _readByte(READ_DATA_INPUT, SAMPLE_SIZE, WRITE_ACCELEROMETER);
    return sampAvail;
}

// Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX30101 (0x03), Write Bytes:
// Register Address and Register Value
// This function writes the given register value at the given register address
// for the MAX30101 sensor and returns a boolean indicating a successful or
// non-successful write.
void Max32664_Hub::writeRegisterMAX30101(uint8_t regAddr, uint8_t regVal)
{
    _writeByte(WRITE_REGISTER, WRITE_MAX30101, regAddr, regVal);
}

// Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_ACCELEROMETER (0x04), Write Bytes:
// Register Address and Register Value
// This function writes the given register value at the given register address
// for the Accelerometer and returns a boolean indicating a successful or
// non-successful write.
void Max32664_Hub::writeRegisterAccel(uint8_t regAddr, uint8_t regVal)
{
    _writeByte(WRITE_REGISTER, WRITE_ACCELEROMETER, regAddr, regVal);
}

// Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30101 (0x03), Write Byte:
// Register Address
// This function reads the given register address for the MAX30101 Sensor and
// returns the values at that register.
uint8_t Max32664_Hub::readRegisterMAX30101(uint8_t regAddr)
{
    uint8_t const regCont = _readByte(READ_REGISTER, READ_MAX30101, regAddr);
    return regCont;
}

// Family Byte: READ_REGISTER (0x41), Index Byte: READ_ACCELEROMETER (0x04), Write Byte:
// Register Address
// This function reads the given register address for the MAX30101 Sensor and
// returns the values at that register.
uint8_t Max32664_Hub::readRegisterAccel(uint8_t regAddr)
{
    uint8_t const regCont = _readByte(READ_REGISTER, READ_ACCELEROMETER, regAddr);
    return regCont;
}

// Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX30101 (0x03)
// This function retrieves the attributes of the AFE (Analog Front End) of the
// MAX30101 sensor. It returns the number of bytes in a word for the sensor
// and the number of registers available.
sensorAttr Max32664_Hub::getAfeAttributesMAX30101()
{
    sensorAttr maxAttr;
    uint8_t tempArray[2] {};

    _readFillArray(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_MAX30101, 2, tempArray);

    maxAttr.byteWord = tempArray[0];
    maxAttr.availRegisters = tempArray[1];

    return maxAttr;
}

// Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte:
// RETRIEVE_AFE_ACCELEROMETER (0x04)
// This function retrieves the attributes of the AFE (Analog Front End) of the
// Accelerometer. It returns the number of bytes in a word for the sensor
// and the number of registers available.
sensorAttr Max32664_Hub::getAfeAttributesAccelerometer()
{
    sensorAttr maxAttr;
    uint8_t tempArray[2] {};

    _readFillArray(READ_ATTRIBUTES_AFE, RETRIEVE_AFE_ACCELEROMETER, 2, tempArray);

    maxAttr.byteWord = tempArray[0];
    maxAttr.availRegisters = tempArray[1];

    return maxAttr;
}

// Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX30101 (0x03)
// This function returns all registers and register values sequentially of the
// MAX30101 sensor: register zero and register value zero to register n and
// register value n. There are 36 registers in this case.
uint8_t Max32664_Hub::dumpRegisterMAX30101(uint8_t regArray[])
{
    uint8_t const numOfBytes = 36;
    uint8_t const status = _readFillArray(DUMP_REGISTERS, DUMP_REGISTER_MAX30101, numOfBytes, regArray);
    return status;
}

// Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_ACCELEROMETER (0x04)
// This function returns all registers and register values sequentially of the
// Accelerometer: register zero and register value zero to register n and
// register value n.
uint8_t Max32664_Hub::dumpRegisterAccelerometer(uint8_t numReg, uint8_t regArray[])
{
    uint8_t const status =
        _readFillArray(DUMP_REGISTERS, DUMP_REGISTER_ACCELEROMETER, numReg, regArray); // Fake read amount
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_TARG_PERC (0x00), Write Byte: AGC_GAIN_ID (0x00)
// This function sets the target percentage of the full-scale ADC range that
// the automatic gain control algorithm uses. It takes a paramater of zero to
// 100 percent.
uint8_t Max32664_Hub::setAlgoRange(uint8_t perc)
{
    if (perc > 100) {
        return INCORR_PARAM;
    }

    // Successful communication or no?
    uint8_t const statusByte = _writeByte(CHANGE_ALGORITHM_CONFIG, SET_TARG_PERC, AGC_GAIN_ID, perc);
    if (statusByte != SUCCESS) {
        return statusByte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_STEP_SIZE (0x00), Write Byte: AGC_STEP_SIZE_ID (0x01)
// This function changes the step size toward the target for the AGC algorithm.
// It takes a paramater of zero to 100 percent.
uint8_t Max32664_Hub::setAlgoStepSize(uint8_t step)
{
    if (step > 100) {
        return INCORR_PARAM;
    }

    // Successful communication or no?
    uint8_t const statusByte = _writeByte(CHANGE_ALGORITHM_CONFIG, SET_STEP_SIZE, AGC_STEP_SIZE_ID, step);
    if (statusByte != SUCCESS) {
        return statusByte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_SENSITIVITY (0x00), Write Byte: AGC_SENSITIVITY_ID (0x02)
// This function changes the sensitivity of the AGC algorithm.
uint8_t Max32664_Hub::setAlgoSensitivity(uint8_t sense)
{
    if (sense > 100) {
        return INCORR_PARAM;
    }

    // Successful communication or no?
    uint8_t const statusByte = _writeByte(CHANGE_ALGORITHM_CONFIG, SET_SENSITIVITY, AGC_SENSITIVITY_ID, sense);
    if (statusByte != SUCCESS) {
        return statusByte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_AVG_SAMPLES (0x00), Write Byte: AGC_NUM_SAMP_ID (0x03)
// This function changes the number of samples that are averaged.
// It takes a paramater of zero to 255.
uint8_t Max32664_Hub::setAlgoSamples(uint8_t avg)
{
    // Successful communication or no?
    uint8_t const statusByte = _writeByte(CHANGE_ALGORITHM_CONFIG, SET_AVG_SAMPLES, AGC_NUM_SAMP_ID, avg);
    if (statusByte != SUCCESS) {
        return statusByte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// WHRM_CONFIG (0x02), Write Byte: MAXIMFAST_COEF_ID (0x0B)
// This function takes three values that are used as the Sp02 coefficients.
// These three values are multiplied by 100,000;
// default values are in order: 159584, -3465966, and 11268987.
uint8_t Max32664_Hub::setMaximFastCoef(int32_t coef1, int32_t coef2, int32_t coef3)
{
    const size_t numCoefVals = 3;
    int32_t coefArr[numCoefVals] = {coef1, coef2, coef3};

    uint8_t const statusByte =
        _writeLongBytes(CHANGE_ALGORITHM_CONFIG, WHRM_CONFIG, MAXIMFAST_COEF_ID, coefArr, numCoefVals);
    if (statusByte != SUCCESS) {
        return statusByte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_PERCENTAGE (0x00), Write Byte: READ_AGC_PERC_ID (0x00)
// This function reads and returns the currently set target percentage
// of the full-scale ADC range that the Automatic Gain Control algorithm is using.
uint8_t Max32664_Hub::readAlgoRange()
{
    uint8_t const range = _readByte(READ_ALGORITHM_CONFIG, READ_AGC_PERCENTAGE, READ_AGC_PERC_ID);
    return range;
}

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_STEP_SIZE (0x00), Write Byte: READ_AGC_STEP_SIZE_ID (0x01)
// This function returns the step size toward the target for the AGC algorithm.
// It returns a value between zero and 100 percent.
uint8_t Max32664_Hub::readAlgoStepSize()
{
    uint8_t const stepSize = _readByte(READ_ALGORITHM_CONFIG, READ_AGC_STEP_SIZE, READ_AGC_STEP_SIZE_ID);
    return stepSize;
}

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_SENSITIVITY_ID (0x00), Write Byte: READ_AGC_SENSITIVITY_ID (0x02)
// This function returns the sensitivity (percentage) of the automatic gain control.
uint8_t Max32664_Hub::readAlgoSensitivity()
{
    uint8_t const sensitivity = _readByte(READ_ALGORITHM_CONFIG, READ_AGC_SENSITIVITY, READ_AGC_SENSITIVITY_ID);
    return sensitivity;
}

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_NUM_SAMPLES (0x00), Write Byte: READ_AGC_NUM_SAMPLES_ID (0x03)
// This function changes the number of samples that are averaged.
// It takes a paramater of zero to 255.
uint8_t Max32664_Hub::readAlgoSamples()
{
    uint8_t const samples = _readByte(READ_ALGORITHM_CONFIG, READ_AGC_NUM_SAMPLES, READ_AGC_NUM_SAMPLES_ID);
    return samples;
}

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_MAX_FAST_COEF (0x02), Write Byte: READ_MAX_FAST_COEF_ID (0x0B)
// This function reads the maximum age for the wrist heart rate monitor
// (WHRM) algorithm. It returns three uint32_t integers that are
// multiplied by 100,000.
uint8_t Max32664_Hub::readMaximFastCoef(int32_t coefArr[3])
{
    const size_t numOfReads = 3;
    uint8_t const status =
        _readMultipleBytes(READ_ALGORITHM_CONFIG, READ_MAX_FAST_COEF, READ_MAX_FAST_COEF_ID, numOfReads, coefArr);
    coefArr[0] = coefArr[0] * 100000;
    coefArr[1] = coefArr[1] * 100000;
    coefArr[2] = coefArr[2] * 100000;
    return status;
}

// Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
// ENABLE_AGC_ALGO (0x00)
// This function enables (one) or disables (zero) the automatic gain control algorithm.
uint8_t Max32664_Hub::agcAlgoControl(uint8_t enable)
{
    if (enable == 0 || enable == 1) {
    } else {
        return INCORR_PARAM;
    }

    uint8_t const statusByte = _enableWrite(ENABLE_ALGORITHM, ENABLE_AGC_ALGO, enable);
    if (statusByte != SUCCESS) {
        return statusByte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
// ENABLE_WHRM_ALGO (0x02)
// This function enables (one) or disables (zero) the wrist heart rate monitor
// algorithm.
uint8_t Max32664_Hub::maximFastAlgoControl(uint8_t mode)
{
    if (mode == 0 || mode == 1 || mode == 2) {
    } else {
        return INCORR_PARAM;
    }

    uint8_t const statusByte = _enableWrite(ENABLE_ALGORITHM, ENABLE_WHRM_ALGO, mode);
    if (statusByte != SUCCESS) {
        return statusByte;
    } else {
        return SUCCESS;
    }
}

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_INIT_VECTOR_BYTES (0x00)
// void Max32664_Hub::setInitBytes

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_AUTH_BYTES (0x01)

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_NUM_PAGES (0x02),
// Write Bytes: 0x00 - Number of pages at byte 0x44 from .msbl file.
bool Max32664_Hub::setNumPages(uint8_t totalPages)
{
    uint8_t const statusByte = _writeByte(BOOTLOADER_FLASH, SET_NUM_PAGES, 0x00, totalPages);
    return statusByte;
}

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: ERASE_FLASH (0x03)
// Returns true on successful communication.
bool Max32664_Hub::eraseFlash()
{
    // This is a unique write in that it does not have a relevant write byte.
    _beginTransmission(_address);
    _write(BOOTLOADER_FLASH);
    _write(ERASE_FLASH);
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t const statusByte = _read();
    return (statusByte == 0);
}

// Family Byte: BOOTLOADER_INFO (0x81), Index Byte: BOOTLOADER_VERS (0x00)
version Max32664_Hub::readBootloaderVers()
{
    version booVers; // BOO!
    _beginTransmission(_address);
    _write(BOOTLOADER_INFO);
    _write(BOOTLOADER_VERS);
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _requestFrom(_address, static_cast<uint8_t>(4));
    uint8_t const statusByte = _read();
    if (statusByte) { // Pass through if SUCCESS (0x00).
        booVers.major = 0;
        booVers.minor = 0;
        booVers.revision = 0;
        return booVers;
    }

    booVers.major = _read();
    booVers.minor = _read();
    booVers.revision = _read();

    return booVers;
}

// Family Byte: IDENTITY (0xFF), Index Byte: READ_SENSOR_HUB_VERS (0x03)
version Max32664_Hub::readSensorHubVersion()
{
    version bioHubVers;
    _beginTransmission(_address);
    _write(IDENTITY);
    _write(READ_SENSOR_HUB_VERS);
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _requestFrom(_address, static_cast<uint8_t>(4));
    uint8_t const statusByte = _read();
    if (statusByte) { // Pass through if SUCCESS (0x00).
        bioHubVers.major = 0;
        bioHubVers.minor = 0;
        bioHubVers.revision = 0;
        return bioHubVers;
    }

    bioHubVers.major = _read();
    bioHubVers.minor = _read();
    bioHubVers.revision = _read();

    return bioHubVers;
}

// Family Byte: IDENTITY (0xFF), Index Byte: READ_ALGO_VERS (0x07)
version Max32664_Hub::readAlgorithmVersion()
{
    version libAlgoVers;
    _beginTransmission(_address);
    _write(IDENTITY);
    _write(READ_ALGO_VERS);
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _requestFrom(_address, static_cast<uint8_t>(4));
    uint8_t const statusByte = _read();
    if (statusByte) { // Pass through if SUCCESS (0x00).
        libAlgoVers.major = 0;
        libAlgoVers.minor = 0;
        libAlgoVers.revision = 0;
        return libAlgoVers;
    }

    libAlgoVers.major = _read();
    libAlgoVers.minor = _read();
    libAlgoVers.revision = _read();

    return libAlgoVers;
}

// ------------------Function Below for MAX32664 Version D (Blood Pressure) ----

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_MEDICATION (0x00)
uint8_t Max32664_Hub::isPatientBPMedication(uint8_t medication)
{
    if (medication != 0x00 && medication != 0x01) {
        return INCORR_PARAM;
    }

    uint8_t const status = _writeByte(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_MEDICATION, medication);
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_MEDICATION (0x00)
uint8_t Max32664_Hub::isPatientBPMedication()
{
    uint8_t const medication = _readByte(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_MEDICATION);
    return medication;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: SYSTOLIC_VALUE (0x02)
uint8_t Max32664_Hub::writeSystolicVals(uint8_t sysVal1, uint8_t sysVal2, uint8_t sysVal3)
{
    const size_t numSysVals = 3;
    uint8_t sysVals[numSysVals] = {sysVal1, sysVal2, sysVal3};
    uint8_t const status = _writeBytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, SYSTOLIC_VALUE, sysVals, numSysVals);

    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: SYSTOLIC_VALUE (0x02)
uint8_t Max32664_Hub::readSystolicVals(uint8_t userArray[])
{
    const size_t numSysVals = 3;
    uint8_t const status =
        _readMultipleBytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, SYSTOLIC_VALUE, numSysVals, userArray);

    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: DIASTOLIC_VALUE (0x01)
uint8_t Max32664_Hub::writeDiastolicVals(uint8_t diasVal1, uint8_t diasVal2, uint8_t diasVal3)
{
    const size_t numDiasVals = 3;
    uint8_t diasVals[numDiasVals] = {diasVal1, diasVal2, diasVal3};
    uint8_t const status = _writeBytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, DIASTOLIC_VALUE, diasVals, numDiasVals);

    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: DIASTOLIC_VALUE (0x01)
uint8_t Max32664_Hub::readDiastolicVals(uint8_t userArray[])
{
    const size_t numDiasVals = 3;
    uint8_t const status =
        _readMultipleBytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, DIASTOLIC_VALUE, numDiasVals, userArray);
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_CALIB_DATA (0x03)
uint8_t Max32664_Hub::writeBPTAlgoData(const uint8_t bptCalibData[])
{
    const size_t numCalibVals = 824;
    uint8_t const status = _writeBytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_CALIB_DATA, bptCalibData, numCalibVals);
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_CALIB_DATA (0x03)
uint8_t Max32664_Hub::readBPTAlgoData(uint8_t userArray[])
{
    const size_t numCalibVals = 824;
    uint8_t const status =
        _readMultipleBytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_CALIB_DATA, numCalibVals, userArray);
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: PATIENT_RESTING (0x05)
uint8_t Max32664_Hub::isPatientResting(uint8_t resting)
{ //

    if (resting != 0x00 && resting != 0x01) {
        return INCORR_PARAM;
    }

    uint8_t const status = _writeByte(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, PATIENT_RESTING, resting);
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: PATIENT_RESTING (0x05)
uint8_t Max32664_Hub::isPatientResting()
{
    uint8_t const resting = _writeByte(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, PATIENT_RESTING);
    return resting;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_SP02_COEFS (0x06)
uint8_t Max32664_Hub::writeSP02AlgoCoef(int32_t intA, int32_t intB, int32_t intC)
{
    const size_t numCoefVals = 3;
    int32_t coefVals[numCoefVals] = {intA, intB, intC};
    uint8_t const status = _writeLongBytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_SP02_COEFS, coefVals, numCoefVals);
    return status;
}

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_SP02_COEFS (0x06)
uint8_t Max32664_Hub::readSP02AlgoCoef(int32_t userArray[])
{ // Have the user provide their own array here and pass the pointer to it

    const size_t numOfReads = 3;
    uint8_t const status =
        _readMultipleBytes(CHANGE_ALGORITHM_CONFIG, BPT_CONFIG, BPT_SP02_COEFS, numOfReads, userArray);
    return status;
}

//-------------------Private Functions-----------------------

void Max32664_Hub::_beginTransmission(uint16_t address)
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
    _nonStop = false;
    _txAddress = address;
    _txLength = 0;
}

void Max32664_Hub::_beginTransmission(int16_t address) { _beginTransmission(static_cast<uint16_t>(address)); }

void Max32664_Hub::_beginTransmission(uint8_t address) { _beginTransmission(static_cast<uint16_t>(address)); }

uint8_t Max32664_Hub::_endTransmission() { return _endTransmission(true); }

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
uint8_t Max32664_Hub::_endTransmission(bool sendStop)
{
    if (_txBuffer == nullptr) {
        ESP_LOGE(TAG, "NULL TX buffer pointer");
        return 4;
    }
    esp_err_t err = ESP_OK;
    if (sendStop) {
        err = i2c_master_write_to_device(I2C_NUM_0, _txAddress, _txBuffer, _txLength, _timeOutMillis);
#if defined(CONFIG_ENABLE_HAL_LOCKS)
        // release lock
        xSemaphoreGive(_lock);
#endif
    } else {
        // mark as non-stop
        _nonStop = true;
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

size_t Max32664_Hub::_write(uint8_t data)
{
    if (_txBuffer == nullptr) {
        ESP_LOGE(TAG, "NULL TX buffer pointer");
        return 0;
    }
    if (_txLength >= _bufferSize) {
        return 0;
    }
    _txBuffer[_txLength++] = data;
    return 1;
}

size_t Max32664_Hub::_write(const uint8_t *data, size_t quantity)
{
    for (size_t i = 0; i < quantity; ++i) {
        if (_write(data[i]) == 0) {
            return i;
        }
    }
    return quantity;
}

int16_t Max32664_Hub::_read()
{
    int16_t value = -1;
    if (_rxBuffer == nullptr) {
        ESP_LOGE(TAG, "NULL RX buffer pointer");
        return value;
    }
    if (_rxIndex < _rxLength) {
        value = _rxBuffer[_rxIndex++];
    }
    return value;
}

size_t Max32664_Hub::_requestFrom(uint16_t address, size_t size)
{
    if (_rxBuffer == nullptr || _txBuffer == nullptr) {
        ESP_LOGE(TAG, "NULL buffer pointer");
        return 0;
    }
    esp_err_t err = ESP_OK;
    if (_nonStop
#if defined(CONFIG_ENABLE_HAL_LOCKS)
        && _nonStopTask == xTaskGetCurrentTaskHandle()
#endif
    ) {
        if (address != _txAddress) {
            ESP_LOGE(TAG, "Unfinished Repeated Start transaction! Expected address do not match! %u != %u", address,
                     _txAddress);
            return 0;
        }
        _nonStop = false;
        _rxIndex = 0;
        _rxLength = 0;
        err = i2c_master_write_read_device(I2C_NUM_0, address, _txBuffer, _txLength, _rxBuffer, size, _timeOutMillis);
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
        _rxIndex = 0;
        _rxLength = 0;
        err = i2c_master_read_from_device(I2C_NUM_0, address, _rxBuffer, size, _timeOutMillis);
        if (err) {
            if (err == ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "i2c_master_read_from_device returned Timeout");
            } else {
                ESP_LOGE(TAG, "i2c_master_read_from_device returned Error %d", err);
            }
        } else {
            _rxLength = size;
        }
    }
#if defined(CONFIG_ENABLE_HAL_LOCKS)
    // release lock
    xSemaphoreGive(_lock);
#endif
    return _rxLength;
}

size_t Max32664_Hub::_requestFrom(uint8_t address, size_t len)
{
    return _requestFrom(static_cast<uint16_t>(address), static_cast<size_t>(len));
}

uint8_t Max32664_Hub::_requestFrom(uint8_t address, uint8_t len)
{
    return _requestFrom(static_cast<uint16_t>(address), static_cast<size_t>(len));
}

uint8_t Max32664_Hub::_requestFrom(uint16_t address, uint8_t len)
{
    return _requestFrom(address, static_cast<size_t>(len));
}

uint8_t Max32664_Hub::_requestFrom(int16_t address, int16_t len)
{
    return _requestFrom(static_cast<uint16_t>(address), static_cast<size_t>(len));
}

// This function uses the given family, index, and write byte to enable
// the given sensor.
uint8_t Max32664_Hub::_enableWrite(uint8_t familyByte, uint8_t indexByte, uint8_t enableByte)
{
    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _write(enableByte);
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * ENABLE_CMD_DELAY); // delayMicroseconds(ENABLE_CMD_DELAY * 1000U);

    // Status Byte, success or no? 0x00 is a successful transmit
    _requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t const statusByte = _read();
    return statusByte;
}

// This function uses the given family, index, and write byte to communicate
// with the MAX32664 which in turn communicates with downward sensors. There
// are two steps demonstrated in this function. First a write to the MCU
// indicating what you want to do, a delay, and then a read to confirm positive
// transmission.
uint8_t Max32664_Hub::_writeByte(uint8_t familyByte, uint8_t indexByte, uint8_t writeByte)
{
    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _write(writeByte);
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    // Status Byte, success or no? 0x00 is a successful transmit
    _requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t const statusByte = _read();
    return statusByte;
}

// This function is the same as the function above and uses the given family,
// index, and write byte, but also takes a 16 bit integer as a paramter to communicate
// with the MAX32664 which in turn communicates with downward sensors. There
// are two steps demonstrated in this function. First a write to the MCU
// indicating what you want to do, a delay, and then a read to confirm positive
// transmission.
uint8_t Max32664_Hub::_writeByte(uint8_t familyByte, uint8_t indexByte, uint8_t writeByte, uint16_t val)
{
    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _write(writeByte);
    _write((val >> 8)); // MSB
    _write(val);        // LSB
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    // Status Byte, success or no? 0x00 is a successful transmit
    _requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t const statusByte = _read();
    return statusByte;
}

// This function sends information to the MAX32664 to specifically write values
// to the registers of downward sensors and so also requires a
// register address and register value as parameters. Again there is the write
// of the specific bytes followed by a read to confirm positive transmission.
uint8_t Max32664_Hub::_writeByte(uint8_t familyByte, uint8_t indexByte, uint8_t writeByte, uint8_t writeVal)
{
    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _write(writeByte);
    _write(writeVal);
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    // Status Byte, 0x00 is a successful transmit.
    _requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t const statusByte = _read();
    return statusByte;
}

// This function sends information to the MAX32664 to specifically write values
// to the registers of downward sensors and so also requires a
// register address and register value as parameters. Again there is the write
// of the specific bytes followed by a read to confirm positive transmission.
uint8_t Max32664_Hub::_writeLongBytes(uint8_t familyByte, uint8_t indexByte, uint8_t writeByte,
                                      const int32_t writeVal[], const size_t size)
{
    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _write(writeByte);

    for (size_t i = 0; i < size; i++) {
        _write(writeVal[i] >> 24);
        _write(writeVal[i] >> 16);
        _write(writeVal[i] >> 8);
        _write(writeVal[i]);
    }

    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    // Status Byte, 0x00 is a successful transmit.
    _requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t const statusByte = _read();
    return statusByte;
}

// This function sends information to the MAX32664 to specifically write values
// to the registers of downward sensors and so also requires a
// register address and register value as parameters. Again there is the write
// of the specific bytes followed by a read to confirm positive transmission.
uint8_t Max32664_Hub::_writeBytes(uint8_t familyByte, uint8_t indexByte, uint8_t writeByte, const uint8_t writeVal[],
                                  const size_t size)
{
    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _write(writeByte);

    for (size_t i = 0; i < size; i++) {
        _write(writeVal[i]);
    }

    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY);

    // Status Byte, 0x00 is a successful transmit.
    _requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t const statusByte = _read();
    return statusByte;
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte an index byte, and
// then delays 60 microseconds, during which the MAX32664 retrieves the requested
// information. An I-squared-C request is then issued, and the information is read.
uint8_t Max32664_Hub::_readByte(uint8_t familyByte, uint8_t indexByte)
{
    uint8_t returnByte;
    uint8_t statusByte;

    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _requestFrom(_address, static_cast<uint8_t>(sizeof(returnByte) + sizeof(statusByte)));
    statusByte = _read();
    if (statusByte) {      // SUCCESS (0x00) - how do I know its
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    returnByte = _read();
    return returnByte; // If good then return the actual byte.
}

uint8_t Max32664_Hub::_readByteFast(uint8_t familyByte, uint8_t indexByte)
{
    uint8_t returnByte;
    uint8_t statusByte;

    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _endTransmission();
    // delayMicroseconds(CMD_DELAY*1000U);

    _requestFrom(_address, static_cast<uint8_t>(sizeof(returnByte) + sizeof(statusByte)));
    statusByte = _read();
    if (statusByte) {      // SUCCESS (0x00) - how do I know its
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    returnByte = _read();
    return returnByte; // If good then return the actual byte.
}

uint8_t Max32664_Hub::_readByteFast(uint8_t familyByte, uint8_t indexByte, uint8_t writeByte)
{
    uint8_t returnByte;
    uint8_t statusByte;

    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _write(writeByte);
    _endTransmission();
    // delayMicroseconds(CMD_DELAY*1000U);

    _requestFrom(_address, static_cast<uint8_t>(sizeof(returnByte) + sizeof(statusByte)));
    statusByte = _read();
    if (statusByte) {      // SUCCESS (0x00) - how do I know its
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    returnByte = _read();
    return returnByte; // If good then return the actual byte.
}

// This function is exactly as the one above except it accepts also receives a
// Write Byte as a parameter. It starts a request by writing the family byte, index byte, and
// write byte to the MAX32664 and then delays 60 microseconds, during which
// the MAX32664 retrieves the requested information. A I-squared-C request is
// then issued, and the information is read.
uint8_t Max32664_Hub::_readByte(uint8_t familyByte, uint8_t indexByte, uint8_t writeByte)
{
    uint8_t returnByte;
    uint8_t statusByte;

    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _write(writeByte);
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _requestFrom(_address, static_cast<uint8_t>(sizeof(returnByte) + sizeof(statusByte)));
    statusByte = _read();
    if (statusByte) {      // SUCCESS (0x00)
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    returnByte = _read();
    return returnByte; // If good then return the actual byte.
}

// This functions is similar to _readMultipleBytes() except it doesn't send a write byte
// and all the information is read at once and returned in a uint8_t array by reference.
uint8_t Max32664_Hub::_readFillArray(uint8_t familyByte, uint8_t indexByte, uint8_t numOfReads, uint8_t array[])
{
    uint8_t statusByte;

    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _endTransmission();

    // vTaskDelay(portTICK_PERIOD_MS * 1);

    _requestFrom(_address, static_cast<uint8_t>(numOfReads + sizeof(statusByte)));
    statusByte = _read();
    // if( (statusByte!=0) && (statusByte!=255) ){// SUCCESS (0x00)
    if (statusByte != 0) { // SUCCESS (0x00)
        for (uint8_t i = 0; i < numOfReads; i++) {
            array[i] = 0;
        }
        return statusByte;
    }

    for (size_t i = 0; i < numOfReads; i++) {
        array[i] = _read();
    }
    return statusByte;
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, an index byte, and
// a write byte and then then delays 60 microseconds, during which the MAX32664
// retrieves the requested information. An I-squared-C request is then issued,
// and the information is read. This differs from the above read commands in
// that it returns a 16 bit integer instead of a single byte.
uint16_t Max32664_Hub::_readIntByte(uint8_t familyByte, uint8_t indexByte, uint8_t writeByte)
{
    uint16_t returnByte;
    uint8_t statusByte;

    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _write(writeByte);
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _requestFrom(_address, static_cast<uint8_t>(sizeof(returnByte) + sizeof(statusByte)));
    statusByte = _read();
    if (statusByte) {      // Pass through if SUCCESS (0x00).
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    returnByte = (_read() << 8);
    returnByte |= _read();

    return returnByte;
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, an index byte, and
// a write byte and then then delays 60 microseconds, during which the MAX32664
// retrieves the requested information. An I-squared-C request is then issued,
// and the information is read. This function is very similar to the one above
// except it returns three uint32_t bytes instead of one.
uint8_t Max32664_Hub::_readMultipleBytes(uint8_t familyByte, uint8_t indexByte, uint8_t writeByte,
                                         const size_t numOfReads, int32_t userArray[])
{
    uint8_t statusByte;

    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _write(writeByte);
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _requestFrom(_address, static_cast<uint8_t>(sizeof(int32_t) * numOfReads + sizeof(statusByte)));
    statusByte = _read();
    if (statusByte) { // Pass through if SUCCESS (0x00).
        return statusByte;
    } else {
        for (size_t i = 0; i < (sizeof(int32_t) * numOfReads); i++) {
            userArray[i] = _read() << 24;
            userArray[i] |= _read() << 16;
            userArray[i] |= _read() << 8;
            userArray[i] |= _read();
        }
        return statusByte;
    }
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, an index byte, and
// a write byte and then then delays 60 microseconds, during which the MAX32664
// retrieves the requested information. An I-squared-C request is then issued,
// and the information is read. This function is very similar to the one above
// except it returns three uint32_t bytes instead of one.
uint8_t Max32664_Hub::_readMultipleBytes(uint8_t familyByte, uint8_t indexByte, uint8_t writeByte,
                                         const size_t numOfReads, uint8_t userArray[])
{
    uint8_t statusByte;

    _beginTransmission(_address);
    _write(familyByte);
    _write(indexByte);
    _write(writeByte);
    _endTransmission();
    vTaskDelay(portTICK_PERIOD_MS * CMD_DELAY); // delayMicroseconds(CMD_DELAY * 1000U);

    _requestFrom(_address, static_cast<uint8_t>(numOfReads + sizeof(statusByte)));
    statusByte = _read();
    if (statusByte) { // Pass through if SUCCESS (0x00).
        return statusByte;
    } else {
        for (size_t i = 0; i < numOfReads; i++) {
            userArray[i] = _read();
        }
        return statusByte;
    }
}
