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
    Since the time is short, I transcripted much of the Wire code, so this can
    improved by using the i2c functions even more directly.
    The Wire library uses semaphores everywhere, and I don't know if this is
    really needed here, since the i2c driver may be using semaphores itself.
    These code chunks are disabled by default, but can be enabled by defining
    the macro CONFIG_ENABLE_HAL_LOCKS in the component config.
*/

#ifndef MAX32664_HUB_LIBRARY_H_
#define MAX32664_HUB_LIBRARY_H_

#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

namespace maxim {

#define READBYTE_FAST // use fast read byte function from Kyle Johnson
// I2C Settings
#ifndef I2C_MASTER_FREQ_HZ
    #define I2C_MASTER_FREQ_HZ 100000U // default 100kHz; can be up to 1MHz
#endif
#ifndef I2C_BUFFER_LENGTH
    #define I2C_BUFFER_LENGTH 64
#endif
#ifndef CONFIG_I2C_MAX32664_ADDR
    #define I2C_MAX32664_ADDR 0x55
#else
    #define I2C_MAX32664_ADDR CONFIG_I2C_MAX32664_ADDR
#endif
#ifndef CONFIG_I2C_MASTER_SDA
    #define I2C_MASTER_SDA GPIO_NUM_21
#else
    #define I2C_MASTER_SDA (gpio_num_t)(CONFIG_I2C_MASTER_SDA)
#endif
#ifndef CONFIG_I2C_MASTER_SCL
    #define I2C_MASTER_SCL GPIO_NUM_22
#else
    #define I2C_MASTER_SCL (gpio_num_t)(CONFIG_I2C_MASTER_SCL)
#endif
#ifndef CONFIG_I2C_MASTER_TIMEOUT_MS
    #define I2C_MASTER_TIMEOUT_MS 1000
#else
    #define I2C_MASTER_TIMEOUT_MS CONFIG_I2C_MASTER_TIMEOUT_MS
#endif
#ifndef CONFIG_I2C_MAX32664_RESET_PIN
    #define I2C_MAX32664_RESET_PIN GPIO_NUM_19
#else
    #define I2C_MAX32664_RESET_PIN (gpio_num_t)(CONFIG_I2C_MAX32664_RESET_PIN)
#endif
#ifndef CONFIG_I2C_MAX32664_MFIO_PIN
    #define I2C_MAX32664_MFIO_PIN GPIO_NUM_18
#else
    #define I2C_MAX32664_MFIO_PIN (gpio_num_t)(CONFIG_I2C_MAX32664_MFIO_PIN)
#endif
#ifndef CONFIG_I2C_MAX32664_PULSE_WIDTH
    #define I2C_MAX32664_PULSE_WIDTH (uint16_t)(411)
#else
    #define I2C_MAX32664_PULSE_WIDTH (uint16_t)(CONFIG_I2C_MAX32664_PULSE_WIDTH)
#endif
#ifndef CONFIG_I2C_MAX32664_SAMPLE_RATE
    #define I2C_MAX32664_SAMPLE_RATE (uint16_t)(200)
#else
    #define I2C_MAX32664_SAMPLE_RATE (uint16_t)(CONFIG_I2C_MAX32664_SAMPLE_RATE)
#endif

// general GPIO defines for clarity
#define GPIO_HIGH 1
#define GPIO_LOW 0

// other defines
#define WRITE_FIFO_INPUT_BYTE 0x04
#define DISABLE 0x00
#define ENABLE 0x01
#define MODE_ONE 0x01
#define MODE_TWO 0x02
#define APP_MODE 0x00
#define BOOTLOADER_MODE 0x08
#define NO_WRITE 0x00
#define INCORR_PARAM 0xEE

#define CONFIGURATION_REGISTER 0x0A
#define PULSE_MASK 0xFC
#define READ_PULSE_MASK 0x03
#define SAMP_MASK 0xE3
#define READ_SAMP_MASK 0x1C
#define ADC_MASK 0x9F
#define READ_ADC_MASK 0x60

#define ENABLE_CMD_DELAY 45  // Milliseconds
#define CMD_DELAY 6          // Milliseconds
#define MAXFAST_ARRAY_SIZE 6 // Number of bytes....
#define MAXFAST_EXTENDED_DATA 5
#define MAX30101_LED_ARRAY 12 // 4 values of 24 bit (3 byte) LED values

#define SET_FORMAT 0x00
#define READ_FORMAT 0x01         // Index Byte under Family Byte: READ_OUTPUT_MODE (0x11)
#define WRITE_SET_THRESHOLD 0x01 // Index Byte for WRITE_INPUT(0x14)
#define WRITE_EXTERNAL_TO_FIFO 0x00

const uint8_t BIO_ADDRESS = I2C_MAX32664_ADDR;

struct BioData {
    uint32_t ir_led;
    uint32_t red_led;
    uint32_t green_led;
    uint16_t heart_rate;  // LSB = 0.1bpm
    uint8_t confidence;   // 0-100% LSB = 1%
    uint16_t oxygen;      // 0-100% LSB = 1%
    uint8_t status;       // 0: Success, 1: Not Ready, 2: Object Detectected, 3: Finger Detected
    float r_value;        // -- Algorithm Mode 2 vv
    int8_t ext_status;    // --
    uint8_t reserve_one;  // --
    uint8_t resserve_two; // -- Algorithm Mode 2 ^^
};

struct Version {
    // 3 bytes total
    uint8_t major;
    uint8_t minor;
    uint8_t revision;
};

struct SensorAttr {
    uint8_t byte_word;
    uint8_t avail_registers;
};

// Status Bytes are communicated back after every I-squared-C transmission and
// are indicators of success or failure of the previous transmission.
typedef enum {
    SUCCESS = 0x00,
    ERR_UNAVAIL_CMD = 0x01,
    ERR_UNAVAIL_FUNC = 0x02,
    ERR_DATA_FORMAT = 0x03,
    ERR_INPUT_VALUE = 0x04,
    ERR_TRY_AGAIN = 0x05,
    ERR_BTLDR_GENERAL = 0x80,
    ERR_BTLDR_CHECKSUM = 0x81,
    ERR_BTLDR_AUTH = 0x82,
    ERR_BTLDR_INVALID_APP = 0x83,
    ERR_UNKNOWN = 0xFF
} read_status_byte_value_t;

// The family register bytes are the larger umbrella for all the Index and
// Write Bytes listed below. You can not reference a nestled byte without first
// referencing it's larger category: Family Register Byte.
typedef enum {
    HUB_STATUS = 0x00,
    SET_DEVICE_MODE = 0x01,
    READ_DEVICE_MODE = 0x02,
    OUTPUT_MODE = 0x10,
    READ_OUTPUT_MODE = 0x11, // not on the datasheet
    READ_DATA_OUTPUT = 0x12,
    READ_DATA_INPUT = 0x13,
    WRITE_INPUT = 0x14, // not on the datasheet
    WRITE_REGISTER = 0x40,
    READ_REGISTER = 0x41,
    READ_ATTRIBUTES_AFE = 0x42,
    DUMP_REGISTERS = 0x43,
    ENABLE_SENSOR = 0x44,
    READ_SENSOR_MODE = 0x45, // not on the datasheet
    CHANGE_ALGORITHM_CONFIG = 0x50,
    READ_ALGORITHM_CONFIG = 0x51,
    ENABLE_ALGORITHM = 0x52,
    BOOTLOADER_FLASH = 0x80,
    BOOTLOADER_INFO = 0x81,
    IDENTITY = 0xFF
} family_register_bytes_t;

// All the defines below are: 1. Index Bytes nestled in the larger category of the
// family registry bytes listed above and 2. The Write Bytes nestled even
// farther under their Index Bytes.

// Write Bytes under Family Byte: SET_DEVICE_MODE (0x01) and Index
// Byte: 0x00.
typedef enum { EXIT_BOOTLOADER = 0x00, RESET = 0x02, ENTER_BOOTLOADER = 0x08 } device_mode_write_bytes_t;

// Write Bytes under Family Byte: OUTPUT_MODE (0x10) and Index byte: SET_FORMAT
// (0x00)
typedef enum {
    PAUSE = 0x00,
    SENSOR_DATA = 0x01,
    ALGO_DATA = 0x02,
    SENSOR_AND_ALGORITHM = 0x03,
    PAUSE_TWO = 0x04,
    SENSOR_COUNTER_BYTE = 0x05,
    ALGO_COUNTER_BYTE = 0x06,
    SENSOR_ALGO_COUNTER = 0x07
} output_mode_write_byte_t;

// Index Byte under the Family Byte: READ_DATA_OUTPUT (0x12)
typedef enum { NUM_SAMPLES = 0x00, READ_DATA = 0x01 } fifo_output_index_byte_t;

// Index Byte under the Family Byte: READ_DATA_INPUT (0x13)
typedef enum {
    SAMPLE_SIZE = 0x00,
    READ_INPUT_MAX_SIZE = 0x01,
    READ_SENSOR_MAX_SIZE = 0x02,   // For external accelerometer
    READ_NUM_SAMPLES_INPUT = 0x03, // For external accelerometer
    READ_NUM_SAMPLES_SENSOR = 0x04
} fifo_external_index_byte_t;

// Index Byte under the Family Registry Byte: WRITE_REGISTER (0x40)
typedef enum {
    WRITE_MAX86140 = 0x00,
    WRITE_MAX30205 = 0x01,
    WRITE_MAX30001 = 0x02,
    WRITE_MAX30101 = 0x03,
    WRITE_ACCELEROMETER = 0x04
} write_register_index_byte_t;

// Index Byte under the Family Registry Byte: READ_REGISTER (0x41)
typedef enum {
    READ_MAX86140 = 0x00,
    READ_MAX30205 = 0x01,
    READ_MAX30001 = 0x02,
    READ_MAX30101 = 0x03,
    READ_ACCELEROMETER = 0x04
} read_register_index_byte_t;

// Index Byte under the Family Registry Byte: READ_ATTRIBUTES_AFE (0x42)
typedef enum {
    RETRIEVE_AFE_MAX86140 = 0x00,
    RETRIEVE_AFE_MAX30205 = 0x01,
    RETRIEVE_AFE_MAX30001 = 0x02,
    RETRIEVE_AFE_MAX30101 = 0x03,
    RETRIEVE_AFE_ACCELEROMETER = 0x04
} get_afe_index_byte_t;

// Index Byte under the Family Byte: DUMP_REGISTERS (0x43)
typedef enum {

    DUMP_REGISTER_MAX86140 = 0x00,
    DUMP_REGISTER_MAX30205 = 0x01,
    DUMP_REGISTER_MAX30001 = 0x02,
    DUMP_REGISTER_MAX30101 = 0x03,
    DUMP_REGISTER_ACCELEROMETER = 0x04

} dump_register_index_byte_t;

// Index Byte under the Family Byte: ENABLE_SENSOR (0x44)
typedef enum {
    ENABLE_MAX86140 = 0x00,
    ENABLE_MAX30205 = 0x01,
    ENABLE_MAX30001 = 0x02,
    ENABLE_MAX30101 = 0x03,
    ENABLE_ACCELEROMETER = 0x04
} sensor_enable_index_byte_t;

// Index Byte for the Family Byte: READ_SENSOR_MODE (0x45)
// not on the datasheet
typedef enum {
    READ_ENABLE_MAX86140 = 0x00,
    READ_ENABLE_MAX30205 = 0x01,
    READ_ENABLE_MAX30001 = 0x02,
    READ_ENABLE_MAX30101 = 0x03,
    READ_ENABLE_ACCELEROMETER = 0x04
} read_sensor_enable_index_byte_t;

// Index Byte under the Family Byte: CHANGE_ALGORITHM_CONFIG (0x50)
typedef enum {
    SET_TARG_PERC = 0x00,
    SET_STEP_SIZE = 0x00,
    SET_SENSITIVITY = 0x00,
    SET_AVG_SAMPLES = 0x00,
    WHRM_CONFIG = 0x02,
    BPT_CONFIG = 0x04,
    WSPO2_CONFIG = 0x05
} algorithm_config_index_byte_t;

// Write Bytes under the Family Byte: CHANGE_ALGORITHM_CONFIG (0x50) and the
// Index Byte: ALGORITHM_CONFIG_INDEX_BYTE - SET_TARG_PERC
typedef enum {
    AGC_GAIN_ID = 0x00,
    AGC_STEP_SIZE_ID = 0x01,
    AGC_SENSITIVITY_ID = 0x02,
    AGC_NUM_SAMP_ID = 0x03,
    MAXIMFAST_COEF_ID = 0x0B
} algo_agc_write_byte_t;

typedef enum {
    BPT_MEDICATION = 0x00,
    DIASTOLIC_VALUE = 0x01,
    SYSTOLIC_VALUE = 0x02,
    BPT_CALIB_DATA = 0x03, // Index + 824 bytes of calibration data
    SET_EST_DATA = 0x04,
    PATIENT_RESTING = 0x05,
    BPT_SP02_COEFS = 0x06
} algo_bpt_write_byte_t;

// Index Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51)
typedef enum {
    READ_AGC_PERCENTAGE = 0x00,
    READ_AGC_STEP_SIZE = 0x00,
    READ_AGC_SENSITIVITY = 0x00,
    READ_AGC_NUM_SAMPLES = 0x00,
    READ_MAX_FAST_COEF = 0x02
} read_algorithm_index_byte_t;

// Write Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51) and Index Byte:
// READ_ALGORITHM_INDEX_BYTE - AGC
typedef enum {
    READ_AGC_PERC_ID = 0x00,
    READ_AGC_STEP_SIZE_ID = 0x01,
    READ_AGC_SENSITIVITY_ID = 0x02,
    READ_AGC_NUM_SAMPLES_ID = 0x03,
    READ_MAX_FAST_COEF_ID = 0x0B
} read_agc_algo_write_byte_t;

// Index Byte under the Family Byte: ENABLE_ALGORITHM (0x52).
typedef enum {
    ENABLE_AGC_ALGO = 0x00,
    ENABLE_AEC_ALGO = 0x01,
    ENABLE_WHRM_ALGO = 0x02,
    ENABLE_ECG_ALGO = 0x03,
    ENABLE_BPT_ALGO = 0x04,
    ENABLE_WSPO2_ALGO = 0x05
} algorithm_mode_enable_index_byte_t;

// Index Byte under the Family Byte: BOOTLOADER_FLASH (0x80).
typedef enum {
    SET_INIT_VECTOR_BYTES = 0x00,
    SET_AUTH_BYTES = 0x01,
    SET_NUM_PAGES = 0x02,
    ERASE_FLASH = 0x03,
    SEND_PAGE_VALUE = 0x04
} bootloader_flash_index_byte_t;

// Index Byte under the Family Byte: BOOTLOADER_INFO (0x81).
typedef enum { BOOTLOADER_VERS = 0x00, PAGE_SIZE = 0x01 } bootloader_info_index_byte_t;

// Index Byte under the Family Byte: IDENTITY (0xFF).
typedef enum { READ_MCU_TYPE = 0x00, READ_SENSOR_HUB_VERS = 0x03, READ_ALGO_VERS = 0x07 } identity_index_bytes_t;

typedef enum { MAX32625 = 0x00, MAX32664 = 0x01 } mcutype_t;

class Max32664Hub {
public:
    // Constructor ----------
    Max32664Hub(gpio_num_t reset_pin = GPIO_NUM_NC, gpio_num_t mfio_pin = GPIO_NUM_NC, uint8_t address = BIO_ADDRESS);

    // Destructor ----------
    ~Max32664Hub();

    // Functions ------------

    // I2C init
    esp_err_t i2c_bus_init(gpio_num_t sda = GPIO_NUM_NC, gpio_num_t scl = GPIO_NUM_NC);
    // ~I2C init

    // Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
    // The following function initializes the sensor. To place the MAX32664 into
    // application mode, the MFIO pin must be pulled HIGH while the board is held
    // in reset for 10ms. After 50 addtional ms have elapsed the board should be
    // in application mode and will return two bytes, the first 0x00 is a
    // successful communcation byte, followed by 0x00 which is the byte indicating
    // which mode the IC is in.
    uint8_t begin(gpio_num_t reset_pin = GPIO_NUM_NC, gpio_num_t mfio_pin = GPIO_NUM_NC);

    // Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
    // The following function puts the MAX32664 into bootloader mode. To place the MAX32664 into
    // bootloader mode, the MFIO pin must be pulled LOW while the board is held
    // in reset for 10ms. After 50 addtional ms have elapsed the board should be
    // in bootloader mode and will return two bytes, the first 0x00 is a
    // successful communcation byte, followed by 0x08 which is the byte indicating
    // that the board is in bootloader mode.
    uint8_t begin_bootloader(gpio_num_t reset_pin = GPIO_NUM_NC, gpio_num_t mfio_pin = GPIO_NUM_NC);

    // Family Byte: HUB_STATUS (0x00), Index Byte: 0x00, No Write Byte.
    // The following function checks the status of the FIFO.
    uint8_t read_sensor_hub_status();

    // Family Byte: SET_DEVICE_MODE (0x01), Index Byte: 0x01, Write Byte: 0x00
    // The following function is an alternate way to set the mode of the of
    // MAX32664. It can take three parameters: Enter and Exit Bootloader Mode, as
    // well as reset.
    // INCOMPLETE
    uint8_t set_operating_mode(uint8_t);

    // This function sets very basic settings to get sensor and biometric data.
    // The biometric data includes data about heartrate, the confidence
    // level, SpO2 levels, and whether the sensor has detected a finger or not.
    uint8_t config_bpm(uint8_t);

    // This function sets very basic settings to get LED count values from the MAX30101.
    // Sensor data includes 24 bit LED values for the three LED channels: Red, IR,
    // and Green.
    uint8_t config_sensor();

    // This function sets very basic settings to get sensor and biometric data.
    // Sensor data includes 24 bit LED values for the two LED channels: Red and IR.
    // The biometric data includes data about heartrate, the confidence
    // level, SpO2 levels, and whether the sensor has detected a finger or not.
    // Of note, the number of samples is set to one.
    uint8_t config_sensor_bpm(uint8_t);

    // This function takes the 8 bytes from the FIFO buffer related to the wrist
    // heart rate algortihm: heart rate (uint16_t), confidence (uint8_t) , SpO2 (uint16_t),
    // and the finger detected status (uint8_t). Note that the the algorithm is stated as
    // "wrist" though the sensor only works with the finger. The data is loaded
    // into the whrmFifo and returned.
    BioData read_bpm();

    // This function takes 9 bytes of LED values from the MAX30101 associated with
    // the RED, IR, and GREEN LEDs. In addition it gets the 8 bytes from the FIFO buffer
    // related to the wrist heart rate algortihm: heart rate (uint16_t), confidence (uint8_t),
    // SpO2 (uint16_t), and the finger detected status (uint8_t). Note that the the algorithm
    // is stated as "wrist" though the sensor only works with the finger. The data is loaded
    // into the whrmFifo and returned.
    BioData read_sensor();

    // This function takes the information of both the LED value and the biometric
    // data from the MAX32664's FIFO. In essence it combines the two functions
    // above into a single function call.
    BioData read_sensor_bpm();

    // This function modifies the pulse width of the MAX30101 LEDs. All of the LEDs
    // are modified to the same width. This will affect the number of samples that
    // can be collected and will also affect the ADC resolution.
    // Width(us) - Resolution -  Sample Rate
    // Default: 69us - 15 resolution - 50 samples per second.
    //  69us     -    15      -   <= 3200 (fastest - least resolution)
    //  118us    -    16      -   <= 1600
    //  215us    -    17      -   <= 1600
    //  411us    -    18      -   <= 1000 (slowest - highest resolution)
    uint8_t set_pulse_width(uint16_t);

    // This function reads the CONFIGURATION_REGISTER (0x0A), bits [1:0] from the
    // MAX30101 Sensor. It returns one of the four settings in microseconds.
    uint16_t read_pulse_width();

    // This function changes the sample rate of the MAX30101 sensor. The sample
    // rate is affected by the set pulse width of the MAX30101 LEDs.
    // Default: 69us - 15 resolution - 50 samples per second.
    // Width(us) - Resolution -  Sample Rate
    //  69us     -    15      -   <= 3200 (fastest - least resolution)
    //  118us    -    16      -   <= 1600
    //  215us    -    17      -   <= 1600
    //  411us    -    18      -   <= 1000 (slowest - highest resolution)
    //  Samples Options:
    //  50, 100, 200, 400, 800, 1000, 1600, 3200
    uint8_t set_sample_rate(uint16_t);

    // This function reads the CONFIGURATION_REGISTER (0x0A), bits [4:2] from the
    // MAX30101 Sensor. It returns one of the 8 possible sample rates.
    uint16_t read_sample_rate();

    // MAX30101 Register: CONFIGURATION_REGISTER (0x0A), bits [6:5]
    // This functions sets the dynamic range of the MAX30101's ADC. The function
    // accepts the higher range as a parameter.
    // Default Range: 7.81pA - 2048nA
    // Possible Ranges:
    // 7.81pA  - 2048nA
    // 15.63pA - 4096nA
    // 32.25pA - 8192nA
    // 62.5pA  - 16384nA
    uint8_t set_adc_range(uint16_t);

    // MAX30101 Register: CONFIGURATION_REGISTER (0x0A), bits [6:5]
    // This function returns the set ADC range of the MAX30101 sensor.
    uint16_t read_adc_range();

    // Family Byte: IDENTITY (0x01), Index Byte: READ_MCU_TYPE, Write Byte: NONE
    // The following function returns a byte that signifies the microcontoller that
    // is in communcation with your host microcontroller. Returns 0x00 for the
    // MAX32625 and 0x01 for the MAX32660/MAX32664.
    uint8_t get_mcu_type();

    // Family Byte: BOOTLOADER_INFO (0x80), Index Byte: BOOTLOADER_VERS (0x00)
    // This function checks the version number of the bootloader on the chip and
    // returns a four bytes: Major version Byte, Minor version Byte, Space Byte,
    // and the Revision Byte.
    int32_t get_bootloader_inf();

    // Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30101 (0x03), Write
    // Byte: senSwitch (parameter - 0x00 or 0x01).
    // This function enables the MAX30101.
    uint8_t max30101_control(uint8_t);

    // Family Byte: READ_SENSOR_MODE (0x45), Index Byte: READ_ENABLE_MAX30101 (0x03)
    // This function checks if the MAX30101 is enabled or not.
    uint8_t read_max30101_state();

    // Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_ACCELEROMETER (0x04), Write
    // Byte: accelSwitch (parameter - 0x00 or 0x01).
    // This function enables the ACCELEROMETER.
    uint8_t accel_control(uint8_t);

    // Family Byte: OUTPUT_MODE (0x10), Index Byte: SET_FORMAT (0x00),
    // Write Byte : outputType (Parameter values in OUTPUT_MODE_WRITE_BYTE)
    uint8_t set_output_mode(uint8_t);

    // Family Byte: OUTPUT_MODE, Index Byte: WRITE_SET_THRESHOLD, Write byte: intThres
    // (parameter - value betwen 0 and 0xFF).
    // This function changes the threshold for the FIFO interrupt bit/pin. The
    // interrupt pin is the MFIO pin which is set to INPUT after IC initialization
    // (begin).
    uint8_t set_fifo_threshold(uint8_t);

    // Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: NUM_SAMPLES (0x00), Write
    // Byte: NONE
    // This function returns the number of samples available in the FIFO.
    uint8_t num_samples_out_fifo();

    // Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_DATA (0x00), Write
    // Byte: NONE
    // This function returns the data in the FIFO.
    uint8_t *get_data_out_fifo(uint8_t data[]);

    // Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_DATA (0x00), Write
    // Byte: NONE
    // This function adds support for the acceleromter that is NOT included on
    // SparkFun's product, The Family Registery of 0x13 and 0x14 is skipped for now.
    uint8_t num_samples_external_sensor();

    // Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX30101 (0x03), Write Bytes:
    // Register Address and Register Value
    // This function writes the given register value at the given register address
    // for the MAX30101 sensor and returns a boolean indicating a successful or
    // non-successful write.
    void write_register_max30101(uint8_t, uint8_t);

    // Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_ACCELEROMETER (0x04), Write Bytes:
    // Register Address and Register Value
    // This function writes the given register value at the given register address
    // for the Accelerometer and returns a boolean indicating a successful or
    // non-successful write.
    void write_register_accel(uint8_t, uint8_t);

    // Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30101 (0x03), Write Byte:
    // Register Address
    // This function reads the given register address for the MAX30101 Sensor and
    // returns the values at that register.
    uint8_t read_register_max30101(uint8_t);

    // Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30101 (0x03), Write Byte:
    // Register Address
    // This function reads the given register address for the MAX30101 Sensor and
    // returns the values at that register.
    uint8_t read_register_accel(uint8_t);

    // Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX30101/ (0x03)
    // This function retrieves the attributes of the AFE (Analog Front End) of the
    // MAX30101 sensor. It returns the number of bytes in a word for the sensor
    // and the number of registers available.
    SensorAttr get_afe_attributes_max30101();

    // Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte:
    // RETRIEVE_AFE_ACCELEROMETER (0x04)
    // This function retrieves the attributes of the AFE (Analog Front End) of the
    // Accelerometer. It returns the number of bytes in a word for the sensor
    // and the number of registers available.
    SensorAttr get_afe_attributes_accelerometer();

    // Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX30101 (0x03)
    // This function returns all registers and register values sequentially of the
    // MAX30101 sensor: register zero and register value zero to register n and
    // register value n. There are 36 registers in this case.
    uint8_t dump_register_max30101(uint8_t reg_array[]);

    // Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_ACCELEROMETER (0x04)
    // This function returns all registers and register values sequentially of the
    // Accelerometer: register zero and register value zero to register n and
    // register value n.
    uint8_t dump_register_accelerometer(uint8_t, uint8_t reg_array[]);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
    // SET_TARG_PERC (0x00), Write Byte: AGC_GAIN_ID (0x00)
    // This function sets the target percentage of the full-scale ADC range that
    // the automatic gain control algorithm uses. It takes a paramater of zero to
    // 100 percent.
    uint8_t set_algo_range(uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
    // SET_STEP_SIZE (0x00), Write Byte: AGC_STEP_SIZE_ID (0x01)
    // This function changes the step size toward the target for the AGC algorithm.
    // It takes a paramater of zero to 100 percent.
    uint8_t set_algo_step_size(uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
    // SET_SENSITIVITY (0x00), Write Byte: AGC_SENSITIVITY_ID (0x02)
    // This function changes the sensitivity of the AGC algorithm.
    uint8_t set_algo_sensitivity(uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
    // SET_AVG_SAMPLES (0x00), Write Byte: AGC_NUM_SAMP_ID (0x03)
    // This function changes the number of samples that are averaged.
    // It takes a paramater of zero to 255.
    uint8_t set_algo_samples(uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
    // WHRM_CONFIG (0x02), Write Byte: MAXIMFAST_COEF_ID (0x0B)
    // This function takes three values that are used as the Sp02 coefficients.
    uint8_t set_maxim_fast_coef(int32_t, int32_t, int32_t);

    // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
    // READ_AGC_PERCENTAGE (0x00), Write Byte: READ_AGC_PERC_ID (0x00)
    // This function reads and returns the currently set target percentage
    // of the full-scale ADC range that the Automatic Gain Control algorithm is using.
    uint8_t read_algo_range();

    // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
    // READ_AGC_STEP_SIZE (0x00), Write Byte: READ_AGC_STEP_SIZE_ID (0x01)
    // This function returns the step size toward the target for the AGC algorithm.
    // It returns a value between zero and 100 percent.
    uint8_t read_algo_step_size();

    // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
    // READ_AGC_SENSITIVITY_ID (0x00), Write Byte: READ_AGC_SENSITIVITY_ID (0x02)
    // This function returns the sensitivity (percentage) of the automatic gain control.
    uint8_t read_algo_sensitivity();

    // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
    // READ_AGC_NUM_SAMPLES (0x00), Write Byte: READ_AGC_NUM_SAMPlES_ID (0x03)
    // This function changes the number of samples that are averaged.
    // It takes a paramater of zero to 255.
    uint8_t read_algo_samples();

    // Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
    // READ_MAX_FAST_COEF (0x02), Write Byte: READ_MAX_FAST_COEF_ID (0x0B)
    // This function reads the maximum age for the wrist heart rate monitor
    // (WHRM) algorithm. It returns three uint32_t integers that are
    // multiplied by 100,000.
    // INCOMPLETE
    uint8_t read_maxim_fast_coef(int32_t coef_arr[3]);

    // Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
    // ENABLE_AGC_ALGO (0x00)
    // This function enables (one) or disables (zero) the automatic gain control algorithm.
    uint8_t agc_algo_control(uint8_t);

    // Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
    // ENABLE_WHRM_ALGO (0x02)
    // This function enables (one) or disables (zero) the wrist heart rate monitor
    // algorithm.
    uint8_t maxim_fast_algo_control(uint8_t);

    // Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_NUM_PAGES (0x02),
    // Write Bytes: 0x00 - Number of pages at byte 0x44 from .msbl file.
    bool set_num_pages(uint8_t);

    // Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: ERASE_FLASH (0x03)
    // Returns true on successful communication.
    bool erase_flash();

    // Family Byte: BOOTLOADER_INFO (0x81), Index Byte: BOOTLOADER_VERS (0x00)
    Version read_bootloader_vers();

    // Family Byte: BOOTLOADER_INFO (0x81), Index Byte: PAGE_SIZE (0x01)
    // Family Byte: IDENTITY (0xFF), Index Byte: READ_SENSOR_HUB_VERS (0x03)
    Version read_sensor_hub_version();

    // Family Byte: IDENTITY (0xFF), Index Byte: READ_ALGO_VERS (0x07)
    Version read_algorithm_version();

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: BPT_MEDICATION (0x00)
    uint8_t is_patient_bp_medication(uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: BPT_MEDICATION (0x00)
    uint8_t is_patient_bp_medication();

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: DIASTOLIC_VALUE (0x02)
    uint8_t write_diastolic_vals(uint8_t, uint8_t, uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: DIASTOLIC_VALUE (0x02)
    uint8_t read_diastolic_vals(uint8_t user_array[]);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: SYSTOLIC_VALUE (0x01)
    uint8_t write_systolic_vals(uint8_t, uint8_t, uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: SYSTOLIC_VALUE (0x01)
    uint8_t read_systolic_vals(uint8_t user_array[]);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: BPT_CALIB_DATA (0x03)
    uint8_t write_bpt_algo_data(const uint8_t bpt_calib_data[]);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: BPT_CALIB_DATA (0x03)
    uint8_t read_bpt_algo_data(uint8_t user_array[]);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: PATIENT_RESTING (0x05)
    uint8_t is_patient_resting(uint8_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: PATIENT_RESTING (0x05)
    uint8_t is_patient_resting();

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: AGC_SP02_COEFS (0x0B)
    uint8_t write_spo2_algo_coef(int32_t, int32_t, int32_t);

    // Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
    // Write Byte: AGC_SP02_COEFS (0x0B)
    uint8_t read_spo2_algo_coef(int32_t user_array[]);

    // Variables ------------
    uint8_t bpm_arr[MAXFAST_ARRAY_SIZE] {};
    uint8_t bpm_arr_two[MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA] {};
    uint8_t sen_arr[MAX30101_LED_ARRAY] {};
    uint8_t bpm_sen_arr[MAXFAST_ARRAY_SIZE + MAX30101_LED_ARRAY] {};
    uint8_t bpm_sen_arr_two[MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA + MAX30101_LED_ARRAY] {};

private:
    // I2C Functions------------

    bool _allocate_wire_buffer();
    void _free_wire_buffer();

    void _begin_transmission(uint16_t address);
    void _begin_transmission(uint8_t address);
    void _begin_transmission(int16_t address);

    uint8_t _end_transmission(bool send_stop);
    uint8_t _end_transmission();

    size_t _request_from(uint16_t address, size_t size);
    size_t _request_from(uint8_t address, size_t len);
    uint8_t _request_from(uint8_t address, uint8_t len);
    uint8_t _request_from(uint16_t address, uint8_t len);
    uint8_t _request_from(int16_t address, int16_t len);

    size_t _write(uint8_t);
    size_t _write(const uint8_t *, size_t);

    // BSP implementation of strlen
    // I just want to avoid using the string library
    size_t _strlen(const int8_t *str)
    {
        const int8_t *s;
        for (s = str; *s; ++s) {
            ;
        }
        return (s - str);
    }

    inline size_t _write(const int8_t *s) { return _write((const uint8_t *)s, _strlen(s)); }

    int16_t _read();

    // ~I2C Functions------------

    // This function uses the given family, index, and write byte to enable
    // the given sensor.
    uint8_t _enable_write(uint8_t, uint8_t, uint8_t);

    // This function uses the given family, index, and write byte to communicate
    // with the MAX32664 which in turn communicates with downward sensors. There
    // are two steps demonstrated in this function. First a write to the MCU
    // indicating what you want to do, a delay, and then a read to confirm positive
    // transmission.
    uint8_t _write_byte(uint8_t, uint8_t, uint8_t);

    // This function sends is simliar to the one above and sends info to the MAX32664
    // but takes an additional uint8_t as a paramter. Again there is the write
    // of the specific bytes followed by a read to confirm positive transmission.
    uint8_t _write_byte(uint8_t, uint8_t, uint8_t, uint8_t);

    // This function is the same as the function above and uses the given family,
    // index, and write byte, but also takes a 16 bit integer as a paramter to communicate
    // with the MAX32664 which in turn communicates with downward sensors. There
    // are two steps demonstrated in this function. First a write to the MCU
    // indicating what you want to do, a delay, and then a read to confirm positive
    // transmission.
    uint8_t _write_byte(uint8_t, uint8_t, uint8_t, uint16_t);

    // This function sends information to the MAX32664 to specifically write values
    // to the registers of downward sensors and so also requires a
    // register address and register value as parameters. Again there is the write
    // of the specific bytes followed by a read to confirm positive transmission.
    uint8_t _write_long_bytes(uint8_t, uint8_t, uint8_t, const int32_t write_val[], size_t);

    // This function sends information to the MAX32664 to specifically write values
    // to the registers of downward sensors and so also requires a
    // register address and register value as parameters. Again there is the write
    // of the specific bytes followed by a read to confirm positive transmission.
    uint8_t _write_bytes(uint8_t, uint8_t, uint8_t, const uint8_t write_val[], size_t);

    // This function handles all read commands or stated another way, all information
    // requests. It starts a request by writing the family byte, index byte, and
    // delays 60 microseconds, during which the MAX32664 retrieves the requested
    // information. An I-squared-C request is then issued, and the information is read and returned.
    uint8_t _read_byte(uint8_t, uint8_t);

    // Same as _readByte() but without the delay
    uint8_t _read_byte_fast(uint8_t family_byte, uint8_t index_byte);
    // Same as _readByte() but without the delay
    uint8_t _read_byte_fast(uint8_t family_byte, uint8_t index_byte, uint8_t write_byte);

    // This function is exactly as the one above except it accepts a Write Byte as
    // a paramter. It starts a request by writing the family byte, index byte, and
    // write byte to the MAX32664, delays 60 microseconds, during which
    // the MAX32664 retrieves the requested information. A I-squared-C request is
    // then issued, and the information is read and returned.
    uint8_t _read_byte(uint8_t, uint8_t, uint8_t);

    // This function handles all read commands or stated another way, all information
    // requests. It starts a request by writing the family byte, an index byte, and
    // a write byte and then then delays 60 microseconds, during which the MAX32664
    // retrieves the requested information. An I-squared-C request is then issued,
    // and the information is read. This differs from the above read commands in
    // that it returns a 16 bit integer instead of 8.
    uint16_t _read_int_byte(uint8_t, uint8_t, uint8_t);

    // This function handles all read commands or stated another way, all information
    // requests. It starts a request by writing the family byte, an index byte, and
    // a write byte and then then delays 60 microseconds, during which the MAX32664
    // retrieves the requested information. An I-squared-C request is then issued,
    // and the information is read. This function is very similar to the one above
    // except it returns three uint32_t bytes instead of one.
    uint8_t _read_multiple_bytes(uint8_t, uint8_t, uint8_t, size_t, int32_t user_array[]);

    // This function handles all read commands or stated another way, all information
    // requests. It starts a request by writing the family byte, an index byte, and
    // a write byte and then then delays 60 microseconds, during which the MAX32664
    // retrieves the requested information. An I-squared-C request is then issued,
    // and the information is read. This function is very similar to the one above
    // except it returns multiple requested bytes.
    uint8_t _read_multiple_bytes(uint8_t, uint8_t, uint8_t, size_t, uint8_t user_array[]);

    // This functions is similar to _readMultipleBytes() except it doesn't send a write byte
    // and all the information is read at once and returned in a uint8_t array by reference.
    uint8_t _read_fill_array(uint8_t, uint8_t, uint8_t, uint8_t array[]);

    // Variables -----------
    gpio_num_t _reset_pin = (gpio_num_t)I2C_MAX32664_RESET_PIN;
    gpio_num_t _mfio_pin = (gpio_num_t)I2C_MAX32664_MFIO_PIN;
    uint8_t _address = I2C_MAX32664_ADDR;
    // uint32_t _writeCoefArr[3]{};
    uint8_t _user_selected_mode;
    uint8_t _sample_rate = 0; // AGC_NUM_SAMPLES average (0-255)
    bool _i2c_started = false;

    size_t _buffer_size = I2C_BUFFER_LENGTH;
    uint8_t *_rx_buffer = nullptr;
    size_t _rx_index = 0;
    size_t _rx_length = 0;

    uint8_t *_tx_buffer = nullptr;
    size_t _tx_length = 0;
    uint16_t _tx_address = 0;

    uint32_t _time_out_millis =
        (uint32_t)((float)(I2C_MASTER_TIMEOUT_MS) / (float)(portTICK_PERIOD_MS)); // default in i2c is 50ms
    bool _non_stop = false;

#if defined(CONFIG_ENABLE_HAL_LOCKS)
    TaskHandle_t _nonStopTask = nullptr;
    SemaphoreHandle_t _lock = nullptr;
#endif
};

} // namespace maxim

#ifdef __cplusplus
}
#endif

#endif // MAX32664_HUB_LIBRARY_H_
