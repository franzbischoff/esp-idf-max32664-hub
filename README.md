# SparkFun Pulse Oximeter and Heart Rate Sensor component for ESP-IDF

This is a modified version of the original from [Sparkfun](https://github.com/sparkfun/SparkFun_Bio_Sensor_Hub_Library)

Use it at your own risk.

---

![SparkFun Pulse Oximeter and Heart Rate Monitor](https://cdn.sparkfun.com/assets/parts/1/3/6/6/4/15219-SparkFun_Pulse_Oximeter_and_Heart_Rate_Sensor_-_MAX30101__Qwiic_-01.jpg)

[_SparkFun Pulse Oximeter and Heart Rate Monitor (SEN-15291)_](https://www.sparkfun.com/products/15219)

The sensor is an I&sup2;C based [biometric](https://en.wikipedia.org/wiki/Biometrics) sensor, utilizing two chips from
Maxim Integrated: the MAX32664 Bio Metric Sensor Hub and the MAX30101 Pulse Oximetry and Heart-Rate Module.

## Documentation

- **[Hookup Guide](https://learn.sparkfun.com/tutorials/sparkfun-pulse-oximeter-and-heart-rate-monitor-hookup-guide)** -
  Basic hookup guide for the SparkFun Pulse Oximeter and Heart Rate Sensor.

## Products that use this library

- [SEN-15291](https://www.sparkfun.com/products/15219)- SparkFun Version 1.0

## License Information

This product is **open source**!

Various bits of the code have different licenses applied. Anything SparkFun wrote is beerware; if you see me (or any
other SparkFun employee) at the local and you've found our code helpful, please buy us a round!

Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and
release anything derivative under the same license.

Distributed as-is; no warranty is given.

## How to Use

1. This component is not available (yet) in the ESP-IDF component registry. So you need to clone this repository into
   your project's `components` folder.

2. As a submodule: In your project, add this as a submodule to your `components/` directory.

```bash
git submodule add https://github.com/franzbischoff/esp-idf-max32664-hub
git submodule update --init --recursive
```

The library can be configured via `idf.py menuconfig` under `Component config->MAX32664 I2C config`.

### Example

One of the original examples was modified and is in the `example/` directory.

## Tips, Tricks, and Gotchas

This code was mainly rewritten to use the i2c driver directly from ESP-IDF. The original code was using the Wire library
from Arduino. Since the time is short, I transcripted much of the Wire code, so this code can be further improved by
using the i2c functions even more directly (contributions are welcome).

The Wire library uses semaphores everywhere, and I do not know if this is really needed here since the i2c driver may be
using semaphores itself. These code chunks are disabled by default but can be enabled by defining the macro
`CONFIG_ENABLE_HAL_LOCKS` in the component config.

The example contained in this repository is minimal. It is advised to check the examples from the original Sparkfun's
repository. You may want to tweak the `Pulse Width` and `Sampling Rate` for your application, and different values may
require different delay times in the loop process.

The initial readings may be preceded by a few warnings from the I2C driver. You can ignore them safely.

Wait for the sensor to start reading, and then you can try with your finger on the sensor. Be patient, as it may take a
few (lot) seconds to get the first readings. Try switching fingers if it takes too long, and don't press too hard.

If you get lots of I2C timeout errors, check the power supply. You may need to use a separate 3.3V power supply for the
sensor.

Finally, check if `CONFIG_FREERTOS_HZ` is set to 1000 in menuconfig. If not, set it to 1000!
