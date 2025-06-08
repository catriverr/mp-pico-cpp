#pragma once
#include <string>

namespace pico_util {
    /// Automatically initializes a Pico without
    /// specifying which device is used for the
    /// serial path in /dev/usbmodem.*.
    ///
    /// Picos must have the MicroPython UF2 to
    /// work with this library.
    bool auto_init();
    /// Manual initialization of serial path
    /// towards a MicroPython Pico.
    bool init(int fd);
    /// Evaluates operation in MicroPython
    /// on the Pico and returns the value
    /// as a string.
    std::string eval(const std::string& python);

    /// returns the Analog-Digital Converter's
    /// voltage status as a uint16 - 0 -> 65535.
    uint16_t ADC(int pin);
    /// returns the core temp of the chip.
    float core_temp();

    enum class GPIOState { LOW, HIGH };
    /// Reads the state of a GPIO pin,
    /// returns LOW for 0V and HIGH for 3.3V.
    GPIOState GPIO_read(int pin);
    /// Sets the state of a GPIO Pin, varying
    /// from 0V  to 3.3V.
    void GPIO_write(int pin, float value);

    struct PWMSettings {
        int freq;
        float duty; // 0.0 - 1.0
    };
    /// Mock value: returns 1000, 0.5f.
    PWMSettings PWM(int pin);
    /// Sets the Frequency and duty of a PWM
    /// on the specified Pin.
    void PWM_set(int pin, uint16_t duty, int freq);
}
