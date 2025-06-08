#include "./pico.cc"

using namespace pico_util;

int main() {
    if (!pico_util::auto_init()) {
        std::cerr << "❌ Failed to connect to a MicroPython-enabled Pico.\n";
        return 1;
    }

    std::cout << "✅ Connected to Pico via MicroPython REPL.\n";

    auto d = std::chrono::system_clock::now();
    auto last = d.time_since_epoch();
    bool a = false;

    for (;;) {
        if ( (std::chrono::system_clock::now().time_since_epoch()) - last > std::chrono::milliseconds(125) ) {
            a = !a;
            GPIO_write(0, a ? 1 : 0);
            last = (std::chrono::system_clock::now().time_since_epoch());
        };
        uint16_t slider = ADC(0);
        uint16_t knob = ADC(1);
        PWM_set(14, 0, 1000); /// resets gpio.
        PWM_set(15, 0, 1000); /// resets gpio.
        bool switch1 = GPIO_read(14) == GPIOState::HIGH;
        bool switch2 = GPIO_read(15) == GPIOState::HIGH;
        std::cout << slider << " - slider | " << knob << " - knob | " << switch1 << " - on/off switch | " << switch2 << " nuke switch"<< "\n";
    };

    return 0;
};
