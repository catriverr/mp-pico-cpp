#pragma once

#include <cstdint>
#include <poll.h>
#include "pico.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <filesystem>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static int try_open_micro_python_repl(const std::string& path) {
    int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (CLOCAL | CREAD | CS8);
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIFLUSH);

    // Send Ctrl+C Ctrl+D + newline
    const char* wake = "\x03\x04\r\n";
    write(fd, wake, strlen(wake));

    usleep(200000); // 200ms

    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, 500) <= 0) {
        close(fd);
        return -1;
    }

    char buf[512] = {0};
    int n = read(fd, buf, sizeof(buf));
    std::string response(buf, n);

    if (response.find(">>>") != std::string::npos ||
        response.find("MicroPython") != std::string::npos) {
        return fd; // VALID connection
    }

    close(fd);
    return -1;
};

static std::vector<std::string> find_serial_devices() {
    std::vector<std::string> matches;
    for (const auto& entry : fs::directory_iterator("/dev")) {
        std::string path = entry.path().string();
        if (path.find("tty.usbmodem") != std::string::npos) {
            matches.push_back(path);
        }
    }
    return matches;
};

namespace pico_util {
    bool auto_init() {
        auto devices = find_serial_devices();
        for (const auto& dev : devices) {
            int fd = try_open_micro_python_repl(dev);
            if (fd >= 0) {
                return init(fd);  // Uses already opened descriptor
            }
        }
        return false;
    }
};



static int serial_fd = -1;

bool pico_util::init(int fd) {
    serial_fd = fd;
    eval("from machine import ADC, PWM, Pin");
    return true;
};

std::string pico_util::eval(const std::string& code) {
    if (serial_fd < 0) return "[ERROR: Not connected]";

    std::string full = code + "\r\n";
    write(serial_fd, full.c_str(), full.size());

    std::string output;
    std::vector<char> buf(512);
    int loops = 20;

    while (loops-- > 0) {
        struct pollfd pfd = { serial_fd, POLLIN, 0 };
        int ret = poll(&pfd, 1, 300);
        if (ret <= 0) break;

        int n = read(serial_fd, buf.data(), buf.size());
        if (n <= 0) break;

        output.append(buf.data(), n);

        // Stop once the prompt is seen again
        if (output.find(">>> ") != std::string::npos || output.find(">>>") != std::string::npos) {
            break;
        }
    }

    // Split into lines
    std::istringstream iss(output);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(iss, line)) {
        lines.push_back(line);
    }

    // Remove first and last line (the echoed command & prompt)
    if (lines.size() >= 2) {
        lines.erase(lines.begin());          // Remove first line
        lines.erase(lines.end() - 1);        // Remove last line
    }

    // Join remaining lines back
    std::string cleaned_output;
    for (size_t i = 0; i < lines.size(); ++i) {
        cleaned_output += lines[i];
        if (i + 1 < lines.size()) cleaned_output += "\n";
    }

    if (!cleaned_output.empty() && cleaned_output.back() == '\r') {
        cleaned_output.pop_back();
    }

    return cleaned_output;
}




uint16_t pico_util::ADC(int pin) {
    std::ostringstream oss;
    oss << "from machine import ADC;"
        << "print(ADC(" << pin << ").read_u16())";
    auto res = eval(oss.str());
    return std::stoi(res);
}

float pico_util::core_temp() {
    return std::stof(eval(
        "import machine, utime;"
        "sensor_temp = machine.ADC(4);"
        "reading = sensor_temp.read_u16() * 3.3 / 65535;"
        "temp = 27 - (reading - 0.706)/0.001721;"
        "print(temp)"
    ));
}

pico_util::GPIOState pico_util::GPIO_read(int pin) {
    std::ostringstream oss;
    oss << "from machine import Pin;"
        << "print(Pin(" << pin << ", Pin.IN).value())";
    std::string res = eval(oss.str());
    return std::stoi(res) == 1 ? GPIOState::HIGH : GPIOState::LOW;
}

void pico_util::GPIO_write(int pin, float value) {
    int val = value >= 0.5 ? 1 : 0;
    std::ostringstream oss;
    oss << "from machine import Pin;"
        << "Pin(" << pin << ", Pin.OUT).value(" << val << ")";
    eval(oss.str());
}

pico_util::PWMSettings pico_util::PWM(int pin) {
    return { 1000, 0.5f }; // Mock value
}

void pico_util::PWM_set(int pin, uint16_t duty, int freq) {
    std::ostringstream oss;
    oss << "from machine import PWM, Pin;"
        << "pwm = PWM(Pin(" << pin << "));"
        << "pwm.freq(" << freq << ");"
        << "pwm.duty_u16(" << duty << ")";
    eval(oss.str());
}
