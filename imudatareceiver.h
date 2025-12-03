#ifndef IMUDATARECEIVER_H
#define IMUDATARECEIVER_H

#pragma once

#include <iostream>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>

#pragma pack(push,1)
struct IMUData {
    uint16_t header;

    uint32_t timestamp;
    float ax, ay, az, gx, gy, gz;

    void print() const {
        std::cout << "[IMUData] header=" << header << " timestamp_us=" << timestamp
                  << " (ax, ay, az)=(" << ax << ", " << ay << ", " << az << ")"
                  << " (gx, gy, gz)=(" << gx << ", " << gy << ", " << gz << ")"
                  << std::endl;
    }
};
#pragma pack(pop)

class IMUDataReceiver {
public:
    IMUDataReceiver() : fd(-1) {
        // Object wordt enkel aangemaakt
    }

    IMUDataReceiver(const std::string &port, int baudrate) : fd(-1) {
        initBaud(port, baudrate);
    }

    ~IMUDataReceiver() {
        stop();
    }

    bool initBaud(const std::string &port, int baudrate) {
        fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            perror("[IMUDataReceiver] Failed to open serial port");
            return false;
        }

        termios tty{};
        if (tcgetattr(fd, &tty) != 0) {
            perror("[IMUDataReceiver] tcgetattr failed");
            return false;
        }

        cfsetospeed(&tty, convertBaud(baudrate));
        cfsetispeed(&tty, convertBaud(baudrate));

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8 bits
        tty.c_cflag |= (CLOCAL | CREAD);                // enable receiver
        tty.c_cflag &= ~(PARENB | PARODD);             // no parity
        tty.c_cflag &= ~CSTOPB;                        // 1 stop bit
        tty.c_cflag &= ~CRTSCTS;                       // no flow control

        tty.c_iflag = 0;
        tty.c_oflag = 0;
        tty.c_lflag = 0;

        tty.c_cc[VMIN]  = 1;   // read blocks until at least 1 byte
        tty.c_cc[VTIME] = 0;   // no timeout

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            perror("[IMUDataReceiver] tcsetattr failed");
            return false;
        }
        return true;
    }

    void syncToData(uint16_t header, size_t frameSize) {
        uint8_t prev = 0, curr = 0;
        std::vector<uint8_t> frame;

        while (true) {
            uint8_t byte;
            int n = ::read(fd, &byte, 1);
            if (n <= 0) continue;

            prev = curr;
            curr = byte;

            // header in little endian
            if (prev == (header & 0xFF) && curr == ((header >> 8) & 0xFF)) {
                std::cout << "[IMUDataReceiver] Header gevonden!" << std::endl;

                frame.clear();
                frame.push_back(prev);
                frame.push_back(curr);

                while (frame.size() < frameSize) {
                    uint8_t b;
                    int n2 = ::read(fd, &b, 1);
                    if (n2 > 0) frame.push_back(b);
                }

                std::cout << "[IMUDataReceiver] Volledig frame van " << frame.size() << " bytes gelezen\n";
                setBlockingFrameMode();
                return;
            }
        }
    }

    void read(IMUData &data) {
        ::read(fd, &data, sizeof(IMUData));
    }

    void stop() {
        if (fd >= 0) {
            std::cout << "[IMUDataReceiver] Closing serial connection..." << std::endl;
            close(fd);
            fd = -1;
        } else {
            std::cout << "[IMUDataReceiver] No active connection to close." << std::endl;
        }
    }


private:
    int fd;

    static speed_t convertBaud(int baud) {
        switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:
            std::cerr << "[IMUDataReceiver] Unsupported baudrate, defaulting to 115200\n";
            return B115200;
        }
    }

    void setBlockingFrameMode() {
        termios tty;
        ::tcgetattr(fd, &tty);
        tty.c_cc[VMIN] = sizeof(IMUData);
        tty.c_cc[VTIME] = 0;
        ::tcsetattr(fd, TCSANOW, &tty);
    }

};


#endif // IMUDATARECEIVER_H
