// BLR-4000-485 激光测距传感器

#include <iostream>
#include <string>
#include <cstring>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <errno.h>
#include <signal.h>

// ============================================================
// 1. SerialPort 类 - 串口通信封装
// ============================================================
class SerialPort {
public:
    SerialPort() : fd_(-1) {}
    ~SerialPort() { close(); }

    bool open(const std::string& port, uint32_t baudrate) {
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0) {
            std::cerr << "Failed to open " << port << ": " << strerror(errno) << std::endl;
            return false;
        }

        struct termios options;
        tcgetattr(fd_, &options);

        cfmakeraw(&options);

        // 关闭硬件流控和软件流控
        options.c_cflag &= ~CRTSCTS;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);

        // 设置波特率
        speed_t speed;
        switch (baudrate) {
            case 9600:   speed = B9600; break;
            case 19200:  speed = B19200; break;
            case 38400:  speed = B38400; break;
            case 57600:  speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            default:     speed = B9600; break;
        }
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        // 8N1
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;

        options.c_cflag |= CREAD | CLOCAL;

        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 5;

        tcsetattr(fd_, TCSANOW, &options);
        tcflush(fd_, TCIOFLUSH);

        return true;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool is_open() const { return fd_ >= 0; }

    int write(const uint8_t* data, uint32_t size) {
        if (fd_ < 0) return -1;
        return ::write(fd_, data, size);
    }

    int read(uint8_t* buffer, uint32_t buffer_size, int timeout_ms) {
        if (fd_ < 0) return -1;

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        struct timeval timeout;
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;

        int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ret <= 0) {
            return 0;
        }

        return ::read(fd_, buffer, buffer_size);
    }

private:
    int fd_;
};

// ============================================================
// 2. BLR4000Sensor 类 - BLR-4000-485 传感器驱动
// ============================================================
class BLR4000Sensor {
public:
    BLR4000Sensor() : addr_(1), initialized_(false) {}
    ~BLR4000Sensor() { close(); }

    bool initialize(const std::string& port, uint32_t baudrate = 115200, uint8_t address = 1) {
        addr_ = address;
        if (!serial_.open(port, baudrate)) {
            std::cerr << "Failed to open serial port: " << port << std::endl;
            return false;
        }
        initialized_ = true;
        return true;
    }

    void close() {
        if (initialized_) {
            serial_.close();
            initialized_ = false;
        }
    }

    bool is_initialized() const { return initialized_; }

    // ---------- 核心测距功能 ----------
    bool readDistance(int32_t& distance_mm) {
        uint16_t value;
        if (!readRegister(0x0000, value)) {
            return false;
        }
        if (value == 0xFFFF) {
            distance_mm = -1;  // 超量程或测不到
        } else {
            distance_mm = value;
        }
        return true;
    }

    bool readBaudrate(uint32_t& baudrate) {
        uint16_t value;
        if (!readRegister(0x0001, value)) {
            return false;
        }
        baudrate = value * 100;
        return true;
    }

    bool setBaudrate(uint32_t baudrate) {
        uint32_t valid_rates[] = {9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800};
        bool valid = false;
        for (uint32_t rate : valid_rates) {
            if (rate == baudrate) {
                valid = true;
                break;
            }
        }
        if (!valid) {
            std::cerr << "Unsupported baudrate: " << baudrate << std::endl;
            return false;
        }
        uint16_t value = baudrate / 100;
        return writeRegister(0x0001, value);
    }

    bool readAddress(uint8_t& address) {
        uint16_t value;
        if (!readRegister(0x0002, value)) {
            return false;
        }
        if (value >= 1 && value <= 247) {
            address = (uint8_t)value;
            return true;
        }
        return false;
    }

    bool setAddress(uint8_t new_address) {
        if (new_address < 1 || new_address > 247) {
            std::cerr << "Address must be 1~247" << std::endl;
            return false;
        }
        if (!writeRegister(0x0002, new_address)) {
            return false;
        }
        addr_ = new_address;
        return true;
    }

    bool readVersion(uint16_t& version) {
        return readRegister(0x0004, version);
    }

    bool readSerialNumber(uint8_t serial[6]) {
        uint8_t req[8];
        uint32_t idx = 0;
        req[idx++] = addr_;
        req[idx++] = 0x03;
        req[idx++] = 0x00;
        req[idx++] = 0x05;
        req[idx++] = 0x00;
        req[idx++] = 0x03;
        uint16_t crc = crc16(req, idx);
        req[idx++] = crc & 0xFF;
        req[idx++] = (crc >> 8) & 0xFF;

        uint8_t resp[256];
        uint32_t resp_len = 0;
        if (!sendRequest(req, idx, resp, resp_len)) {
            return false;
        }

        if (resp_len < 9 || resp[2] != 0x06) {
            return false;
        }

        for (int i = 0; i < 6; i++) {
            serial[i] = resp[3 + i];
        }
        return true;
    }

    bool factoryReset() {
        return writeRegister(0x000F, 0x0001);
    }

private:
    // ---------- CRC16校验码 ----------
    static uint16_t crc16(const uint8_t* data, uint32_t len) {
        uint16_t crc = 0xFFFF;
        for (uint32_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    // ---------- Modbus通信 ----------
    bool sendRequest(const uint8_t* req, uint32_t req_len, uint8_t* resp, uint32_t& resp_len, int timeout_ms = 100) {
        if (!initialized_ || !serial_.is_open()) {
            return false;
        }

        // 清空串口缓冲区
        uint8_t dummy[64];
        while (serial_.read(dummy, sizeof(dummy), 10) > 0) {}

        // 发送
        int written = serial_.write(req, req_len);
        if (written != (int)req_len) {
            std::cerr << "Write failed: " << written << "/" << req_len << std::endl;
            return false;
        }

        // 读取响应
        uint8_t buffer[256];
        uint32_t total_read = 0;
        int retry = 0;
        const int max_retry = 3;

        while (total_read < 6 && retry < max_retry) {
            int n = serial_.read(buffer + total_read, sizeof(buffer) - total_read, timeout_ms);
            if (n > 0) {
                total_read += n;
            } else {
                retry++;
                usleep(10000);
            }
        }

        if (total_read < 6) {
            std::cerr << "Response timeout, received " << total_read << " bytes" << std::endl;
            return false;
        }

        // 检查地址和功能码
        if (buffer[0] != req[0] || buffer[1] != req[1]) {
            std::cerr << "Response address/function mismatch" << std::endl;
            return false;
        }

        // 检查CRC
        uint16_t calc_crc = crc16(buffer, total_read - 2);
        uint16_t resp_crc = (buffer[total_read - 1] << 8) | buffer[total_read - 2];
        if (calc_crc != resp_crc) {
            std::cerr << "CRC mismatch: calc=0x" << std::hex << calc_crc 
                      << " resp=0x" << resp_crc << std::dec << std::endl;
            return false;
        }

        memcpy(resp, buffer, total_read);
        resp_len = total_read;
        return true;
    }

    bool readRegister(uint16_t reg_addr, uint16_t& value) {
        uint8_t req[8];
        uint32_t idx = 0;
        req[idx++] = addr_;
        req[idx++] = 0x03;
        req[idx++] = (reg_addr >> 8) & 0xFF;
        req[idx++] = reg_addr & 0xFF;
        req[idx++] = 0x00;
        req[idx++] = 0x01;
        uint16_t crc = crc16(req, idx);
        req[idx++] = crc & 0xFF;
        req[idx++] = (crc >> 8) & 0xFF;

        uint8_t resp[256];
        uint32_t resp_len = 0;
        if (!sendRequest(req, idx, resp, resp_len)) {
            return false;
        }

        if (resp_len < 5 || resp[2] != 0x02) {
            return false;
        }

        value = (resp[3] << 8) | resp[4];
        return true;
    }

    bool writeRegister(uint16_t reg_addr, uint16_t value) {
        uint8_t req[8];
        uint32_t idx = 0;
        req[idx++] = addr_;
        req[idx++] = 0x06;
        req[idx++] = (reg_addr >> 8) & 0xFF;
        req[idx++] = reg_addr & 0xFF;
        req[idx++] = (value >> 8) & 0xFF;
        req[idx++] = value & 0xFF;
        uint16_t crc = crc16(req, idx);
        req[idx++] = crc & 0xFF;
        req[idx++] = (crc >> 8) & 0xFF;

        uint8_t resp[256];
        uint32_t resp_len = 0;
        if (!sendRequest(req, idx, resp, resp_len)) {
            return false;
        }

        return (resp_len >= 8 && memcmp(req, resp, 8) == 0);
    }

    SerialPort serial_;
    uint8_t addr_;
    bool initialized_;
};

// ============================================================
// 3. 主程序
// ============================================================
static volatile bool running = true;

void signal_handler(int sig) {
    (void)sig;
    running = false;
}

void print_usage(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " [port] [baudrate] [address]" << std::endl;
    std::cout << "  port      : serial device, default /dev/ttyS0" << std::endl;
    std::cout << "  baudrate  : 9600, 19200, 38400, 57600, 115200(default), 230400, 256000, 460800" << std::endl;
    std::cout << "  address   : Modbus address 1~247, default 1" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << prog_name << "                          # use defaults" << std::endl;
    std::cout << "  " << prog_name << " /dev/ttyS0 9600 1       # use 9600 baud, address 1" << std::endl;
    std::cout << "  " << prog_name << " /dev/ttyUSB0 115200 2   # use USB serial, address 2" << std::endl;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);

    // ---------- 解析参数 ----------
    std::string port = "/dev/ttyS0";
    uint32_t baudrate = 9600;
    uint8_t address = 0x53;

    if (argc >= 2) {
        if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
            print_usage(argv[0]);
            return 0;
        }
        port = argv[1];
    }
    if (argc >= 3) {
        baudrate = std::stoul(argv[2]);
    }
    if (argc >= 4) {
        address = (uint8_t)std::stoul(argv[3]);
    }

    std::cout << "============================================" << std::endl;
    std::cout << "  BLR-4000-485 激光测距传感器" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "  串口     : " << port << std::endl;
    std::cout << "  波特率 : " << baudrate << std::endl;
    std::cout << "  从机地址  : 0x" << std::hex << (int)address << std::dec << std::endl;
    std::cout << "============================================" << std::endl;

    // ---------- 初始化传感器 ----------
    BLR4000Sensor sensor;
    if (!sensor.initialize(port, baudrate, address)) {
        std::cerr << "Failed to initialize sensor!" << std::endl;
        return -1;
    }

    // ---------- 主循环：持续测距 ----------
    int loop_count = 0;
    while (running) {
        int32_t distance_mm;

        if (sensor.readDistance(distance_mm)) {
            if (distance_mm < 0) {
                std::cout << "[" << ++loop_count << "] 距离: 超出量程 (>4m or <5cm)" << std::endl;
            } else {
                float distance_m = distance_mm / 1000.0f;
                std::cout << "[" << ++loop_count << "] 距离: " 
                          << distance_mm << " mm = " << distance_m << " m" << std::endl;
            }
        } else {
            std::cerr << "[" << ++loop_count << "] 读取失败!" << std::endl;
        }

        usleep(1000000);  // 1s
    }

    // ---------- 清理 ----------
    sensor.close();
    std::cout << std::endl << "关闭传感器传送" << std::endl;

    return 0;
}
