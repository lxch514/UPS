// SINDT-485 三维运动姿态测量系统 
// 功能: 角度、经纬度、原点经纬度、距离、定位状态

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
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include <ctime>

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

        options.c_cflag &= ~CRTSCTS;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);

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
        if (ret <= 0) return 0;

        return ::read(fd_, buffer, buffer_size);
    }

private:
    int fd_;
};

// ============================================================
// 2. SINDT_Sensor 类 - SINDT-485 传感器驱动
// ============================================================
class SINDT_Sensor {
public:
    SINDT_Sensor() : addr_(0x65), initialized_(false) {}
    ~SINDT_Sensor() { close(); }

    bool initialize(const std::string& port, uint32_t baudrate = 9600, uint8_t address = 0x65) {
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

    // ============================================================
    // 1. 读取三轴角度 (Roll/Pitch/Yaw)
    // ============================================================
    bool readAngle(float& roll, float& pitch, float& yaw) {
        uint16_t regs[3];
        if (!readRegisters(0x3D, 3, regs)) return false;
        roll = (int16_t)regs[0] / 32768.0f * 180.0f;
        pitch = (int16_t)regs[1] / 32768.0f * 180.0f;
        yaw = (int16_t)regs[2] / 32768.0f * 180.0f;
        return true;
    }

    // ============================================================
    // 2. 读取经纬度 (单位: 度, 格式: dd.dddddd)
    //    注意: 模块返回的是 ddmm.mmmmm 格式（度分）
    // ============================================================
    bool readLatLon(double& latitude, double& longitude) {
        uint16_t regs[4];  // LonL, LonH, LatL, LatH
        if (!readRegisters(0x49, 4, regs)) return false;

        int32_t lon_raw = (int32_t)((regs[0] << 16) | regs[1]);
        int32_t lat_raw = (int32_t)((regs[2] << 16) | regs[3]);

        if (lon_raw == 0 || lat_raw == 0) {
            longitude = 0.0;
            latitude = 0.0;
            return false;
        }

        longitude = nmeaToDegree(lon_raw);
        latitude = nmeaToDegree(lat_raw);
        return true;
    }

    // ============================================================
    // 3. 读取GPS定位状态 (通过卫星数判断)
    // ============================================================
    bool readGPSStatus(uint16_t& svnum, uint16_t& pdop, uint16_t& hdop, uint16_t& vdop) {
        uint16_t regs[4];
        if (!readRegisters(0x55, 4, regs)) return false;
        svnum = regs[0];
        pdop = regs[1];
        hdop = regs[2];
        vdop = regs[3];
        return true;
    }

    // ============================================================
    // 4. 读取GPS海拔 (单位: 米)
    // ============================================================
    bool readGPSHeight(float& height) {
        uint16_t regs[2];
        if (!readRegisters(0x4D, 2, regs)) return false;
        height = (int16_t)regs[0] / 10.0f;  
        return true;
    }

    // ============================================================
    // 5. 读取版本号 (验证通信)
    // ============================================================
    bool readVersion(uint16_t& version) {
        return readRegister(0x2E, version);
    }

    // ============================================================
    // 6. 一次性读取所有需要的数据 (推荐使用, 效率最高)
    // ============================================================
    bool readAllData(float& roll, float& pitch, float& yaw,
                     double& latitude, double& longitude,
                     uint16_t& svnum, float& height) {
        uint16_t angle_regs[3];
        if (!readRegisters(0x3D, 3, angle_regs)) return false;
        roll = (int16_t)angle_regs[0] / 32768.0f * 180.0f;
        pitch = (int16_t)angle_regs[1] / 32768.0f * 180.0f;
        yaw = (int16_t)angle_regs[2] / 32768.0f * 180.0f;

        uint16_t lonlat_regs[4];
        if (!readRegisters(0x49, 4, lonlat_regs)) return false;
        int32_t lon_raw = (int32_t)((lonlat_regs[0] << 16) | lonlat_regs[1]);
        int32_t lat_raw = (int32_t)((lonlat_regs[2] << 16) | lonlat_regs[3]);
        if (lon_raw != 0 && lat_raw != 0) {
            longitude = nmeaToDegree(lon_raw);
            latitude = nmeaToDegree(lat_raw);
        } else {
            longitude = 0.0;
            latitude = 0.0;
        }

        uint16_t svnum_reg;
        if (!readRegister(0x55, svnum_reg)) return false;
        svnum = svnum_reg;

        uint16_t height_reg;
        if (!readRegister(0x4D, height_reg)) return false;
        height = (int16_t)height_reg / 10.0f;

        return true;
    }

private:
    // ============================================================
    // CRC16 校验码计算
    // ============================================================
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

    // ============================================================
    // NMEA格式转度: ddmm.mmmmm -> dd.dddddd
    // ============================================================
    static double nmeaToDegree(int32_t nmea) {
        double raw = (double)nmea / 100000.0;
        int32_t degrees = (int32_t)(raw / 100.0);
        double minutes = raw - degrees * 100.0;
        return degrees + minutes / 60.0;
    }

    // ============================================================
    // 底层Modbus通信
    // ============================================================
    bool sendRequest(const uint8_t* req, uint32_t req_len, uint8_t* resp, uint32_t& resp_len, int timeout_ms = 100) {
        if (!initialized_ || !serial_.is_open()) return false;

        uint8_t dummy[64];
        while (serial_.read(dummy, sizeof(dummy), 10) > 0) {}

        int written = serial_.write(req, req_len);
        if (written != (int)req_len) {
            std::cerr << "Write failed: " << written << "/" << req_len << std::endl;
            return false;
        }

        uint8_t buffer[256];
        uint32_t total_read = 0;
        int retry = 0;
        const int max_retry = 3;
        uint32_t expected_min = 5;

        while (total_read < expected_min && retry < max_retry) {
            int n = serial_.read(buffer + total_read, sizeof(buffer) - total_read, timeout_ms);
            if (n > 0) {
                total_read += n;
                if (total_read >= 3) {
                    expected_min = 3 + buffer[2] + 2;
                }
            } else {
                retry++;
                usleep(10000);
            }
        }

        if (total_read < expected_min) {
            std::cerr << "Response timeout, received " << total_read << " bytes" << std::endl;
            return false;
        }

        if (buffer[0] != req[0] || buffer[1] != req[1]) {
            std::cerr << "Response address/function mismatch" << std::endl;
            return false;
        }

        uint16_t calc_crc = crc16(buffer, total_read - 2);
        uint16_t resp_crc = (buffer[total_read - 1] << 8) | buffer[total_read - 2];
        if (calc_crc != resp_crc) {
            std::cerr << "CRC mismatch" << std::endl;
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
        if (!sendRequest(req, idx, resp, resp_len)) return false;

        if (resp_len < 5 || resp[2] != 0x02) return false;
        value = (resp[3] << 8) | resp[4];
        return true;
    }

    bool readRegisters(uint16_t start_addr, uint16_t count, uint16_t* values) {
        if (count == 0 || count > 125) return false;

        uint8_t req[8];
        uint32_t idx = 0;
        req[idx++] = addr_;
        req[idx++] = 0x03;
        req[idx++] = (start_addr >> 8) & 0xFF;
        req[idx++] = start_addr & 0xFF;
        req[idx++] = (count >> 8) & 0xFF;
        req[idx++] = count & 0xFF;
        uint16_t crc = crc16(req, idx);
        req[idx++] = crc & 0xFF;
        req[idx++] = (crc >> 8) & 0xFF;

        uint8_t resp[256];
        uint32_t resp_len = 0;
        if (!sendRequest(req, idx, resp, resp_len)) return false;

        uint32_t expected_len = 3 + count * 2 + 2;
        if (resp_len < expected_len || resp[2] != count * 2) return false;

        for (uint16_t i = 0; i < count; i++) {
            values[i] = (resp[3 + i * 2] << 8) | resp[4 + i * 2];
        }
        return true;
    }

    SerialPort serial_;
    uint8_t addr_;
    bool initialized_;
};

// ============================================================
// 3. 全局变量和辅助函数
// ============================================================
static volatile bool running = true;

void signal_handler(int sig) {
    (void)sig;
    running = false;
}

double calcDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

void print_usage(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " [port] [baudrate] [address]" << std::endl;
    std::cout << "  port      : serial device, default /dev/ttyS0" << std::endl;
    std::cout << "  baudrate  : 9600(default), 19200, 38400, 57600, 115200" << std::endl;
    std::cout << "  address   : Modbus address, default 0x65" << std::endl;
    std::cout << std::endl;
    std::cout << "Commands (运行时输入):" << std::endl;
    std::cout << "  origin    : 设置当前位置为原点" << std::endl;
    std::cout << "  status    : 显示当前定位状态" << std::endl;
    std::cout << "  reset     : 重置原点" << std::endl;
    std::cout << "  quit/exit : 退出程序" << std::endl;
}

// ============================================================
// 4. 主程序
// ============================================================
int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);

    // ---------- 解析参数 ----------
    std::string port = "/dev/ttyS0";
    uint32_t baudrate = 9600;
    uint8_t address = 0x65;

    if (argc >= 2) {
        if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
            print_usage(argv[0]);
            return 0;
        }
        port = argv[1];
    }
    if (argc >= 3) baudrate = std::stoul(argv[2]);
    if (argc >= 4) {
        std::string addr_str = argv[3];
        if (addr_str.find("0x") == 0 || addr_str.find("0X") == 0) {
            address = (uint8_t)std::stoul(addr_str, nullptr, 16);
        } else {
            address = (uint8_t)std::stoul(addr_str);
        }
    }

    std::cout << "============================================" << std::endl;
    std::cout << "  SINDT-485 定位与姿态测量系统" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "  串口设备  : " << port << std::endl;
    std::cout << "  波特率    : " << baudrate << std::endl;
    std::cout << "  从机地址  : 0x" << std::hex << (int)address << std::dec << std::endl;
    std::cout << "============================================" << std::endl;

    // ---------- 初始化传感器 ----------
    SINDT_Sensor sensor;
    if (!sensor.initialize(port, baudrate, address)) {
        std::cerr << "Failed to initialize sensor!" << std::endl;
        return -1;
    }

    // ---------- 验证通信 ----------
    uint16_t version;
    if (sensor.readVersion(version)) {
        std::cout << "  固件版本  : V" << (version >> 8) << "." << (version & 0xFF) << std::endl;
    } else {
        std::cerr << "  固件版本  : 读取失败！" << std::endl;
    }
    std::cout << "--------------------" << std::endl;
    std::cout << "  原点未设置" << std::endl;
    std::cout << "  输入 'origin' 设置当前位置为原点" << std::endl;
    std::cout << "  输入 'status' 查看定位状态" << std::endl;
    std::cout << "  输入 'quit' 退出" << std::endl;
    std::cout << std::endl;

    // ---------- 原点相关变量 ----------
    double origin_lat = 0.0, origin_lon = 0.0;
    bool origin_set = false;
    int loop_count = 0;

    // ---------- 主循环 ----------
    while (running) {
        float roll, pitch, yaw;
        double latitude = 0.0, longitude = 0.0;
        uint16_t svnum = 0;
        float height = 0.0;

        bool has_gps = sensor.readAllData(roll, pitch, yaw, latitude, longitude, svnum, height);
        loop_count++;

        bool is_fixed = has_gps && (svnum >= 4);

        // ---------- 获取当前时间 ----------
        time_t now = time(nullptr);
        struct tm* tm_now = localtime(&now);

        // ---------- 显示数据 ----------
        std::cout << "\r["
                  << std::setw(4) << std::setfill('0') << (tm_now->tm_year + 1900) << "-"
                  << std::setw(2) << std::setfill('0') << (tm_now->tm_mon + 1) << "-"
                  << std::setw(2) << std::setfill('0') << tm_now->tm_mday << " "
                  << std::setw(2) << std::setfill('0') << tm_now->tm_hour << ":"
                  << std::setw(2) << std::setfill('0') << tm_now->tm_min << ":"
                  << std::setw(2) << std::setfill('0') << tm_now->tm_sec << "] ";

        // 角度
        std::cout << std::fixed << std::setprecision(1)
                  << "R:" << roll << "° P:" << pitch << "° Y:" << yaw << "° | ";

        // 定位状态
        if (is_fixed) {
            std::cout << "GPS:固定(" << svnum << "星) | ";
        } else if (svnum > 0 && svnum < 4) {
            std::cout << "GPS:搜星(" << svnum << "星) | ";
        } else {
            std::cout << "GPS:未定位  | ";
        }

        // 经纬度
        if (is_fixed) {
            std::cout << std::setprecision(6)
                      << "位置:" << latitude << "°, " << longitude << "° | ";
        } else {
            std::cout << "位置:--, -- | ";
        }

        // 距离
        if (origin_set && is_fixed) {
            double dist = calcDistance(origin_lat, origin_lon, latitude, longitude);
            std::cout << "距原点:" << std::setprecision(1) << dist << "m";
        } else if (origin_set && !is_fixed) {
            std::cout << "距原点:GPS未定位";
        } else {
            std::cout << "距原点:原点未设置";
        }

        std::cout << std::flush;

        // ---------- 检测键盘输入 (非阻塞) ----------
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(STDIN_FILENO, &read_fds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;

        int ret = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &tv);

        if (ret > 0 && FD_ISSET(STDIN_FILENO, &read_fds)) {
            std::string cmd;
            std::getline(std::cin, cmd);

            if (cmd == "origin" || cmd == "o") {
                if (is_fixed) {
                    origin_lat = latitude;
                    origin_lon = longitude;
                    origin_set = true;
                    std::cout << "\n\n原点已设置: "
                              << std::setprecision(6) << origin_lat << "°, "
                              << origin_lon << "°" << std::endl;
                } else {
                    std::cout << "\n\nGPS未定位，无法设置原点！" << std::endl;
                }
            } else if (cmd == "status" || cmd == "s") {
                std::cout << "\n\n========== 定位状态 ==========" << std::endl;
                std::cout << "  卫星数    : " << svnum << std::endl;
                std::cout << "  定位状态  : " << (is_fixed ? "固定 " : "未固定 ") << std::endl;
                if (is_fixed) {
                    std::cout << std::setprecision(6);
                    std::cout << "  纬度      : " << latitude << "°" << std::endl;
                    std::cout << "  经度      : " << longitude << "°" << std::endl;
                    std::cout << "  海拔      : " << std::setprecision(1) << height << "m" << std::endl;
                }
                if (origin_set) {
                    std::cout << std::setprecision(6);
                    std::cout << "  原点纬度  : " << origin_lat << "°" << std::endl;
                    std::cout << "  原点经度  : " << origin_lon << "°" << std::endl;
                    if (is_fixed) {
                        double dist = calcDistance(origin_lat, origin_lon, latitude, longitude);
                        std::cout << "  距原点    : " << std::setprecision(1) << dist << "m" << std::endl;
                    }
                } else {
                    std::cout << "  原点      : 未设置" << std::endl;
                }
                std::cout << "================================" << std::endl;
            } else if (cmd == "reset") {
                origin_set = false;
                origin_lat = 0.0;
                origin_lon = 0.0;
                std::cout << "\n\n原点已重置" << std::endl;
            } else if (cmd == "quit" || cmd == "exit" || cmd == "q") {
                running = false;
                std::cout << "\n" << std::endl;
            }
        }
    }

    // ---------- 清理 ----------
    sensor.close();
    std::cout << "传感器已关闭，程序退出。" << std::endl;
    return 0;
}