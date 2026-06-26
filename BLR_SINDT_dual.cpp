// 双串口独立读取程序：BLR-4000 (ttyS0) + SINDT-485 (ttyS1)
// 编译: g++ -std=c++11 -o BLR_SINDT_dual BLR_SINDT_dual.cpp -lpthread
// 运行: ./BLR_SINDT_dual

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
#include <thread>
#include <mutex>
#include <atomic>

// ============================================================
// 1. SerialPort 类 (串口封装)
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
// 2. Modbus 底层工具函数
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

bool sendModbusRequest(SerialPort& serial, uint8_t* req, uint32_t req_len,
                        uint8_t* resp, uint32_t& resp_len, int timeout_ms = 150) {
    if (!serial.is_open()) return false;

    uint8_t dummy[64];
    while (serial.read(dummy, sizeof(dummy), 10) > 0) {}

    int written = serial.write(req, req_len);
    if (written != (int)req_len) {
        return false;
    }

    uint8_t buffer[256];
    uint32_t total_read = 0;
    int retry = 0;
    const int max_retry = 3;
    uint32_t expected_min = 5;

    while (total_read < expected_min && retry < max_retry) {
        int n = serial.read(buffer + total_read, sizeof(buffer) - total_read, timeout_ms);
        if (n > 0) {
            total_read += n;
            if (total_read >= 3) {
                expected_min = 3 + buffer[2] + 2;
            }
        } else {
            retry++;
            usleep(5000);
        }
    }

    if (total_read < expected_min) {
        return false;
    }

    uint16_t calc_crc = crc16(buffer, total_read - 2);
    uint16_t resp_crc = (buffer[total_read - 1] << 8) | buffer[total_read - 2];
    if (calc_crc != resp_crc) {
        return false;
    }

    memcpy(resp, buffer, total_read);
    resp_len = total_read;
    return true;
}

// ============================================================
// 3. 距离计算函数 (Haversine)
// ============================================================
double calcDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dlat/2) * sin(dlat/2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

// ============================================================
// 4. NMEA 格式转度
// ============================================================
double nmeaToDegree(int32_t nmea) {
    double raw = (double)nmea / 100000.0;
    int32_t degrees = (int32_t)(raw / 100.0);
    double minutes = raw - degrees * 100.0;
    return degrees + minutes / 60.0;
}

// ============================================================
// 5. BLR-4000 读取函数
// ============================================================
bool readBLR4000(SerialPort& serial, uint8_t addr, float& distance_m) {
    uint8_t req[8];
    req[0] = addr;
    req[1] = 0x04;
    req[2] = 0x00;
    req[3] = 0x00;
    req[4] = 0x00;
    req[5] = 0x01;
    uint16_t crc = crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    uint8_t resp[256];
    uint32_t resp_len = 0;
    if (!sendModbusRequest(serial, req, 8, resp, resp_len)) {
        return false;
    }

    if (resp_len < 5 || resp[2] != 0x02) return false;

    uint16_t dist_mm = (resp[3] << 8) | resp[4];
    if (dist_mm == 0xFFFF) {
        distance_m = -1.0f;
    } else {
        distance_m = dist_mm / 1000.0f;
    }
    return true;
}

// ============================================================
// 6. SINDT-485 读取函数
// ============================================================
bool readSINDT_Angle(SerialPort& serial, uint8_t addr, float& roll, float& pitch, float& yaw) {
    uint8_t req[8];
    req[0] = addr;
    req[1] = 0x03;
    req[2] = 0x00;
    req[3] = 0x3D;
    req[4] = 0x00;
    req[5] = 0x03;
    uint16_t crc = crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    uint8_t resp[256];
    uint32_t resp_len = 0;
    if (!sendModbusRequest(serial, req, 8, resp, resp_len)) {
        return false;
    }

    if (resp_len < 5 || resp[2] != 0x06) return false;

    roll  = (int16_t)((resp[3] << 8) | resp[4]) / 32768.0f * 180.0f;
    pitch = (int16_t)((resp[5] << 8) | resp[6]) / 32768.0f * 180.0f;
    yaw   = (int16_t)((resp[7] << 8) | resp[8]) / 32768.0f * 180.0f;
    return true;
}

bool readSINDT_GPS(SerialPort& serial, uint8_t addr, double& latitude, double& longitude) {
    uint8_t req[8];
    req[0] = addr;
    req[1] = 0x03;
    req[2] = 0x00;
    req[3] = 0x49;
    req[4] = 0x00;
    req[5] = 0x04;
    uint16_t crc = crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    uint8_t resp[256];
    uint32_t resp_len = 0;
    if (!sendModbusRequest(serial, req, 8, resp, resp_len)) {
        return false;
    }

    if (resp_len < 5 || resp[2] != 0x08) return false;

    int32_t lon_raw = (int32_t)((resp[3] << 24) | (resp[4] << 16) | (resp[5] << 8) | resp[6]);
    int32_t lat_raw = (int32_t)((resp[7] << 24) | (resp[8] << 16) | (resp[9] << 8) | resp[10]);

    if (lon_raw == 0 || lat_raw == 0) {
        longitude = 0.0;
        latitude = 0.0;
        return false;
    }

    longitude = nmeaToDegree(lon_raw);
    latitude = nmeaToDegree(lat_raw);
    return true;
}

bool readSINDT_SV(SerialPort& serial, uint8_t addr, uint16_t& svnum) {
    uint8_t req[8];
    req[0] = addr;
    req[1] = 0x03;
    req[2] = 0x00;
    req[3] = 0x55;
    req[4] = 0x00;
    req[5] = 0x01;
    uint16_t crc = crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    uint8_t resp[256];
    uint32_t resp_len = 0;
    if (!sendModbusRequest(serial, req, 8, resp, resp_len)) {
        return false;
    }

    if (resp_len < 5 || resp[2] != 0x02) return false;

    svnum = (resp[3] << 8) | resp[4];
    return true;
}

// ============================================================
// 7. 共享数据结构和互斥锁
// ============================================================
struct BLRData {
    float distance_m;
    bool valid;
};

struct SINDTData {
    float roll, pitch, yaw;
    double lat, lon;
    uint16_t svnum;
    bool valid_angle;
    bool valid_gps;
    bool valid_sv;
};

BLRData g_blr_data = {0.0f, false};
SINDTData g_sindt_data = {0.0f, 0.0f, 0.0f, 0.0, 0.0, 0, false, false, false};
std::mutex g_data_mutex;
std::atomic<bool> running(true);

// 原点数据
double origin_lat = 0.0, origin_lon = 0.0;
bool origin_set = false;
std::mutex g_origin_mutex;

// ============================================================
// 8. BLR-4000 线程函数 (ttyS0)
// ============================================================
void thread_BLR() {
    SerialPort serial;
    if (!serial.open("/dev/ttyS0", 9600)) {
        std::cerr << "[BLR] 无法打开 /dev/ttyS0" << std::endl;
        return;
    }

    uint8_t addr = 0x53;
    std::cout << "[BLR] 线程启动，串口 /dev/ttyS0，地址 0x53" << std::endl;

    while (running) {
        float dist = 0.0f;
        bool ok = readBLR4000(serial, addr, dist);

        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            g_blr_data.distance_m = dist;
            g_blr_data.valid = ok;
        }

        usleep(1000000);  // 2Hz
    }

    serial.close();
    std::cout << "[BLR] 线程退出" << std::endl;
}

// ============================================================
// 9. SINDT-485 线程函数 (ttyS3)
// ============================================================
void thread_SINDT() {
    SerialPort serial;
    if (!serial.open("/dev/ttyS3", 9600)) {
        std::cerr << "[SINDT] 无法打开 /dev/ttyS3" << std::endl;
        return;
    }

    uint8_t addr = 0x65;
    std::cout << "[SINDT] 线程启动，串口 /dev/ttyS3，地址 0x65" << std::endl;

    while (running) {
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        double lat = 0.0, lon = 0.0;
        uint16_t svnum = 0;

        bool ok_angle = readSINDT_Angle(serial, addr, roll, pitch, yaw);
        bool ok_gps = readSINDT_GPS(serial, addr, lat, lon);
        bool ok_sv = readSINDT_SV(serial, addr, svnum);

        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            g_sindt_data.roll = roll;
            g_sindt_data.pitch = pitch;
            g_sindt_data.yaw = yaw;
            g_sindt_data.lat = lat;
            g_sindt_data.lon = lon;
            g_sindt_data.svnum = svnum;
            g_sindt_data.valid_angle = ok_angle;
            g_sindt_data.valid_gps = ok_gps;
            g_sindt_data.valid_sv = ok_sv;
        }

        usleep(1000000);  // 2Hz
    }

    serial.close();
    std::cout << "[SINDT] 线程退出" << std::endl;
}

// ============================================================
// 10. 获取当前时间字符串
// ============================================================
std::string getTimeStr() {
    time_t now = time(nullptr);
    struct tm* tm_now = localtime(&now);
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             tm_now->tm_year + 1900, tm_now->tm_mon + 1, tm_now->tm_mday,
             tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);
    return std::string(buf);
}

// ============================================================
// 11. 显示帮助信息
// ============================================================
void showHelp() {
    std::cout << "\n========== 交互命令 ==========" << std::endl;
    std::cout << "  origin  (o)  : 设置当前位置为原点 (需GPS固定)" << std::endl;
    std::cout << "  status  (s)  : 查看完整定位状态" << std::endl;
    std::cout << "  reset   (r)  : 重置原点" << std::endl;
    std::cout << "  help    (h)  : 显示本帮助信息" << std::endl;
    std::cout << "  quit    (q)  : 退出程序" << std::endl;
    std::cout << "================================" << std::endl;
}

// ============================================================
// 12. 主程序
// ============================================================
void signal_handler(int sig) {
    (void)sig;
    running = false;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);

    std::cout << "==================================================" << std::endl;
    std::cout << "  双串口独立读取程序" << std::endl;
    std::cout << "  BLR-4000   : /dev/ttyS0, 地址 0x53, 9600bps" << std::endl;
    std::cout << "  SINDT-485  : /dev/ttyS1, 地址 0x65, 9600bps" << std::endl;
    std::cout << "==================================================" << std::endl;
    showHelp();
    std::cout << "按 Ctrl+C 或输入 'quit' 停止" << std::endl << std::endl;

    // ---------- 启动两个线程 ----------
    std::thread t1(thread_BLR);
    std::thread t2(thread_SINDT);

    // ---------- 主循环：显示数据和接收命令 ----------
    int count = 0;
    while (running) {
        count++;

        // 读取共享数据（加锁）
        BLRData blr;
        SINDTData sindt;
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            blr = g_blr_data;
            sindt = g_sindt_data;
        }

        bool is_fixed = sindt.valid_gps && sindt.valid_sv && (sindt.svnum >= 4);

        // ----- 显示数据 -----
        std::cout << "[" << getTimeStr() << "] ";

        // BLR-4000 距离
        if (blr.valid) {
            if (blr.distance_m < 0) {
                std::cout << "距离: 超量程 ";
            } else {
                std::cout << "距离: " << std::fixed << std::setprecision(3) << blr.distance_m << "m ";
            }
        } else {
            std::cout << "距离: -- ";
        }

        std::cout << "| ";

        // SINDT-485 角度
        if (sindt.valid_angle) {
            std::cout << "角度: R=" << std::setprecision(1) << sindt.roll
                      << "° P=" << sindt.pitch << "° Y=" << sindt.yaw << "° ";
        } else {
            std::cout << "角度: -- ";
        }

        std::cout << "| ";

        // GPS
        if (is_fixed) {
            std::cout << "GPS: 固定(" << sindt.svnum << "星) "
                      << std::setprecision(6) << sindt.lat << "°, " << sindt.lon << "°";
        } else if (sindt.valid_sv && sindt.svnum > 0) {
            std::cout << "GPS: 搜星(" << sindt.svnum << "星)";
        } else {
            std::cout << "GPS: 未定位";
        }

        // 距原点距离
        {
            std::lock_guard<std::mutex> lock(g_origin_mutex);
            if (origin_set && is_fixed) {
                double dist = calcDistance(origin_lat, origin_lon, sindt.lat, sindt.lon);
                std::cout << " | 距原点: " << std::setprecision(1) << dist << "m";
            } else if (origin_set) {
                std::cout << " | 距原点: GPS未定位";
            }
        }

        std::cout << std::endl;

        // ----- 非阻塞键盘输入检测 -----
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(STDIN_FILENO, &read_fds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000;

        int ret = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &tv);

        if (ret > 0 && FD_ISSET(STDIN_FILENO, &read_fds)) {
            std::string cmd;
            std::getline(std::cin, cmd);

            // ----- origin: 设置原点 -----
            if (cmd == "origin" || cmd == "o") {
                if (is_fixed) {
                    std::lock_guard<std::mutex> lock(g_origin_mutex);
                    origin_lat = sindt.lat;
                    origin_lon = sindt.lon;
                    origin_set = true;
                    std::cout << "\n原点已设置: "
                              << std::setprecision(6) << origin_lat << "°, "
                              << origin_lon << "°" << std::endl;
                } else {
                    std::cout << "\nGPS 未定位，无法设置原点！" << std::endl;
                }
            }

            // ----- status: 查看定位状态 -----
            else if (cmd == "status" || cmd == "s") {
                std::cout << "\n========== 定位状态 ==========" << std::endl;
                std::cout << "  卫星数    : " << sindt.svnum << std::endl;
                std::cout << "  定位状态  : " << (is_fixed ? "固定 " : "未固定 ") << std::endl;
                if (is_fixed) {
                    std::cout << std::setprecision(6);
                    std::cout << "  纬度      : " << sindt.lat << "°" << std::endl;
                    std::cout << "  经度      : " << sindt.lon << "°" << std::endl;
                }
                {
                    std::lock_guard<std::mutex> lock(g_origin_mutex);
                    if (origin_set) {
                        std::cout << std::setprecision(6);
                        std::cout << "  原点纬度  : " << origin_lat << "°" << std::endl;
                        std::cout << "  原点经度  : " << origin_lon << "°" << std::endl;
                        if (is_fixed) {
                            double dist = calcDistance(origin_lat, origin_lon, sindt.lat, sindt.lon);
                            std::cout << "  距原点    : " << std::setprecision(1) << dist << " m" << std::endl;
                        }
                    } else {
                        std::cout << "  原点      : 未设置" << std::endl;
                    }
                }
                std::cout << "================================" << std::endl;
            }

            // ----- reset: 重置原点 -----
            else if (cmd == "reset" || cmd == "r") {
                std::lock_guard<std::mutex> lock(g_origin_mutex);
                origin_set = false;
                origin_lat = 0.0;
                origin_lon = 0.0;
                std::cout << "\n原点已重置" << std::endl;
            }

            // ----- help: 显示帮助 -----
            else if (cmd == "help" || cmd == "h" || cmd == "?") {
                showHelp();
            }

            // ----- quit: 退出 -----
            else if (cmd == "quit" || cmd == "exit" || cmd == "q") {
                running = false;
                std::cout << "\n正在退出..." << std::endl;
            }
        }

        usleep(1000000);  // 主循环 10Hz，快速响应命令
    }

    // ---------- 等待线程退出 ----------
    t1.join();
    t2.join();

    std::cout << "程序已退出" << std::endl;
    return 0;
}