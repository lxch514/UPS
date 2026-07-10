// 多传感器程序: BLR-4000 (测距) + SINDT-485 (姿态/GPS) + AFF500RS (风速)
// 编译: g++ -std=c++11 multi_sensor.cpp -o multi_sensor -pthread
// 运行: ./multi_sensor 

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
#include <vector>
#include <atomic>
#include <chrono>
#include <sstream>

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
                        uint8_t* resp, uint32_t& resp_len, int timeout_ms = 200) {
    if (!serial.is_open()) return false;

    // 清空接收缓冲区
    uint8_t dummy[64];
    while (serial.read(dummy, sizeof(dummy), 10) > 0) {}

    // 发送请求
    int written = serial.write(req, req_len);
    if (written != (int)req_len) {
        return false;
    }

    // 读取响应
    uint8_t buffer[256];
    uint32_t total_read = 0;
    int retry = 0;
    const int max_retry = 4;
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

    // 验证CRC
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
// 5. 传感器数据结构
// ============================================================
struct SensorData {
    // BLR-4000
    float distance_m = 0.0f;
    bool blr_ok = false;
    
    // SINDT-485
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    double latitude = 0.0;
    double longitude = 0.0;
    uint16_t svnum = 0;
    bool angle_ok = false;
    bool gps_ok = false;
    bool sv_ok = false;
    
    // AFF500RS (风速)
    float wind_speed = 0.0f;
    bool wind_ok = false;
    
    // 综合状态
    bool is_fixed() const {
        return gps_ok && sv_ok && (svnum >= 4);
    }
};

// ============================================================
// 6. BLR-4000 读取函数 
// ============================================================
bool readBLR4000(SerialPort& serial, uint8_t addr, float& distance_m) {
    // BLR-4000 使用功能码0x04读取输入寄存器
    // 距离值在寄存器0x0000，单位mm
    uint8_t req[8];
    req[0] = addr;
    req[1] = 0x04;  // 读输入寄存器
    req[2] = 0x00;
    req[3] = 0x00;
    req[4] = 0x00;
    req[5] = 0x01;  // 读取1个寄存器
    uint16_t crc = crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    uint8_t resp[256];
    uint32_t resp_len = 0;
    
    if (!sendModbusRequest(serial, req, 8, resp, resp_len, 500)) {
        return false;
    }

    // 验证响应: 地址 + 功能码 + 数据长度(2字节) + 2字节数据 + 2字节CRC
    if (resp_len < 7 || resp[0] != addr || resp[1] != 0x04 || resp[2] != 0x02) {
        return false;
    }

    uint16_t dist_mm = (resp[3] << 8) | resp[4];
    
    // 0xFFFF 表示超量程或无效数据
    if (dist_mm == 0xFFFF) {
        distance_m = -1.0f;
        return true;  // 返回成功，但距离为-1表示超量程
    }
    
    distance_m = dist_mm / 1000.0f;
    return true;
}

// ============================================================
// 7. SINDT-485 读取函数 - 修正角度显示
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
    if (!sendModbusRequest(serial, req, 8, resp, resp_len, 300)) {
        return false;
    }

    if (resp_len < 11 || resp[0] != addr || resp[1] != 0x03 || resp[2] != 0x06) {
        return false;
    }

    // 读取角度值 (单位: 0.01度)
    int16_t roll_raw = (resp[3] << 8) | resp[4];
    int16_t pitch_raw = (resp[5] << 8) | resp[6];
    int16_t yaw_raw = (resp[7] << 8) | resp[8];
    
    roll = roll_raw / 100.0f;
    pitch = pitch_raw / 100.0f;
    yaw = yaw_raw / 100.0f;
    
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
    if (!sendModbusRequest(serial, req, 8, resp, resp_len, 300)) {
        return false;
    }

    if (resp_len < 13 || resp[0] != addr || resp[1] != 0x03 || resp[2] != 0x08) {
        return false;
    }

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
    if (!sendModbusRequest(serial, req, 8, resp, resp_len, 300)) {
        return false;
    }

    if (resp_len < 7 || resp[0] != addr || resp[1] != 0x03 || resp[2] != 0x02) {
        return false;
    }

    svnum = (resp[3] << 8) | resp[4];
    return true;
}

// ============================================================
// 8. AFF500RS 风速传感器读取函数
// ============================================================
bool readAFF500RS(SerialPort& serial, uint8_t addr, float& wind_speed) {
    // Modbus RTU 读取命令: [地址] 03 00 63 00 01 [CRC]
    uint8_t req[8];
    req[0] = addr;
    req[1] = 0x03;
    req[2] = 0x00;
    req[3] = 0x63;
    req[4] = 0x00;
    req[5] = 0x01;
    uint16_t crc = crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    uint8_t resp[256];
    uint32_t resp_len = 0;
    if (!sendModbusRequest(serial, req, 8, resp, resp_len, 300)) {
        return false;
    }

    if (resp_len < 7 || resp[0] != addr || resp[1] != 0x03 || resp[2] != 0x02) {
        return false;
    }

    uint16_t wind_raw = (resp[3] << 8) | resp[4];
    wind_speed = wind_raw / 1000.0f;
    return true;
}

// ============================================================
// 9. 获取当前时间字符串
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
// 10. 调试：BLR-4000 详细诊断
// ============================================================
void debugBLR4000(SerialPort& serial, uint8_t addr) {
    std::cout << "\n========== BLR-4000 调试模式 (地址: 0x" 
              << std::hex << (int)addr << std::dec << ") ==========" << std::endl;
    
    struct TestCase {
        uint8_t func;
        uint16_t reg_addr;
        uint16_t reg_count;
        std::string desc;
    };
    
    std::vector<TestCase> tests = {
        {0x04, 0x0000, 0x0001, "功能码04, 寄存器0x0000 (输入寄存器)"},
        {0x04, 0x0001, 0x0001, "功能码04, 寄存器0x0001"},
        {0x04, 0x0002, 0x0001, "功能码04, 寄存器0x0002"},
        {0x04, 0x0003, 0x0001, "功能码04, 寄存器0x0003"},
        {0x03, 0x0000, 0x0001, "功能码03, 寄存器0x0000 (保持寄存器)"},
        {0x03, 0x0001, 0x0001, "功能码03, 寄存器0x0001"},
        {0x03, 0x0002, 0x0001, "功能码03, 寄存器0x0002"},
        {0x03, 0x0003, 0x0001, "功能码03, 寄存器0x0003"},
        {0x03, 0x1000, 0x0001, "功能码03, 寄存器0x1000"},
        {0x04, 0x1000, 0x0001, "功能码04, 寄存器0x1000"},
        {0x03, 0x0064, 0x0001, "功能码03, 寄存器0x0064"},
        {0x04, 0x0064, 0x0001, "功能码04, 寄存器0x0064"},
    };
    
    for (const auto& test : tests) {
        uint8_t req[8];
        req[0] = addr;
        req[1] = test.func;
        req[2] = (test.reg_addr >> 8) & 0xFF;
        req[3] = test.reg_addr & 0xFF;
        req[4] = (test.reg_count >> 8) & 0xFF;
        req[5] = test.reg_count & 0xFF;
        uint16_t crc = crc16(req, 6);
        req[6] = crc & 0xFF;
        req[7] = (crc >> 8) & 0xFF;
        
        std::cout << "\n测试: " << test.desc << std::endl;
        std::cout << "发送: ";
        for (int i = 0; i < 8; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)req[i] << " ";
        }
        std::cout << std::dec << std::endl;
        
        uint8_t resp[256];
        uint32_t resp_len = 0;
        
        if (sendModbusRequest(serial, req, 8, resp, resp_len, 300)) {
            std::cout << "响应 (" << resp_len << " 字节): ";
            for (uint32_t i = 0; i < resp_len; i++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)resp[i] << " ";
            }
            std::cout << std::dec << std::endl;
            
            if (resp_len >= 5) {
                if (resp[1] & 0x80) {
                    std::cout << "错误代码: 0x" << std::hex << (int)resp[2] << std::dec << std::endl;
                    switch(resp[2]) {
                        case 0x01: std::cout << "  错误: 非法功能码" << std::endl; break;
                        case 0x02: std::cout << "  错误: 非法数据地址" << std::endl; break;
                        case 0x03: std::cout << "  错误: 非法数据值" << std::endl; break;
                        case 0x04: std::cout << "  错误: 从设备故障" << std::endl; break;
                        default: std::cout << "  未知错误" << std::endl; break;
                    }
                } else if (resp[2] == 0x02) {
                    uint16_t value = (resp[3] << 8) | resp[4];
                    std::cout << "原始值: 0x" << std::hex << value << std::dec << " = " << value;
                    if (value != 0xFFFF) {
                        float dist = value / 1000.0f;
                        std::cout << " -> 距离: " << std::fixed << std::setprecision(3) << dist << "m";
                    } else {
                        std::cout << " (超量程)";
                    }
                    std::cout << std::endl;
                }
            }
        } else {
            std::cout << "无响应或响应超时" << std::endl;
        }
        
        usleep(100000);
    }
    std::cout << "==============================================" << std::endl;
}

// ============================================================
// 11. 显示帮助信息
// ============================================================
void showHelp() {
    std::cout << "\n========== 交互命令 ==========" << std::endl;
    std::cout << "  origin  (o)  : 设置当前位置为原点 (需GPS固定)" << std::endl;
    std::cout << "  status  (s)  : 查看完整定位状态" << std::endl;
    std::cout << "  reset   (r)  : 重置原点" << std::endl;
    std::cout << "  debug   (d)  : 运行BLR-4000调试" << std::endl;
    std::cout << "  help    (h)  : 显示本帮助信息" << std::endl;
    std::cout << "  quit    (q)  : 退出程序" << std::endl;
    std::cout << "================================" << std::endl;
}

// ============================================================
// 12. 读取所有传感器数据 (轮询)
// ============================================================
void readAllSensors(SerialPort& serial, 
                    uint8_t addr_BLR, uint8_t addr_SINDT, uint8_t addr_WIND,
                    SensorData& data) {
    // 读取 BLR-4000 距离
    data.blr_ok = readBLR4000(serial, addr_BLR, data.distance_m);
    
    // 读取 SINDT-485 角度
    data.angle_ok = readSINDT_Angle(serial, addr_SINDT, data.roll, data.pitch, data.yaw);
    
    // 读取 SINDT-485 GPS
    data.gps_ok = readSINDT_GPS(serial, addr_SINDT, data.latitude, data.longitude);
    
    // 读取 SINDT-485 卫星数
    data.sv_ok = readSINDT_SV(serial, addr_SINDT, data.svnum);
    
    // 读取 AFF500RS 风速
    data.wind_ok = readAFF500RS(serial, addr_WIND, data.wind_speed);
}

// ============================================================
// 13. 显示传感器数据 - 修正角度显示格式
// ============================================================
void displayData(const SensorData& data, bool origin_set, 
                 double origin_lat, double origin_lon) {
    std::cout << "[" << getTimeStr() << "] ";
    
    // 距离
    if (data.blr_ok) {
        if (data.distance_m < 0) {
            std::cout << "距离: 超量程 ";
        } else {
            std::cout << "距离: " << std::fixed << std::setprecision(3) 
                      << data.distance_m << "m ";
        }
    } else {
        std::cout << "距离: -- ";
    }
    
    std::cout << "| ";
    
    // 角度 - 修正格式，避免科学计数法
    if (data.angle_ok) {
        // 使用 fixed 和 setprecision(1) 确保显示为小数而非科学计数法
        std::cout << "角度: R=" << std::fixed << std::setprecision(1) << data.roll
                  << "° P=" << std::fixed << std::setprecision(1) << data.pitch 
                  << "° Y=" << std::fixed << std::setprecision(1) << data.yaw << "° ";
    } else {
        std::cout << "角度: -- ";
    }
    
    std::cout << "| ";
    
    // 风速
    if (data.wind_ok) {
        std::cout << "风速: " << std::fixed << std::setprecision(3) 
                  << data.wind_speed << "m/s ";
    } else {
        std::cout << "风速: -- ";
    }
    
    std::cout << "| ";
    
    // GPS
    bool is_fixed = data.is_fixed();
    if (is_fixed) {
        std::cout << "GPS: 固定(" << data.svnum << "星) "
                  << std::fixed << std::setprecision(6) << data.latitude << "°, " 
                  << data.longitude << "°";
    } else if (data.sv_ok && data.svnum > 0) {
        std::cout << "GPS: 搜星(" << data.svnum << "星)";
    } else {
        std::cout << "GPS: 未定位";
    }
    
    // 距原点距离
    if (origin_set && is_fixed) {
        double dist = calcDistance(origin_lat, origin_lon, 
                                   data.latitude, data.longitude);
        std::cout << " | 距原点: " << std::fixed << std::setprecision(1) << dist << "m";
    } else if (origin_set) {
        std::cout << " | 距原点: GPS未定位";
    }
    
    std::cout << std::endl;
}

// ============================================================
// 14. 打印使用说明
// ============================================================
void print_usage(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  [port]        : serial device, default /dev/ttyS0" << std::endl;
    std::cout << "  [baudrate]    : default 9600" << std::endl;
    std::cout << "  [addr_BLR]    : BLR-4000 address, default 0x53" << std::endl;
    std::cout << "  [addr_SINDT]  : SINDT-485 address, default 0x65" << std::endl;
    std::cout << "  [addr_WIND]   : AFF500RS address, default 0x31" << std::endl;
    std::cout << "  --debug, -d   : 开启调试模式" << std::endl;
    std::cout << "  -h, --help    : 显示帮助信息" << std::endl;
    std::cout << std::endl;
    showHelp();
    std::cout << std::endl;
    std::cout << "Example: " << prog_name << " /dev/ttyS0 9600 0x53 0x65 0x31" << std::endl;
    std::cout << "Example: " << prog_name << " /dev/ttyS0 9600 0x53 0x65 0x31 --debug" << std::endl;
}

// ============================================================
// 15. 主程序
// ============================================================
static volatile bool running = true;
static double origin_lat = 0.0, origin_lon = 0.0;
static bool origin_set = false;

void signal_handler(int sig) {
    (void)sig;
    running = false;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    
    // ---------- 解析参数 ----------
    std::string port = "/dev/ttyS0";
    uint32_t baudrate = 9600;
    uint8_t addr_BLR = 0x53;
    uint8_t addr_SINDT = 0x65;
    uint8_t addr_WIND = 0x31;
    bool debug_mode = false;
    
    if (argc >= 2) {
        if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
            print_usage(argv[0]);
            return 0;
        }
        port = argv[1];
    }
    if (argc >= 3) baudrate = std::stoul(argv[2]);
    if (argc >= 4) {
        std::string s = argv[3];
        addr_BLR = (s.find("0x") == 0) ? (uint8_t)std::stoul(s, nullptr, 16) : (uint8_t)std::stoul(s);
    }
    if (argc >= 5) {
        std::string s = argv[4];
        addr_SINDT = (s.find("0x") == 0) ? (uint8_t)std::stoul(s, nullptr, 16) : (uint8_t)std::stoul(s);
    }
    if (argc >= 6) {
        std::string s = argv[5];
        addr_WIND = (s.find("0x") == 0) ? (uint8_t)std::stoul(s, nullptr, 16) : (uint8_t)std::stoul(s);
    }
    
    // 检查调试参数
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--debug" || std::string(argv[i]) == "-d") {
            debug_mode = true;
        }
    }
    
    std::cout << "==================================================" << std::endl;
    std::cout << "  BLR-4000 + SINDT-485 + AFF500RS 三传感器系统" << std::endl;
    std::cout << "  串口: " << port << "  波特率: " << baudrate << std::endl;
    std::cout << "  BLR-4000  地址: 0x" << std::hex << (int)addr_BLR << std::dec << std::endl;
    std::cout << "  SINDT-485 地址: 0x" << std::hex << (int)addr_SINDT << std::dec << std::endl;
    std::cout << "  AFF500RS  地址: 0x" << std::hex << (int)addr_WIND << std::dec << std::endl;
    if (debug_mode) {
        std::cout << "  ** 调试模式已开启 **" << std::endl;
    }
    std::cout << "==================================================" << std::endl;
    
    // ---------- 打开串口 ----------
    SerialPort serial;
    if (!serial.open(port, baudrate)) {
        std::cerr << "无法打开串口 " << port << std::endl;
        return -1;
    }
    
    // ---------- 调试模式 ----------
    if (debug_mode) {
        debugBLR4000(serial, addr_BLR);
        std::cout << "\n调试完成，按 Enter 继续..." << std::endl;
        std::cin.get();
    }
    
    // ---------- 显示交互命令帮助 ----------
    showHelp();
    std::cout << "按 Ctrl+C 或输入 'q' 停止" << std::endl << std::endl;
    
    // ---------- 主循环 ----------
    SensorData data;
    const int READ_INTERVAL_US = 1000000;  // 1秒
    auto last_read_time = std::chrono::steady_clock::now();
    
    while (running) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_read_time).count();
        
        if (elapsed >= READ_INTERVAL_US) {
            // 读取所有传感器
            readAllSensors(serial, addr_BLR, addr_SINDT, addr_WIND, data);
            last_read_time = now;
            
            // 显示数据
            displayData(data, origin_set, origin_lat, origin_lon);
        }
        
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
                if (data.is_fixed()) {
                    origin_lat = data.latitude;
                    origin_lon = data.longitude;
                    origin_set = true;
                    std::cout << "\n原点已设置: "
                              << std::fixed << std::setprecision(6) << origin_lat << "°, "
                              << origin_lon << "°" << std::endl;
                } else {
                    std::cout << "\nGPS 未定位，无法设置原点！" << std::endl;
                }
            }
            
            // ----- status: 查看定位状态 -----
            else if (cmd == "status" || cmd == "s") {
                std::cout << "\n========== 定位状态 ==========" << std::endl;
                std::cout << "  卫星数    : " << data.svnum << std::endl;
                std::cout << "  定位状态  : " << (data.is_fixed() ? "固定 " : "未固定 ") << std::endl;
                if (data.is_fixed()) {
                    std::cout << std::fixed << std::setprecision(6);
                    std::cout << "  纬度      : " << data.latitude << "°" << std::endl;
                    std::cout << "  经度      : " << data.longitude << "°" << std::endl;
                }
                if (origin_set) {
                    std::cout << std::fixed << std::setprecision(6);
                    std::cout << "  原点纬度  : " << origin_lat << "°" << std::endl;
                    std::cout << "  原点经度  : " << origin_lon << "°" << std::endl;
                    if (data.is_fixed()) {
                        double dist = calcDistance(origin_lat, origin_lon, 
                                                   data.latitude, data.longitude);
                        std::cout << "  距原点    : " << std::fixed << std::setprecision(1) << dist << " m" << std::endl;
                    }
                } else {
                    std::cout << "  原点      : 未设置" << std::endl;
                }
                std::cout << "================================" << std::endl;
            }
            
            // ----- reset: 重置原点 -----
            else if (cmd == "reset" || cmd == "r") {
                origin_set = false;
                origin_lat = 0.0;
                origin_lon = 0.0;
                std::cout << "\n原点已重置" << std::endl;
            }
            
            // ----- debug: 运行调试 -----
            else if (cmd == "debug" || cmd == "d") {
                debugBLR4000(serial, addr_BLR);
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
        
        usleep(10000);
    }
    
    serial.close();
    std::cout << "程序已退出" << std::endl;
    return 0;
}