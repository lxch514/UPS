// FD07-3超声波测距传感器
// 编译: g++ -std=c++11 FD07.cpp -o FD07
// 运行: ./FD07

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <csignal>
#include <sys/select.h>
#include <cerrno>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <sstream>

// ============================================================
// 全局停止标志 (使用原子变量)
// ============================================================
std::atomic<bool> stop_flag(false);

void signal_handler(int) {
    stop_flag = true;
    std::cout << "\n收到中断信号，正在退出..." << std::endl;
}

// ============================================================
// FD07-3 传感器类
// ============================================================
class FD07Sensor {
private:
    int fd;                          // 串口文件描述符
    std::string port;                // 串口设备路径
    int baudrate;                    // 波特率
    uint8_t device_addr;             // 设备地址 (默认0x21)
    bool is_open;                    // 串口打开状态

    // CRC16 (Modbus) 校验计算
    uint16_t calculate_crc16(const std::vector<uint8_t>& data) {
        uint16_t crc = 0xFFFF;
        for (uint8_t byte : data) {
            crc ^= byte;
            for (int i = 0; i < 8; i++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    // 配置串口参数 (8N1)
    bool configure_serial() {
        struct termios options;

        if (tcgetattr(fd, &options) != 0) {
            std::cerr << "获取串口配置失败: " << strerror(errno) << std::endl;
            return false;
        }

        // 设置波特率
        speed_t baud;
        switch (baudrate) {
            case 2400:   baud = B2400;   break;
            case 4800:   baud = B4800;   break;
            case 9600:   baud = B9600;   break;
            case 19200:  baud = B19200;  break;
            case 38400:  baud = B38400;  break;
            case 57600:  baud = B57600;  break;
            case 115200: baud = B115200; break;
            default:
                std::cerr << "不支持的波特率: " << baudrate << std::endl;
                return false;
        }

        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        // 8N1: 8数据位, 无校验, 1停止位
        options.c_cflag &= ~PARENB;      // 无校验
        options.c_cflag &= ~CSTOPB;      // 1停止位
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;          // 8数据位

        options.c_cflag &= ~CRTSCTS;     // 无硬件流控
        options.c_cflag |= CREAD | CLOCAL;

        // 原始模式
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;

        // 输入标志
        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        // 超时设置: VMIN=0, VTIME=10 (1秒超时)
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10;

        tcflush(fd, TCIFLUSH);
        if (tcsetattr(fd, TCSANOW, &options) != 0) {
            std::cerr << "设置串口配置失败: " << strerror(errno) << std::endl;
            return false;
        }

        tcflush(fd, TCIOFLUSH);
        return true;
    }

    // 发送Modbus命令并接收响应
    bool send_command_and_receive(const std::vector<uint8_t>& cmd,
                                  std::vector<uint8_t>& response,
                                  int expected_min_len = 7,
                                  int timeout_ms = 1000) {
        if (!is_open || fd == -1) {
            std::cerr << "串口未打开" << std::endl;
            return false;
        }

        response.clear();
        tcflush(fd, TCIOFLUSH);

        ssize_t bytes_written = write(fd, cmd.data(), cmd.size());
        if (bytes_written != static_cast<ssize_t>(cmd.size())) {
            std::cerr << "发送命令失败: " << strerror(errno) << std::endl;
            return false;
        }

        tcdrain(fd);

        // 使用select等待数据
        fd_set read_fds;
        struct timeval timeout;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;

        int ret = select(fd + 1, &read_fds, NULL, NULL, &timeout);
        if (ret < 0) {
            std::cerr << "select错误: " << strerror(errno) << std::endl;
            return false;
        } else if (ret == 0) {
            std::cerr << "等待响应超时" << std::endl;
            return false;
        }

        // 读取数据
        uint8_t buffer[256];
        int attempts = 0;
        const int max_attempts = 10;

        while (attempts < max_attempts) {
            ssize_t bytes_read = read(fd, buffer, sizeof(buffer));
            if (bytes_read > 0) {
                response.insert(response.end(), buffer, buffer + bytes_read);
                if (static_cast<int>(response.size()) >= expected_min_len) {
                    break;
                }
            } else if (bytes_read == 0) {
                break;
            } else {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "读取错误: " << strerror(errno) << std::endl;
                    return false;
                }
            }
            attempts++;
            usleep(10000);
        }

        if (response.empty()) {
            std::cerr << "未接收到响应数据" << std::endl;
            return false;
        }

        return true;
    }

    // 校验CRC
    bool verify_crc(const std::vector<uint8_t>& response) {
        if (response.size() < 3) {
            return false;
        }

        std::vector<uint8_t> data_without_crc(response.begin(), response.end() - 2);
        uint16_t recv_crc = (response[response.size() - 1] << 8) | response[response.size() - 2];
        uint16_t calc_crc = calculate_crc16(data_without_crc);

        if (recv_crc != calc_crc) {
            std::cerr << "CRC校验失败: 接收 0x" << std::hex << recv_crc
                      << ", 计算 0x" << calc_crc << std::dec << std::endl;
            return false;
        }
        return true;
    }

    // 解析读寄存器响应
    int parse_read_response(const std::vector<uint8_t>& response) {
        if (response.size() < 7) {
            std::cerr << "响应数据长度不足: " << response.size() << " 字节 (需要至少7字节)" << std::endl;
            return -1;
        }

        if (response[0] != device_addr) {
            std::cerr << "设备地址不匹配: 期望 0x" << std::hex << static_cast<int>(device_addr)
                      << ", 收到 0x" << static_cast<int>(response[0]) << std::dec << std::endl;
            return -1;
        }

        // 检查异常响应 (功能码最高位为1)
        if (response[1] == (0x03 | 0x80)) {
            std::cerr << "设备返回异常码: 0x" << std::hex << static_cast<int>(response[2]) << std::dec << std::endl;
            switch(response[2]) {
                case 0x01: std::cerr << "  异常: 非法功能码" << std::endl; break;
                case 0x02: std::cerr << "  异常: 非法数据地址" << std::endl; break;
                case 0x03: std::cerr << "  异常: 非法数据值" << std::endl; break;
                case 0x04: std::cerr << "  异常: 从设备故障" << std::endl; break;
                default: std::cerr << "  未知异常" << std::endl; break;
            }
            return -1;
        }

        if (response[1] != 0x03) {
            std::cerr << "功能码错误: 期望 0x03, 收到 0x" << std::hex 
                      << static_cast<int>(response[1]) << std::dec << std::endl;
            return -1;
        }

        uint8_t byte_count = response[2];
        if (byte_count != 2) {
            std::cerr << "数据字节数异常: " << static_cast<int>(byte_count) << " (期望2)" << std::endl;
            return -1;
        }

        if (!verify_crc(response)) {
            return -1;
        }

        int value = (response[3] << 8) | response[4];
        return value;
    }

    // 解析写单寄存器响应
    bool parse_write_response(const std::vector<uint8_t>& response,
                              uint16_t expected_reg,
                              uint16_t expected_value) {
        if (response.size() < 8) {
            std::cerr << "写响应长度不足: " << response.size() << " 字节 (需要至少8字节)" << std::endl;
            return false;
        }

        if (response[0] != device_addr) {
            std::cerr << "设备地址不匹配: 期望 0x" << std::hex << static_cast<int>(device_addr)
                      << ", 收到 0x" << static_cast<int>(response[0]) << std::dec << std::endl;
            return false;
        }

        // 检查异常响应
        if (response[1] == (0x06 | 0x80)) {
            std::cerr << "设备返回异常码: 0x" << std::hex << static_cast<int>(response[2]) << std::dec << std::endl;
            return false;
        }

        if (response[1] != 0x06) {
            std::cerr << "功能码错误: 期望 0x06, 收到 0x" << std::hex 
                      << static_cast<int>(response[1]) << std::dec << std::endl;
            return false;
        }

        if (!verify_crc(response)) {
            return false;
        }

        uint16_t reg = (response[2] << 8) | response[3];
        uint16_t value = (response[4] << 8) | response[5];

        if (reg != expected_reg) {
            std::cerr << "寄存器地址不匹配: 期望 0x" << std::hex << expected_reg
                      << ", 收到 0x" << reg << std::dec << std::endl;
            return false;
        }

        if (value != expected_value) {
            std::cerr << "寄存器值不匹配: 期望 0x" << std::hex << expected_value
                      << ", 收到 0x" << value << std::dec << std::endl;
            return false;
        }

        return true;
    }

public:
    // 构造函数：默认地址 0x21
    FD07Sensor(const std::string& port_name = "/dev/ttyS3", int baud = 9600, uint8_t addr = 0x21)
        : fd(-1), port(port_name), baudrate(baud), device_addr(addr), is_open(false) {}

    ~FD07Sensor() {
        close_port();
    }

    // 打开串口
    bool open_port() {
        fd = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (fd == -1) {
            std::cerr << "无法打开串口: " << port << std::endl;
            std::cerr << "错误信息: " << strerror(errno) << std::endl;
            std::cerr << "提示: 请检查串口权限，例如: sudo chmod 666 " << port << std::endl;
            return false;
        }

        if (!configure_serial()) {
            ::close(fd);
            fd = -1;
            return false;
        }

        is_open = true;
        std::cout << "✓ 串口 " << port << " 打开成功" << std::endl;
        std::cout << "  波特率: " << baudrate << " bps" << std::endl;
        std::cout << "  设备地址: 0x" << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(device_addr) << std::dec << std::endl;
        return true;
    }

    // 关闭串口
    void close_port() {
        if (fd != -1) {
            ::close(fd);
            fd = -1;
            is_open = false;
            std::cout << "✓ 串口已关闭" << std::endl;
        }
    }

    // 检查串口是否打开
    bool is_port_open() const {
        return is_open;
    }

    // 获取当前配置
    void get_config(std::string& out_port, int& out_baud, uint8_t& out_addr) {
        out_port = port;
        out_baud = baudrate;
        out_addr = device_addr;
    }

    // 更新设备地址
    void set_device_addr(uint8_t addr) {
        device_addr = addr;
    }

    // 更新波特率
    void set_baudrate(int baud) {
        baudrate = baud;
    }

    // 更新串口
    void set_port(const std::string& new_port) {
        port = new_port;
    }

    // 构建读命令 0x03
    std::vector<uint8_t> build_read_command(uint16_t reg_addr = 0x0001,
                                            uint16_t reg_count = 0x0001) {
        std::vector<uint8_t> cmd = {
            device_addr,
            0x03,
            static_cast<uint8_t>((reg_addr >> 8) & 0xFF),
            static_cast<uint8_t>(reg_addr & 0xFF),
            static_cast<uint8_t>((reg_count >> 8) & 0xFF),
            static_cast<uint8_t>(reg_count & 0xFF)
        };

        uint16_t crc = calculate_crc16(cmd);
        cmd.push_back(crc & 0xFF);
        cmd.push_back((crc >> 8) & 0xFF);
        return cmd;
    }

    // 构建写单寄存器命令 0x06
    std::vector<uint8_t> build_write_command(uint16_t reg_addr, uint16_t reg_value) {
        std::vector<uint8_t> cmd = {
            device_addr,
            0x06,
            static_cast<uint8_t>((reg_addr >> 8) & 0xFF),
            static_cast<uint8_t>(reg_addr & 0xFF),
            static_cast<uint8_t>((reg_value >> 8) & 0xFF),
            static_cast<uint8_t>(reg_value & 0xFF)
        };

        uint16_t crc = calculate_crc16(cmd);
        cmd.push_back(crc & 0xFF);
        cmd.push_back((crc >> 8) & 0xFF);
        return cmd;
    }

    // 读取距离
    // reg_addr: 0x0001 = 实际值, 0x0002 = 处理值
    int read_distance(uint16_t reg_addr = 0x0001) {
        std::vector<uint8_t> cmd = build_read_command(reg_addr, 0x0001);
        std::vector<uint8_t> response;

        if (!send_command_and_receive(cmd, response, 7, 1000)) {
            return -1;
        }

        return parse_read_response(response);
    }

    // 连续读取距离 (带统计)
    void continuous_read(uint16_t reg_addr = 0x0001, double interval_sec = 0.5) {
        stop_flag = false;
        
        std::cout << "\n开始连续读取距离数据..." << std::endl;
        std::cout << "模式: " << ((reg_addr == 0x0001) ? "实际值" : "处理值") << std::endl;
        std::cout << "间隔: " << interval_sec << " 秒" << std::endl;
        std::cout << "按 Ctrl+C 停止\n" << std::endl;

        int count = 0;
        int success = 0;
        std::vector<double> distances;

        while (!stop_flag.load()) {
            count++;
            int distance_mm = read_distance(reg_addr);
            
            if (distance_mm >= 0) {
                success++;
                double distance_cm = distance_mm / 10.0;
                double distance_m = distance_mm / 1000.0;
                distances.push_back(distance_m);

                std::cout << "[" << std::setw(3) << count << "] ✓ 距离: " 
                          << distance_mm << " mm = "
                          << std::fixed << std::setprecision(1) << distance_cm << " cm = "
                          << std::setprecision(3) << distance_m << " m" << std::endl;
            } else {
                std::cout << "[" << std::setw(3) << count << "] ✗ 读取失败" << std::endl;
            }

            // 等待间隔
            if (!stop_flag.load() && count > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(
                    static_cast<int>(interval_sec * 1000)));
            }
        }

        // 打印统计
        std::cout << "\n========== 统计信息 ==========" << std::endl;
        std::cout << "总读取次数: " << count << std::endl;
        std::cout << "成功次数: " << success << std::endl;
        if (count > 0) {
            std::cout << "成功率: " << std::fixed << std::setprecision(1) 
                      << (success * 100.0 / count) << "%" << std::endl;
        }
        if (!distances.empty()) {
            double sum = 0, max = distances[0], min = distances[0];
            for (double d : distances) {
                sum += d;
                if (d > max) max = d;
                if (d < min) min = d;
            }
            double avg = sum / distances.size();
            std::cout << "\n距离统计:" << std::endl;
            std::cout << "  平均值: " << std::fixed << std::setprecision(3) << avg << " m" << std::endl;
            std::cout << "  最大值: " << max << " m" << std::endl;
            std::cout << "  最小值: " << min << " m" << std::endl;
            std::cout << "  波动: " << (max - min) << " m" << std::endl;
        }
        std::cout << "================================" << std::endl;
    }

    // 写寄存器
    bool write_register(uint16_t reg_addr, uint16_t reg_value) {
        std::vector<uint8_t> cmd = build_write_command(reg_addr, reg_value);
        std::vector<uint8_t> response;

        if (!send_command_and_receive(cmd, response, 8, 1000)) {
            return false;
        }

        return parse_write_response(response, reg_addr, reg_value);
    }

    // 修改设备地址
    // 寄存器: 0x0003, 范围: 0x01~0xFC
    bool change_device_id(uint8_t new_addr) {
        if (new_addr < 0x01 || new_addr > 0xFC) {
            std::cerr << "设备地址超出范围，合法范围: 0x01 ~ 0xFC" << std::endl;
            return false;
        }

        std::cout << "正在修改设备地址: 0x" << std::hex
                  << std::setw(2) << std::setfill('0') << static_cast<int>(device_addr)
                  << " -> 0x" << std::setw(2) << static_cast<int>(new_addr) << std::dec << std::endl;

        if (!write_register(0x0003, new_addr)) {
            std::cerr << "✗ 修改设备地址失败" << std::endl;
            return false;
        }

        set_device_addr(new_addr);
        std::cout << "✓ 设备地址修改成功，新地址: 0x"
                  << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(new_addr) << std::dec << std::endl;
        return true;
    }

    // 修改波特率
    // 寄存器映射:
    // 0x0010 -> 2400, 0x0011 -> 4800, 0x0012 -> 9600
    // 0x0013 -> 19200, 0x0014 -> 38400, 0x0015 -> 57600, 0x0016 -> 115200
    bool change_baudrate(int new_baud) {
        uint16_t reg_addr = 0;
        switch (new_baud) {
            case 2400:   reg_addr = 0x0010; break;
            case 4800:   reg_addr = 0x0011; break;
            case 9600:   reg_addr = 0x0012; break;
            case 19200:  reg_addr = 0x0013; break;
            case 38400:  reg_addr = 0x0014; break;
            case 57600:  reg_addr = 0x0015; break;
            case 115200: reg_addr = 0x0016; break;
            default:
                std::cerr << "不支持修改为该波特率: " << new_baud << std::endl;
                std::cerr << "支持的波特率: 2400, 4800, 9600, 19200, 38400, 57600, 115200" << std::endl;
                return false;
        }

        std::cout << "正在修改波特率: " << baudrate << " -> " << new_baud << std::endl;
        std::cout << "寄存器: 0x" << std::hex << std::setw(4) << std::setfill('0') 
                  << reg_addr << std::dec << std::endl;

        // 按说明书，写入值固定为 0x0001
        if (!write_register(reg_addr, 0x0001)) {
            std::cerr << "✗ 修改波特率失败" << std::endl;
            return false;
        }

        std::cout << "✓ 波特率修改命令发送成功" << std::endl;
        std::cout << "注意: 设备波特率已改变，当前串口参数仍是旧波特率。" << std::endl;
        std::cout << "请重新打开程序后，使用新波特率连接。" << std::endl;

        return true;
    }

    // 获取设备信息
    void print_info() {
        std::cout << "\n========== FD07-3 传感器信息 ==========" << std::endl;
        std::cout << "  串口: " << port << std::endl;
        std::cout << "  波特率: " << baudrate << " bps" << std::endl;
        std::cout << "  设备地址: 0x" << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(device_addr) << std::dec << std::endl;
        std::cout << "  量程: 3~300 cm (30~3000 mm)" << std::endl;
        std::cout << "  精度: ±(0.5 + 0.006*距离) cm" << std::endl;
        std::cout << "  工作温度: -15~65°C" << std::endl;
        std::cout << "  盲区: 3 cm" << std::endl;
        std::cout << "  工作间隔: >100ms (实际值), >300ms (处理值)" << std::endl;
        std::cout << "========================================" << std::endl;
    }

    // 测试通信
    bool test_communication() {
        std::cout << "测试通信中..." << std::endl;
        int result = read_distance(0x0001);
        if (result >= 0) {
            std::cout << "✓ 通信正常！距离: " << result << " mm" << std::endl;
            return true;
        } else {
            std::cout << "✗ 通信失败，请检查连接和配置" << std::endl;
            return false;
        }
    }
};

// ============================================================
// 交互式菜单
// ============================================================
class Menu {
private:
    FD07Sensor sensor;
    bool running;

    void clear_screen() {
        std::cout << "\033[2J\033[1;1H";  // ANSI清屏
    }

    void show_header() {
        std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║           FD07-3 超声波测距传感器 交互式工具               ║" << std::endl;
        std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
        
        std::string port;
        int baud;
        uint8_t addr;
        sensor.get_config(port, baud, addr);
        
        std::cout << "当前配置:" << std::endl;
        std::cout << "  串口: " << port << std::endl;
        std::cout << "  波特率: " << baud << " bps" << std::endl;
        std::cout << "  设备地址: 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << (int)addr << std::dec << std::endl;
        std::cout << std::endl;
    }

    void show_menu() {
        std::cout << "┌─────────────────────────────────────────────────────────────┐" << std::endl;
        std::cout << "│  1. 单次读取距离 (实际值)                                  │" << std::endl;
        std::cout << "│  2. 单次读取距离 (处理值)                                  │" << std::endl;
        std::cout << "│  3. 连续读取距离 (实际值)                                  │" << std::endl;
        std::cout << "│  4. 连续读取距离 (处理值)                                  │" << std::endl;
        std::cout << "│  5. 修改设备地址                                           │" << std::endl;
        std::cout << "│  6. 修改波特率                                             │" << std::endl;
        std::cout << "│  7. 修改串口                                               │" << std::endl;
        std::cout << "│  8. 测试通信                                               │" << std::endl;
        std::cout << "│  9. 显示传感器信息                                         │" << std::endl;
        std::cout << "│  0. 退出程序                                               │" << std::endl;
        std::cout << "└─────────────────────────────────────────────────────────────┘" << std::endl;
        std::cout << "请选择操作 [0-9]: ";
    }

    void wait_for_enter() {
        std::cout << "\n按 Enter 继续...";
        std::cin.get();
        std::cin.get();
    }

    void read_single_actual() {
        if (!sensor.is_port_open()) {
            if (!sensor.open_port()) return;
        }
        std::cout << "\n读取实际值..." << std::endl;
        int distance = sensor.read_distance(0x0001);
        if (distance >= 0) {
            double distance_cm = distance / 10.0;
            double distance_m = distance / 1000.0;
            std::cout << "\n✓ 距离: " << distance << " mm" << std::endl;
            std::cout << "       = " << std::fixed << std::setprecision(1) 
                      << distance_cm << " cm" << std::endl;
            std::cout << "       = " << std::setprecision(3) << distance_m << " m" << std::endl;
        } else {
            std::cout << "\n✗ 读取失败" << std::endl;
        }
    }

    void read_single_processed() {
        if (!sensor.is_port_open()) {
            if (!sensor.open_port()) return;
        }
        std::cout << "\n读取处理值..." << std::endl;
        int distance = sensor.read_distance(0x0002);
        if (distance >= 0) {
            double distance_cm = distance / 10.0;
            double distance_m = distance / 1000.0;
            std::cout << "\n✓ 距离: " << distance << " mm" << std::endl;
            std::cout << "       = " << std::fixed << std::setprecision(1) 
                      << distance_cm << " cm" << std::endl;
            std::cout << "       = " << std::setprecision(3) << distance_m << " m" << std::endl;
        } else {
            std::cout << "\n✗ 读取失败" << std::endl;
        }
    }

    void read_continuous_actual() {
        if (!sensor.is_port_open()) {
            if (!sensor.open_port()) return;
        }
        double interval;
        std::cout << "请输入读取间隔(秒, 默认0.5): ";
        std::string input;
        std::cin >> input;
        if (input.empty()) interval = 0.5;
        else interval = std::atof(input.c_str());
        if (interval < 0.1) interval = 0.1;
        sensor.continuous_read(0x0001, interval);
    }

    void read_continuous_processed() {
        if (!sensor.is_port_open()) {
            if (!sensor.open_port()) return;
        }
        double interval;
        std::cout << "请输入读取间隔(秒, 默认0.5): ";
        std::string input;
        std::cin >> input;
        if (input.empty()) interval = 0.5;
        else interval = std::atof(input.c_str());
        if (interval < 0.3) interval = 0.3;
        sensor.continuous_read(0x0002, interval);
    }

    void change_address() {
        if (!sensor.is_port_open()) {
            if (!sensor.open_port()) return;
        }
        std::cout << "请输入新地址 (十六进制, 如 0x22): ";
        std::string input;
        std::cin >> input;
        uint8_t new_addr = static_cast<uint8_t>(std::strtoul(input.c_str(), nullptr, 0));
        sensor.change_device_id(new_addr);
    }

    void change_baudrate() {
        if (!sensor.is_port_open()) {
            if (!sensor.open_port()) return;
        }
        std::cout << "支持的波特率: 2400, 4800, 9600, 19200, 38400, 57600, 115200" << std::endl;
        std::cout << "请输入新波特率: ";
        int new_baud;
        std::cin >> new_baud;
        sensor.change_baudrate(new_baud);
    }

    void change_port() {
        std::cout << "请输入串口设备 (如 /dev/ttyS3): ";
        std::string new_port;
        std::cin >> new_port;
        if (sensor.is_port_open()) {
            sensor.close_port();
        }
        sensor.set_port(new_port);
        std::cout << "✓ 串口已修改为: " << new_port << std::endl;
    }

    void test_communication() {
        if (!sensor.is_port_open()) {
            if (!sensor.open_port()) return;
        }
        sensor.test_communication();
    }

public:
    Menu() : sensor("/dev/ttyS3", 9600, 0x21), running(true) {
        signal(SIGINT, signal_handler);
    }

    void run() {
        while (running) {
            std::cout << "\n";
            show_header();
            show_menu();
            
            std::string choice;
            std::cin >> choice;
            std::cin.ignore();  // 清除换行符

            switch (choice[0]) {
                case '1':
                    read_single_actual();
                    wait_for_enter();
                    break;
                case '2':
                    read_single_processed();
                    wait_for_enter();
                    break;
                case '3':
                    read_continuous_actual();
                    break;
                case '4':
                    read_continuous_processed();
                    break;
                case '5':
                    change_address();
                    wait_for_enter();
                    break;
                case '6':
                    change_baudrate();
                    wait_for_enter();
                    break;
                case '7':
                    change_port();
                    wait_for_enter();
                    break;
                case '8':
                    test_communication();
                    wait_for_enter();
                    break;
                case '9':
                    sensor.print_info();
                    wait_for_enter();
                    break;
                case '0':
                    running = false;
                    if (sensor.is_port_open()) {
                        sensor.close_port();
                    }
                    std::cout << "程序已退出" << std::endl;
                    break;
                default:
                    std::cout << "无效选择，请重试" << std::endl;
                    wait_for_enter();
                    break;
            }
        }
    }
};

// ============================================================
// 主函数
// ============================================================
int main() {
    Menu menu;
    menu.run();
    return 0;
}