// 风速传感器数据读取程序 (Modbus RTU)
// 编译: g++ -std=c++11 AFF500RS.cpp -o AFF500RS
// 运行: ./AFF500RS

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <ctime>
#include <cmath>
#include <map>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>
#include <cstring>
#include <sstream>

using namespace std;

// 风力等级结构体
struct WindLevel {
    int level;
    string name;
    string description;
};

class WindSpeedSensor {
private:
    string port;
    int baudrate;
    int timeout_ms;
    int serial_fd;
    uint8_t device_addr;  // 设备地址
    
    // 获取当前时间戳字符串
    string getTimestamp() {
        auto now = chrono::system_clock::now();
        auto now_c = chrono::system_clock::to_time_t(now);
        auto now_ms = chrono::duration_cast<chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        char buffer[32];
        strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localtime(&now_c));
        
        char result[64];
        snprintf(result, sizeof(result), "%s.%03ld", buffer, now_ms.count());
        return string(result);
    }
    
    // 计算 Modbus CRC16 校验
    void calculateCRC16(const vector<uint8_t>& data, uint8_t& crc_low, uint8_t& crc_high) {
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
        
        crc_low = crc & 0xFF;
        crc_high = (crc >> 8) & 0xFF;
    }
    
    // 十六进制字符串输出
    string hexDump(const vector<uint8_t>& data) {
        stringstream ss;
        for (uint8_t byte : data) {
            ss << hex << uppercase << setw(2) << setfill('0') << (int)byte << " ";
        }
        return ss.str();
    }
    
public:
    WindSpeedSensor(const string& port = "/dev/ttyS0", 
                    int baudrate = 9600, 
                    int timeout_ms = 1000,
                    uint8_t addr = 0x01)  // 默认地址0x01
        : port(port), baudrate(baudrate), timeout_ms(timeout_ms), 
          serial_fd(-1), device_addr(addr) {}
    
    ~WindSpeedSensor() {
        disconnect();
    }
    
    // 打开串口连接
    bool connect() {
        serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        
        if (serial_fd < 0) {
            cerr << "✗ 串口打开失败: " << port << endl;
            cerr << "  提示: 请检查串口是否存在或是否有权限访问" << endl;
            cerr << "  尝试: sudo chmod 666 " << port << endl;
            return false;
        }
        
        // 配置串口
        struct termios options;
        tcgetattr(serial_fd, &options);
        
        // 设置波特率
        speed_t baud;
        switch (baudrate) {
            case 9600:   baud = B9600;   break;
            case 19200:  baud = B19200;  break;
            case 38400:  baud = B38400;  break;
            case 57600:  baud = B57600;  break;
            case 115200: baud = B115200; break;
            default:     baud = B9600;   break;
        }
        
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);
        
        // 8N1 模式 (8数据位, 无校验, 1停止位)
        options.c_cflag &= ~PARENB;  // 无校验
        options.c_cflag &= ~CSTOPB;  // 1停止位
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;      // 8数据位
        
        // 原始模式
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        
        // 超时设置
        options.c_cc[VMIN] = 8;      // 最少读取8字节
        options.c_cc[VTIME] = 10;    // 1秒超时 (10 * 0.1s)
        
        tcsetattr(serial_fd, TCSANOW, &options);
        tcflush(serial_fd, TCIOFLUSH);
        
        cout << "✓ 串口 " << port << " 已打开 (波特率: " << baudrate 
             << ", 设备地址: 0x" << hex << uppercase << (int)device_addr << dec << ")" << endl;
        usleep(100000);  // 等待100ms
        
        return true;
    }
    
    // 关闭串口连接
    void disconnect() {
        if (serial_fd >= 0) {
            close(serial_fd);
            serial_fd = -1;
            cout << "✓ 串口已关闭" << endl;
        }
    }
    
    // 根据风速计算蒲福风级
    WindLevel getWindLevel(double wind_speed) {
        // 蒲福风级标准
        struct WindScale {
            double min_speed;
            double max_speed;
            int level;
            string name;
            string desc;
        };
        
        static const WindScale scales[] = {
            {0.0,  0.2,  0,  "无风",     "静"},
            {0.3,  1.5,  1,  "软风",     "烟能表示风向"},
            {1.6,  3.3,  2,  "轻风",     "人面感觉有风"},
            {3.4,  5.4,  3,  "微风",     "旌旗展开"},
            {5.5,  7.9,  4,  "和风",     "吹起尘土"},
            {8.0,  10.7, 5,  "清劲风",   "小树摇摆"},
            {10.8, 13.8, 6,  "强风",     "电线有声"},
            {13.9, 17.1, 7,  "疾风",     "步行困难"},
            {17.2, 20.7, 8,  "大风",     "折毁树枝"},
            {20.8, 24.4, 9,  "烈风",     "小损房屋"},
            {24.5, 28.4, 10, "狂风",     "拔起树木"},
            {28.5, 32.6, 11, "暴风",     "损毁普遍"},
            {32.7, 36.9, 12, "飓风",     "摧毁巨大"},
            {37.0, 41.4, 13, "台风",     "严重破坏"},
            {41.5, 46.1, 14, "强台风",   "极度破坏"},
            {46.2, 50.9, 15, "强台风",   "毁灭性"},
            {51.0, 56.0, 16, "超强台风", "极其危险"},
            {56.1, 61.2, 17, "超强台风", "灾难性"},
        };
        
        for (const auto& scale : scales) {
            if (wind_speed >= scale.min_speed && wind_speed <= scale.max_speed) {
                return {scale.level, scale.name, scale.desc};
            }
        }
        
        // 超过17级
        if (wind_speed > 61.2) {
            return {17, "超强台风", "灾难性"};
        }
        
        return {0, "无风", "静"};
    }
    
    // 读取风速值
    bool readWindSpeed(double& wind_speed, bool verbose = true) {
        // Modbus RTU 读取命令: [地址] 03 00 63 00 01 [CRC低] [CRC高]
        vector<uint8_t> command = {device_addr, 0x03, 0x00, 0x63, 0x00, 0x01};
        
        uint8_t crc_low, crc_high;
        calculateCRC16(command, crc_low, crc_high);
        command.push_back(crc_low);
        command.push_back(crc_high);
        
        // 清空接收缓冲区
        tcflush(serial_fd, TCIFLUSH);
        
        // 发送命令
        ssize_t bytes_written = write(serial_fd, command.data(), command.size());
        
        if (bytes_written != (ssize_t)command.size()) {
            if (verbose) {
                cerr << "✗ 发送命令失败" << endl;
            }
            return false;
        }
        
        if (verbose) {
            cout << "\n[" << getTimestamp() << "] SEND: ";
            cout << hexDump(command);
            cout << dec << endl;
        }
        
        // 等待响应
        usleep(50000);  // 50ms
        
        // 读取响应 (预期7字节)
        uint8_t response[32];
        ssize_t bytes_read = read(serial_fd, response, sizeof(response));
        
        if (bytes_read >= 7) {
            if (verbose) {
                cout << "[" << getTimestamp() << "] RECV: ";
                for (int i = 0; i < bytes_read; i++) {
                    cout << hex << uppercase << setw(2) << setfill('0') 
                         << (int)response[i] << " ";
                }
                cout << dec << endl;
            }
            
            // 验证响应格式 - 检查地址、功能码和数据长度
            if (response[0] == device_addr && response[1] == 0x03 && response[2] == 0x02) {
                // 提取风速数据 (大端序)
                uint16_t wind_speed_raw = (response[3] << 8) | response[4];
                wind_speed = wind_speed_raw / 1000.0;
                
                // 获取风力等级
                WindLevel level = getWindLevel(wind_speed);
                
                if (verbose) {
                    cout << "\n>>> 风速值 = 0x" << hex << uppercase << setw(4) 
                         << setfill('0') << wind_speed_raw << dec 
                         << " / 1000 = " << wind_speed_raw 
                         << " / 1000 = " << fixed << setprecision(3) 
                         << wind_speed << " m/s" << endl;
                    cout << "    速度换算: " << fixed << setprecision(2) 
                         << (wind_speed * 3.6) << " km/h" << endl;
                    cout << "    风力等级: " << level.level << "级 - " 
                         << level.name << " (" << level.description << ")" << endl;
                }
                
                return true;
            } else {
                if (verbose) {
                    cerr << "✗ 响应格式错误" << endl;
                    cerr << "  期望: " << hex << uppercase 
                         << (int)device_addr << " 03 02 xx xx crc_low crc_high" << dec << endl;
                    if (bytes_read > 0) {
                        cerr << "  实际: ";
                        for (int i = 0; i < bytes_read; i++) {
                            cerr << hex << uppercase << setw(2) << setfill('0') 
                                 << (int)response[i] << " ";
                        }
                        cerr << dec << endl;
                    }
                }
                return false;
            }
        } else {
            if (verbose) {
                cerr << "✗ 响应超时或数据不足: 收到 " << bytes_read << " 字节" << endl;
            }
            return false;
        }
    }
    
    // 连续读取风速
    void continuousRead(double interval = 0.5, int count = -1) {
        int read_count = 0;
        int success_count = 0;
        vector<double> wind_speeds;
        vector<int> wind_levels;
        
        cout << "\n" << string(60, '=') << endl;
        cout << "开始连续读取风速数据 (间隔: " << fixed << setprecision(1) 
             << interval << "秒)" << endl;
        cout << "按 Ctrl+C 停止" << endl;
        cout << string(60, '=') << endl;
        
        while (true) {
            if (count > 0 && read_count >= count) {
                break;
            }
            
            double wind_speed;
            if (readWindSpeed(wind_speed, true)) {
                success_count++;
                wind_speeds.push_back(wind_speed);
                WindLevel level = getWindLevel(wind_speed);
                wind_levels.push_back(level.level);
            }
            
            read_count++;
            if (count < 0 || read_count < count) {
                this_thread::sleep_for(chrono::milliseconds((int)(interval * 1000)));
            }
        }
        
        // 统计信息
        cout << "\n" << string(60, '=') << endl;
        cout << "统计信息:" << endl;
        cout << string(60, '=') << endl;
        cout << "总读取次数: " << read_count << endl;
        cout << "成功次数: " << success_count << endl;
        
        if (read_count > 0) {
            cout << "成功率: " << fixed << setprecision(1) 
                 << (success_count * 100.0 / read_count) << "%" << endl;
        }
        
        if (!wind_speeds.empty()) {
            double avg_speed = 0;
            double max_speed = wind_speeds[0];
            double min_speed = wind_speeds[0];
            
            for (double speed : wind_speeds) {
                avg_speed += speed;
                if (speed > max_speed) max_speed = speed;
                if (speed < min_speed) min_speed = speed;
            }
            avg_speed /= wind_speeds.size();
            
            WindLevel avg_level = getWindLevel(avg_speed);
            WindLevel max_level = getWindLevel(max_speed);
            WindLevel min_level = getWindLevel(min_speed);
            
            cout << "\n风速统计:" << endl;
            cout << "  平均风速: " << fixed << setprecision(2) << avg_speed 
                 << " m/s (" << (avg_speed * 3.6) << " km/h) - " 
                 << avg_level.level << "级 " << avg_level.name << endl;
            cout << "  最大风速: " << fixed << setprecision(2) << max_speed 
                 << " m/s (" << (max_speed * 3.6) << " km/h) - " 
                 << max_level.level << "级 " << max_level.name << endl;
            cout << "  最小风速: " << fixed << setprecision(2) << min_speed 
                 << " m/s (" << (min_speed * 3.6) << " km/h) - " 
                 << min_level.level << "级 " << min_level.name << endl;
            cout << "  风速波动: " << fixed << setprecision(2) 
                 << (max_speed - min_speed) << " m/s" << endl;
            
            // 风力等级分布统计
            cout << "\n风力等级分布:" << endl;
            map<int, int> level_count;
            for (int level : wind_levels) {
                level_count[level]++;
            }
            
            for (const auto& pair : level_count) {
                WindLevel level_info = getWindLevel(pair.first + 0.5);
                double percentage = (pair.second * 100.0) / wind_levels.size();
                cout << "  " << pair.first << "级 (" << level_info.name << "): " 
                     << pair.second << "次 (" << fixed << setprecision(1) 
                     << percentage << "%)" << endl;
            }
        }
        
        cout << string(60, '=') << "\n" << endl;
    }
};

int main() {
    // 配置参数
    const string PORT = "/dev/ttyS0";
    const int BAUDRATE = 9600;
    const uint8_t DEVICE_ADDR = 0x31;  // 设备地址0x31
    
    cout << string(60, '=') << endl;
    cout << "  风速传感器数据读取程序 (Modbus RTU)" << endl;
    cout << "  支持风速显示和蒲福风级等级判定" << endl;
    cout << string(60, '=') << endl;
    cout << "设备地址: 0x" << hex << uppercase << (int)DEVICE_ADDR << dec << endl;
    
    // 创建传感器对象
    WindSpeedSensor sensor(PORT, BAUDRATE, 1000, DEVICE_ADDR);
    
    // 连接传感器
    if (!sensor.connect()) {
        cerr << "\n提示: 如果使用 /dev/ttyS0 需要权限,请尝试:" << endl;
        cerr << "  sudo chmod 666 /dev/ttyS0" << endl;
        cerr << "  或者" << endl;
        cerr << "  sudo usermod -a -G dialout $USER" << endl;
        cerr << "  (然后注销重新登录)" << endl;
        return 1;
    }
    
    try {
        // 连续读取风速
        sensor.continuousRead(1);
    } catch (...) {
        cout << "\n✓ 用户中断读取" << endl;
    }
    
    return 0;
}