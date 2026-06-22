#include <iostream>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <chrono>
#include <thread>
#include <sstream>
#include <iomanip>
#include <cstdint>
#include <cstdlib>

using namespace std;

// ============================================================
//  LWK7575 鼓风机控制类
// ============================================================
class LWK7575 {
public:
    // 构造函数
    LWK7575(const string& port, uint8_t slave_id = 0x01)
        : port_(port), slave_id_(slave_id), fd_(-1) {}

    // 析构函数
    ~LWK7575() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    // ========== 串口初始化 ==========
    bool init(int baudrate = 9600) {
        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0) {
            cerr << "[错误] 打开串口失败: " << port_ << endl;
            return false;
        }

        struct termios options;
        if (tcgetattr(fd_, &options) != 0) {
            cerr << "[错误] 获取串口属性失败" << endl;
            return false;
        }

        // 设置波特率
        speed_t speed;
        switch (baudrate) {
            case 9600:   speed = B9600; break;
            case 19200:  speed = B19200; break;
            case 38400:  speed = B38400; break;
            case 115200: speed = B115200; break;
            default:     speed = B9600; break;
        }
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        // 8数据位，无校验，1停止位 (8N1)
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag |= CLOCAL | CREAD;

        // 原始模式
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;

        // 设置超时
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 100;

        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            cerr << "[错误] 设置串口属性失败" << endl;
            return false;
        }

        tcflush(fd_, TCIOFLUSH);
        cout << "[OK] 串口初始化成功: " << port_ << " @ " << baudrate << " bps" << endl;
        cout << "[OK] 从机地址: 0x" << hex << (int)slave_id_ << dec << endl;
        return true;
    }

    // ========== 配对从机 ==========
    bool pairSlave(uint8_t new_address) {
        cout << "\n[配对] 正在配对，新地址: 0x" << hex << (int)new_address << dec << endl;
        cout << "[配对] 请确保驱动板已长按按键进入配对模式（绿灯闪烁）" << endl;

        uint8_t old_slave = slave_id_;
        slave_id_ = 0xC8;

        bool result = writeRegister(0x0000, new_address);

        slave_id_ = old_slave;

        if (result) {
            cout << "[OK] 配对成功！从机地址: 0x" << hex << (int)new_address << dec << endl;
            slave_id_ = new_address;
        } else {
            cout << "[失败] 配对失败，请检查连接和配对模式" << endl;
        }

        return result;
    }

    // ========== 控制功能 ==========
    bool startFan() {
        cout << "[控制] 启动风机..." << endl;
        return writeRegister(0x0003, 0x0000);
    }

    bool stopFan() {
        cout << "[控制] 停止风机..." << endl;
        return writeRegister(0x0003, 0x0001);
    }

    bool setSpeed(uint16_t speed) {
        if (speed > 4000) {
            cerr << "[错误] 转速超出范围 (0-4000)" << endl;
            return false;
        }
        cout << "[控制] 设置转速: " << speed << "/4000";
        if (speed < 80) {
            cout << " (小于80，风机将停止)";
        }
        cout << endl;
        return writeRegister(0x0006, speed);
    }

    bool setMode(uint16_t mode) {
        if (mode > 1) {
            cerr << "[错误] 模式错误 (0=手动, 1=自动)" << endl;
            return false;
        }
        cout << "[控制] 设置模式: " << (mode == 0 ? "手动" : "自动") << endl;
        return writeRegister(0x0009, mode);
    }

    bool setDelayShutdown(uint16_t seconds) {
        cout << "[控制] 设置延时关机: " << seconds << " 秒" << endl;
        return writeRegister(0x000A, seconds);
    }

    // ========== 读取状态 ==========
    uint16_t readRunningStatus() {
        uint16_t status = readRegister(0x0001);
        if (status != 0xFFFF) {
            cout << "[状态] 运行状态: ";
            switch(status) {
                case 0:  cout << "停止"; break;
                case 1:  cout << "启动中"; break;
                case 2:  cout << "正常运行"; break;
                case 6:  cout << "上电初始化"; break;
                case 7:  cout << "错误状态"; break;
                default: cout << "未知(" << status << ")"; break;
            }
            cout << endl;
        }
        return status;
    }

    uint16_t readErrorStatus() {
        uint16_t error = readRegister(0x0002);
        if (error != 0xFFFF) {
            if (error == 0) {
                cout << "[状态] 异常状态: 无" << endl;
            } else {
                cout << "[状态] 异常状态: 0x" << hex << error << dec << endl;
                if (error & 0x01) cout << "   └─ 过流" << endl;
                if (error & 0x02) cout << "   └─ 过压" << endl;
                if (error & 0x04) cout << "   └─ 欠压" << endl;
                if (error & 0x08) cout << "   └─ 过温" << endl;
                if (error & 0x10) cout << "   └─ 堵转" << endl;
                if (error & 0x20) cout << "   └─ 短路" << endl;
                if (error & 0x40) cout << "   └─ 霍尔错误" << endl;
                if (error & 0x80) cout << "   └─ 缺相" << endl;
            }
        }
        return error;
    }

    uint32_t readActualSpeed() {
        uint32_t speed = readTwoRegisters(0x0004);
        if (speed != 0xFFFFFFFF) {
            cout << "[状态] 实际转速: " << speed << " RPM" << endl;
        }
        return speed;
    }

    uint16_t readSetSpeed() {
        uint16_t speed = readRegister(0x0006);
        if (speed != 0xFFFF) {
            cout << "[状态] 设定转速: " << speed << "/4000" << endl;
        }
        return speed;
    }

    uint16_t readMode() {
        uint16_t mode = readRegister(0x0009);
        if (mode != 0xFFFF) {
            cout << "[状态] 当前模式: " << (mode == 0 ? "手动" : "自动") << endl;
        }
        return mode;
    }

    // ========== 打印完整状态 ==========
    void printStatus() {
        cout << "\n╔══════════════════════════════════════╗" << endl;
        cout << "║      LWK7575 鼓风机状态信息         ║" << endl;
        cout << "╠══════════════════════════════════════╣" << endl;
        readRunningStatus();
        readErrorStatus();
        readActualSpeed();
        readSetSpeed();
        readMode();
        cout << "╚══════════════════════════════════════╝\n" << endl;
    }

    // ========== 交互式控制台 ==========
    void interactiveMode() {
        cout << "\n╔══════════════════════════════════════╗" << endl;
        cout << "║      LWK7575 鼓风机交互控制台       ║" << endl;
        cout << "╠══════════════════════════════════════╣" << endl;
        cout << "║  [1] 启动风机                       ║" << endl;
        cout << "║  [2] 停止风机                       ║" << endl;
        cout << "║  [3] 设置转速 (0-4000)              ║" << endl;
        cout << "║  [4] 设置模式 (0=手动, 1=自动)      ║" << endl;
        cout << "║  [5] 查看状态                       ║" << endl;
        cout << "║  [6] 配对从机                       ║" << endl;
        cout << "║  [7] 设置延时关机 (秒)              ║" << endl;
        cout << "║  [0] 退出                           ║" << endl;
        cout << "╚══════════════════════════════════════╝" << endl;

        int choice;
        while (true) {
            cout << "\n请输入命令: ";
            cin >> choice;

            switch (choice) {
                case 0:
                    cout << "[退出] 程序结束" << endl;
                    return;

                case 1:
                    startFan();
                    break;

                case 2:
                    stopFan();
                    break;

                case 3: {
                    uint16_t speed;
                    cout << "请输入转速 (0-4000): ";
                    cin >> speed;
                    setSpeed(speed);
                    break;
                }

                case 4: {
                    uint16_t mode;
                    cout << "请输入模式 (0=手动, 1=自动): ";
                    cin >> mode;
                    setMode(mode);
                    break;
                }

                case 5:
                    printStatus();
                    break;

                case 6: {
                    uint8_t addr;
                    cout << "请输入新从机地址 (1-255): ";
                    cin >> addr;
                    pairSlave(addr);
                    break;
                }

                case 7: {
                    uint16_t seconds;
                    cout << "请输入延时关机时间 (秒): ";
                    cin >> seconds;
                    setDelayShutdown(seconds);
                    break;
                }

                default:
                    cout << "[警告] 无效命令，请重新输入" << endl;
                    break;
            }

            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }

private:
    string port_;
    uint8_t slave_id_;
    int fd_;

    // ========== CRC16 计算 ==========
    uint16_t crc16(const vector<uint8_t>& data) {
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

    // ========== 串口读写 ==========
    bool writeData(const vector<uint8_t>& data) {
        if (fd_ < 0) return false;

        int sent = write(fd_, data.data(), data.size());
        if (sent != (int)data.size()) {
            cerr << "[错误] 发送数据失败" << endl;
            return false;
        }

        cout << "[发送] ";
        for (uint8_t b : data) {
            cout << hex << setw(2) << setfill('0') << (int)b << " ";
        }
        cout << dec << endl;
        return true;
    }

    vector<uint8_t> readData(int timeout_ms = 1000) {
        vector<uint8_t> response;
        if (fd_ < 0) return response;

        fd_set read_fds;
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
        if (ret < 0 || ret == 0) {
            if (ret == 0) {
                cout << "[警告] 接收超时" << endl;
            }
            return response;
        }

        uint8_t buffer[256];
        int n = read(fd_, buffer, sizeof(buffer));
        if (n > 0) {
            response.insert(response.end(), buffer, buffer + n);
            cout << "[接收] ";
            for (int i = 0; i < n; i++) {
                cout << hex << setw(2) << setfill('0') << (int)buffer[i] << " ";
            }
            cout << dec << endl;
        }

        return response;
    }

    // ========== 写单个寄存器 ==========
    bool writeRegister(uint16_t reg_addr, uint16_t value) {
        vector<uint8_t> frame;
        frame.push_back(slave_id_);
        frame.push_back(0x06);
        frame.push_back((reg_addr >> 8) & 0xFF);
        frame.push_back(reg_addr & 0xFF);
        frame.push_back((value >> 8) & 0xFF);
        frame.push_back(value & 0xFF);

        uint16_t crc = crc16(frame);
        frame.push_back(crc & 0xFF);
        frame.push_back((crc >> 8) & 0xFF);

        if (!writeData(frame)) return false;

        auto response = readData(1000);
        if (response.size() < 8) return false;

        // 检查响应
        if (response.size() == 8 && response[1] == 0x06) {
            return true;
        }

        // 检查是否为错误响应
        if (response.size() >= 3 && (response[1] & 0x80)) {
            cerr << "[错误] 从机返回错误码: 0x" << hex << (int)response[2] << dec << endl;
        }

        return false;
    }

    // ========== 读单个寄存器 ==========
    uint16_t readRegister(uint16_t reg_addr) {
        vector<uint8_t> frame;
        frame.push_back(slave_id_);
        frame.push_back(0x03);
        frame.push_back((reg_addr >> 8) & 0xFF);
        frame.push_back(reg_addr & 0xFF);
        frame.push_back(0x00);
        frame.push_back(0x01);

        uint16_t crc = crc16(frame);
        frame.push_back(crc & 0xFF);
        frame.push_back((crc >> 8) & 0xFF);

        if (!writeData(frame)) return 0xFFFF;

        auto response = readData(1000);
        if (response.size() < 5) return 0xFFFF;

        if (response.size() >= 5 && response[1] == 0x03) {
            uint16_t value = (response[3] << 8) | response[4];
            return value;
        }

        // 检查是否为错误响应
        if (response.size() >= 3 && (response[1] & 0x80)) {
            cerr << "[错误] 从机返回错误码: 0x" << hex << (int)response[2] << dec << endl;
        }

        return 0xFFFF;
    }

    // ========== 读两个连续的寄存器 (32位) ==========
    uint32_t readTwoRegisters(uint16_t reg_addr) {
        vector<uint8_t> frame;
        frame.push_back(slave_id_);
        frame.push_back(0x03);
        frame.push_back((reg_addr >> 8) & 0xFF);
        frame.push_back(reg_addr & 0xFF);
        frame.push_back(0x00);
        frame.push_back(0x02);

        uint16_t crc = crc16(frame);
        frame.push_back(crc & 0xFF);
        frame.push_back((crc >> 8) & 0xFF);

        if (!writeData(frame)) return 0xFFFFFFFF;

        auto response = readData(1000);
        if (response.size() < 7) return 0xFFFFFFFF;

        if (response.size() >= 7 && response[1] == 0x03) {
            uint32_t value = 0;
            value |= (uint32_t)response[3] << 24;
            value |= (uint32_t)response[4] << 16;
            value |= (uint32_t)response[5] << 8;
            value |= response[6];
            return value;
        }

        return 0xFFFFFFFF;
    }
};

// ============================================================
//  显示帮助信息
// ============================================================
void showHelp(const char* program_name) {
    cout << "\n使用方法:" << endl;
    cout << "  " << program_name << " [串口设备] [从机地址]" << endl;
    cout << "\n参数说明:" << endl;
    cout << "  串口设备  : 串口设备路径，默认 /dev/ttyS0" << endl;
    cout << "  从机地址  : Modbus从机地址(1-255)，默认 1" << endl;
    cout << "\n示例:" << endl;
    cout << "  " << program_name << "                          # 使用默认参数" << endl;
    cout << "  " << program_name << " /dev/ttyS0 1           # 指定串口和地址" << endl;
    cout << endl;
}

// ============================================================
//  主程序入口
// ============================================================
int main(int argc, char* argv[]) {
    cout << "\n╔══════════════════════════════════════╗" << endl;
    cout << "║      LWK7575 鼓风机控制程序         ║" << endl;
    cout << "║      test_7575 v1.0                  ║" << endl;
    cout << "╚══════════════════════════════════════╝\n" << endl;

    // 参数解析 - 默认使用 /dev/ttyS0
    string port = "/dev/ttyS0";
    uint8_t slave_id = 0x01;

    if (argc >= 2) {
        string arg1 = argv[1];
        if (arg1 == "-h" || arg1 == "--help") {
            showHelp(argv[0]);
            return 0;
        }
        port = arg1;
    }
    if (argc >= 3) {
        slave_id = (uint8_t)atoi(argv[2]);
        if (slave_id == 0) slave_id = 1;
    }

    cout << "串口设备    : " << port << endl;
    cout << "从机地址    : 0x" << hex << (int)slave_id << dec << endl;

    // 创建控制对象
    LWK7575 fan(port, slave_id);

    // 初始化串口
    if (!fan.init(9600)) {
        cerr << "[错误] 串口初始化失败，程序退出" << endl;
        return -1;
    }

    // 等待系统稳定
    this_thread::sleep_for(chrono::milliseconds(500));

    // 读取初始状态
    fan.printStatus();

    // 进入交互模式
    fan.interactiveMode();

    return 0;
}