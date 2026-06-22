#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cstdint>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <signal.h>
#include <ctime>

//CRC校验码
uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

//串口模块
class SerialPort {
private:
    int fd;
    std::string port_name;
    
public:
    SerialPort(const std::string& port) : fd(-1), port_name(port) {}
    
    ~SerialPort() { if (isOpen()) close(); }
    
    bool open() {
        fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY);
        if (fd == -1) return false;
        return configure();
    }
    
    bool configure() {
        struct termios options;
        if (tcgetattr(fd, &options) != 0) return false;
        
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~PARENB;
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CRTSCTS;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10;
        
        tcflush(fd, TCIOFLUSH);
        return tcsetattr(fd, TCSANOW, &options) == 0;
    }

 //发送和接收模块   
    ssize_t send(const uint8_t* data, size_t len) {
        if (fd == -1) return -1;
        ssize_t written = write(fd, data, len);
        tcdrain(fd);
        return written;
    }
    
    ssize_t send_and_receive(const uint8_t* send_data, size_t send_len, uint8_t* recv_buffer, size_t recv_max_len,int timeout_ms = 500) 
    {
        if (fd == -1) return -1;
        
        tcflush(fd, TCIOFLUSH);
        
        ssize_t sent = write(fd, send_data, send_len);
        if (sent != (ssize_t)send_len) return -1;
        
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        
        int ret = select(fd + 1, &readfds, NULL, NULL, &tv);
        if (ret > 0) {
            return read(fd, recv_buffer, recv_max_len);
        }
        return 0;
    }
    
    int getFd() const { return fd; }
    
    void close() { if (fd != -1) ::close(fd); fd = -1; }
    bool isOpen() const { return fd != -1; }
    void flush() { if (fd != -1) tcflush(fd, TCIOFLUSH); }
};

//Modbus指令
std::vector<uint8_t> control_y1(uint8_t addr, bool close_relay) {
    uint8_t command[] = {
        addr, 0x06, 0x00, 0x00, 0x00, 
        close_relay ? 0x01 : 0x00
    };
    
    size_t len = sizeof(command);
    uint16_t crc = crc16(command, len);
    
    std::vector<uint8_t> full;
    full.insert(full.end(), command, command + len);
    full.push_back(crc & 0xFF);
    full.push_back((crc >> 8) & 0xFF);
    
    return full;
}

std::vector<uint8_t> read_x1(uint8_t addr) {
    uint8_t command[] = {
        addr, 0x04, 0x00, 0x00, 0x00, 0x01
    };
    
    size_t len = sizeof(command);
    uint16_t crc = crc16(command, len);
    
    std::vector<uint8_t> full;
    full.insert(full.end(), command, command + len);
    full.push_back(crc & 0xFF);
    full.push_back((crc >> 8) & 0xFF);
    
    return full;
}

int response_x1(const uint8_t* response, size_t len) {
    if (len < 7) return -1;
    if (response[1] != 0x04) return -1;
    if (response[2] != 0x02) return -1;
    
    uint16_t value = (response[3] << 8) | response[4];
    return (value & 0x0001) ? 1 : 0;
}

// 全局变量与信号处理
std::atomic<bool> g_running(true);
SerialPort* g_serial = nullptr;
std::vector<uint8_t> g_open_cmd;   
std::vector<uint8_t> g_close_cmd;  
uint8_t g_modbus_addr = 0x42;

void signal_handler(int sig) {
    (void)sig;
    g_running = false;
}

std::string get_time_str() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    char time_str[26];
    ctime_r(&time_t_now, time_str);
    time_str[24] = '\0';
    return std::string(time_str);
}

// 发送断开Y1指令
bool send_disconnect_y1() {
    if (!g_serial) return false;
    uint8_t resp[32];
    ssize_t resp_len = g_serial->send_and_receive(g_open_cmd.data(), g_open_cmd.size(),resp, sizeof(resp), 500);
    return (resp_len > 0);
}

// 发送吸合Y1指令
bool send_connect_y1() {
    if (!g_serial) return false;
    uint8_t resp[32];
    ssize_t resp_len = g_serial->send_and_receive(g_close_cmd.data(), g_close_cmd.size(), resp, sizeof(resp), 500);
    return (resp_len > 0);
}

// ========== 非阻塞键盘输入检测 ==========
bool kbhit() {
    struct timeval tv = {0, 0};
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    return select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0;
}

// ========== 等待用户确认市电是否已恢复 ==========
bool confirm_power_restored(int timeout_seconds = 30) {
    std::cout << std::endl;
    std::cout << "╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  ❓ 请确认：市电是否已经稳定恢复？                      ║" << std::endl;
    std::cout << "║                                                          ║" << std::endl;
    std::cout << "║  输入 Y (确认市电已恢复，断开Y1切回市电)                ║" << std::endl;
    std::cout << "║  输入 N (市电尚未恢复/不想切换，保持UPS供电)            ║" << std::endl;
    std::cout << "║                                                          ║" << std::endl;
    std::cout << "║  ⏰ 超时 " << timeout_seconds << " 秒后自动选择  N (保持UPS供电)  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << "请确认 (Y/N): " << std::flush;
    
    // 设置终端为原始模式（无需回车即可读取单个字符）
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    bool result = false;
    bool timeout = false;
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        // 检查是否超时
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= timeout_seconds) {
            timeout = true;
            break;
        }
        
        // 检查是否有按键
        if (kbhit()) {
            char c = getchar();
            if (c == 'Y' || c == 'y') {
                result = true;
                break;
            } else if (c == 'N' || c == 'n') {
                result = false;
                break;
            }
        }
        
        // 显示倒计时
        int remaining = timeout_seconds - elapsed;
        std::cout << "\r请确认 (Y/N): 等待输入... " << remaining << "秒后自动选择 N" << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    
    if (timeout) {
        std::cout << "\n\n⏰ 确认超时！默认选择 N (保持UPS供电)" << std::endl;
        return false;
    }
    
    if (result) {
        std::cout << "\n✅ 已确认：市电已稳定恢复，准备切回市电供电" << std::endl;
    } else {
        std::cout << "\nℹ️ 选择 N：保持UPS供电，市电暂不切换" << std::endl;
    }
    
    return result;
}

// ———————————主程序————————————— 
int main() {
    std::cout << "================================" << std::endl;
    std::cout << "         UPS供电系统" << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << std::endl;
    
    // 配置参数
    const std::string SERIAL_PORT = "/dev/ttyS0";
    const uint8_t MODBUS_ADDR = 0x42;
    const int MONITOR_INTERVAL_MS = 500;      // 监控间隔（毫秒）
    const int CONFIRM_TIMEOUT_SECONDS = 30;   // 人工确认超时时间（秒）
    
    g_modbus_addr = MODBUS_ADDR;
    
    std::cout << "配置信息:" << std::endl;
    std::cout << "  串口设备: " << SERIAL_PORT << std::endl;
    printf("  模块地址: 0x%02X\n", MODBUS_ADDR);
    std::cout << "  波特率: 9600, 8N1" << std::endl;
    std::cout << "  确认超时: " << CONFIRM_TIMEOUT_SECONDS << "秒" << std::endl;
    std::cout << std::endl;
    
    // 初始化串口
    SerialPort serial(SERIAL_PORT);
    if (!serial.open()) {
        std::cerr << " 串口打开失败 " << SERIAL_PORT << std::endl;
        return 1;
    }
    g_serial = &serial;
    
    // 构建指令
    g_close_cmd = control_y1(MODBUS_ADDR, true);
    g_open_cmd = control_y1(MODBUS_ADDR, false);
    auto read_cmd = read_x1(MODBUS_ADDR);
    
    // 吸合Y1（程序启动时恢复市电控制）
    std::cout << ">>> 启动初始化：吸合Y1，恢复市电控制..." << std::endl;
    if (send_connect_y1()) {
        std::cout << "✅ Y1已吸合，UPS支路待命" << std::endl;
    } else {
        std::cout << "⚠️ 未收到响应，请检查485通信" << std::endl;
    }
    
    std::cout << std::endl;
    
    //读取X1状态
    std::cout << ">>> 检测初始市电状态：" << std::endl;
    
    uint8_t resp[32];
    ssize_t resp_len = serial.send_and_receive(read_cmd.data(), read_cmd.size(),resp, sizeof(resp), 500);
    
    int last_x1 = -1;
    bool y1_closed = true;   // 记录当前Y1状态
    
    if (resp_len > 0) {
        int x1 = response_x1(resp, resp_len);
        last_x1 = x1;
        if (x1 == 1) {
            std::cout << "   ✅ 市电正常" << std::endl;
        } else if (x1 == 0) {
            std::cout << "   ⚠️ 市电掉电，设备由UPS供电" << std::endl;
        }
    } else {
        std::cout << "   ❌ 初始读取失败" << std::endl;
    }

    std::cout << std::endl;
    std::cout << ">>> 开始监控供电状态（按 Ctrl+C 退出）" << std::endl;
    std::cout << std::endl;
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    int monitor_count = 0;
    int fail_count = 0;
    bool ups_supplying = false;   
    bool waiting_confirmation = false;  // 是否正在等待人工确认
    
    while (g_running) {
        // 读取X1状态
        resp_len = serial.send_and_receive(read_cmd.data(), read_cmd.size(), resp, sizeof(resp), 300);
        
        if (resp_len > 0) {
            int x1 = response_x1(resp, resp_len);
            fail_count = 0;
            
            // 状态变化检测
            if (x1 != last_x1) {
                std::string time_str = get_time_str();
                
                if (x1 == 0 && last_x1 == 1) {
                    // 市电掉电
                    std::cout << std::endl;
                    std::cout << "╔════════════════════════════════════════════════╗" << std::endl;
                    std::cout << "║  ⚠️  [" << time_str << "] ⚠️ ║" << std::endl;
                    std::cout << "║                                              ║" << std::endl;
                    std::cout << "║  检测到市电掉电！                             ║" << std::endl;
                    std::cout << "║  UPS已自动切换供电，设备不会断电             ║" << std::endl;
                    std::cout << "║  Y1保持吸合，等待市电恢复                    ║" << std::endl;
                    std::cout << "╚════════════════════════════════════════════════╝" << std::endl;
                    std::cout << std::endl;
                    ups_supplying = true;
                    waiting_confirmation = false;  // 重置确认状态
                    
                } else if (x1 == 1 && last_x1 == 0) {
                    // 市电恢复
                    std::cout << std::endl;
                    std::cout << "╔════════════════════════════════════════════════╗" << std::endl;
                    std::cout << "║  💡  [" << time_str << "] 💡 ║" << std::endl;
                    std::cout << "║                                              ║" << std::endl;
                    std::cout << "║  检测到市电恢复信号！                         ║" << std::endl;
                    std::cout << "║  当前仍由UPS供电，等待用户确认               ║" << std::endl;
                    std::cout << "╚════════════════════════════════════════════════╝" << std::endl;
                    
                    // 等待用户确认市电是否已恢复
                    bool power_restored = confirm_power_restored(CONFIRM_TIMEOUT_SECONDS);
                    
                    if (power_restored) {
                        // 用户确认市电已恢复：断开Y1，切回市电
                        std::cout << std::endl;
                        std::cout << ">>> 执行断电操作：断开Y1，切回市电供电..." << std::endl;
                        
                        if (send_disconnect_y1()) {
                            std::cout << "✅ Y1已断开，已切回市电供电" << std::endl;
                            y1_closed = false;
                            
                            // 程序退出前提示
                            std::cout << std::endl;
                            std::cout << "╔════════════════════════════════════════════════╗" << std::endl;
                            std::cout << "║  ✅ 已切回市电供电                           ║" << std::endl;
                            std::cout << "║  系统由市电供电，Y1已断开                   ║" << std::endl;
                            std::cout << "║  下次启动时将自动吸合Y1，恢复UPS待命        ║" << std::endl;
                            std::cout << "╚════════════════════════════════════════════════╝" << std::endl;
                            std::cout << std::endl;
                            
                            // 等待用户按回车退出
                            std::cout << "按 Enter 键退出程序..." << std::endl;
                            std::cin.get();
                            
                            serial.close();
                            return 0;  // 程序退出
                            
                        } else {
                            std::cout << "❌ 断开Y1失败，请检查485通信" << std::endl;
                        }
                    } else {
                        // 用户确认市电未恢复或选择保持UPS供电
                        std::cout << std::endl;
                        std::cout << "ℹ️ 保持UPS供电，Y1保持吸合" << std::endl;
                        std::cout << "   如需切回市电，请按 Ctrl+C 退出后重新启动程序" << std::endl;
                        std::cout << std::endl;
                    }
                    
                    ups_supplying = false;
                    waiting_confirmation = false;
                }
                
                last_x1 = x1;
            }
            
            // 实时状态显示
            monitor_count++;
            if (monitor_count >= (1000 / MONITOR_INTERVAL_MS)) {
                std::string time_str = get_time_str();
                
                // 确定供电来源
                std::string power_source;
                if (x1 == 1 && !ups_supplying) {
                    power_source = "市电供电";
                } else {
                    power_source = "UPS供电";
                }
                
                std::cout << "\r[" << time_str << "]"
                          << " | 供电: " << power_source
                          << " | Y1: " << (y1_closed ? "已吸合" : "已断开")
                          << "                     " << std::flush;
                monitor_count = 0;
            }
            
        } else {
            // 通信失败
            fail_count++;
            if (fail_count == 1) {
                std::cout << std::endl;
                std::cout << "⚠️ 485通信异常" << std::endl;
            } else if (fail_count % 10 == 0) {
                std::cout << "\r⚠️ 485通信异常 " << fail_count << " 次" << std::flush;
            }
        }
        
        // 监控间隔
        std::this_thread::sleep_for(std::chrono::milliseconds(MONITOR_INTERVAL_MS));
    }
    
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "        监控已停止" << std::endl;
    std::cout << "========================================" << std::endl;
    
    serial.close();
    return 0;
}