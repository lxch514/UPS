#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cstdint>
#include <vector>
#include <sys/select.h>
#include <chrono>      // 添加这个头文件

// CRC16-Modbus 计算函数
uint16_t crc16_modbus(const uint8_t* data, size_t len) {
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

class SerialPort {
private:
    int fd;
    std::string port_name;
    
public:
    SerialPort(const std::string& port) : fd(-1), port_name(port) {}
    
    ~SerialPort() {
        if (isOpen()) close();
    }
    
    bool open() {
        fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
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
        options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
        
        // 设置原始输入模式，不等待
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;
        
        tcflush(fd, TCIOFLUSH);
        
        return tcsetattr(fd, TCSANOW, &options) == 0;
    }
    
    bool send(const uint8_t* data, size_t len) {
        if (fd == -1) return false;
        ssize_t written = write(fd, data, len);
        tcdrain(fd);
        return written == (ssize_t)len;
    }
    
    // 接收函数：带超时功能
    ssize_t receive(uint8_t* buffer, size_t max_len, int timeout_ms = 1000) {
        if (fd == -1) return -1;
        
        fd_set readfds;
        struct timeval tv;
        
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        
        int ret = select(fd + 1, &readfds, NULL, NULL, &tv);
        
        if (ret > 0) {
            return read(fd, buffer, max_len);
        } else if (ret == 0) {
            return 0;  // 超时
        } else {
            return -1;  // 错误
        }
    }
    
    // 清空接收缓冲区
    void flush_rx() {
        if (fd != -1) {
            tcflush(fd, TCIFLUSH);
        }
    }
    
    void close() {
        if (fd != -1) ::close(fd);
        fd = -1;
    }
    
    bool isOpen() const { return fd != -1; }
};

// 打印十六进制数据
void print_hex(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
}

// 验证CRC
bool verify_crc(const uint8_t* data, size_t len) {
    if (len < 2) return false;
    
    uint16_t received_crc = (data[len-2]) | (data[len-1] << 8);
    uint16_t calculated_crc = crc16_modbus(data, len - 2);
    
    return received_crc == calculated_crc;
}

int main() {
    SerialPort serial("/dev/ttyS0");
    
    if (!serial.open()) {
        std::cerr << "无法打开串口 /dev/ttyS0" << std::endl;
        return 1;
    }
    
    std::cout << "串口已打开，配置为 9600 8N1" << std::endl;
    std::cout << std::endl;

    // 填写你的指令（不需要手动加CRC） 
    uint8_t command[] = {
        0x42, 0x06,       
        0x00, 0x00,       
        0x00, 0x00   
    };
    
    size_t data_len = sizeof(command);
    
    // 计算CRC
    uint16_t crc = crc16_modbus(command, data_len);
    
    // 构建完整指令
    std::vector<uint8_t> full_command;
    for (size_t i = 0; i < data_len; i++) {
        full_command.push_back(command[i]);
    }
    full_command.push_back(crc & 0xFF);
    full_command.push_back((crc >> 8) & 0xFF);
    
    // 显示发送信息
    std::cout << "=== 发送数据 ===" << std::endl;
    std::cout << "原始数据（" << data_len << "字节）: ";
    print_hex(command, data_len);
    std::cout << std::endl;
    
    printf("CRC: %02X %02X\n", crc & 0xFF, (crc >> 8) & 0xFF);
    
    std::cout << "完整指令（" << full_command.size() << "字节）: ";
    print_hex(full_command.data(), full_command.size());
    std::cout << std::endl;
    
    // 清空接收缓冲区
    serial.flush_rx();
    
    // 发送数据
    if (!serial.send(full_command.data(), full_command.size())) {
        std::cerr << "发送失败" << std::endl;
        return 1;
    }
    
    std::cout << "数据已发送" << std::endl;
    std::cout << std::endl;
    
    // ============================================
    // 接收响应数据
    // ============================================
    std::cout << "=== 等待响应 ===" << std::endl;
    
    uint8_t response[256];
    ssize_t received = serial.receive(response, sizeof(response), 2000);
    
    if (received > 0) {
        std::cout << "接收到 " << received << " 字节: ";
        print_hex(response, received);
        std::cout << std::endl;
        
        if (received >= 2) {
            if (verify_crc(response, received)) {
                std::cout << "CRC校验正确" << std::endl;
            } else {
                std::cout << "CRC校验失败" << std::endl;
            }
        }
        
        // 解析响应
        if (received >= 8 && response[1] == 0x10) {
            std::cout << std::endl;
            std::cout << "=== 响应解析 ===" << std::endl;
            printf("设备地址: 0x%02X\n", response[0]);
            printf("功能码: 0x%02X\n", response[1]);
            printf("起始地址: 0x%02X%02X\n", response[2], response[3]);
            printf("寄存器数量: 0x%02X%02X\n", response[4], response[5]);
            std::cout << "写入成功！" << std::endl;
        }
        
    } else if (received == 0) {
        std::cout << " 未收到响应（超时）" << std::endl;
        std::cout << "设备可能不返回响应数据" << std::endl;
    } else {
        std::cerr << " 接收错误" << std::endl;
    }
    
    serial.close();
    std::cout << std::endl << "程序执行完成" << std::endl;
    
    return 0;
}
