// g++ -o test_yn8588 test_yn8588.cpp -std=c++11
// ./test_yn8588

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <errno.h>
#include <sys/select.h>

// 计算CRC16
unsigned short calculateCRC(const unsigned char* data, int len) {
    unsigned short crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 打印数据帧
void printFrame(const unsigned char* data, int len, const char* prefix) {
    printf("%s: ", prefix);
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

// 读取风速（寄存器地址0x0000）
bool readWindSpeed(int fd, unsigned char addr, float& speed, unsigned char* response, int& resp_len) {
    // 构建Modbus请求: 地址 03 00 00 00 01 CRC
    unsigned char request[8];
    request[0] = addr;      // 设备地址
    request[1] = 0x03;      // 功能码：读保持寄存器
    request[2] = 0x00;      // 起始地址高字节
    request[3] = 0x00;      // 起始地址低字节
    request[4] = 0x00;      // 寄存器数量高字节
    request[5] = 0x01;      // 寄存器数量低字节（读取1个寄存器）
    
    unsigned short crc = calculateCRC(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    
    // 打印发送的请求帧
    printFrame(request, 8, "发送");
    
    // 清空缓冲区
    tcflush(fd, TCIOFLUSH);
    usleep(50000);
    
    // 发送请求
    ssize_t written = write(fd, request, 8);
    if (written != 8) {
        printf("发送请求失败 (写入=%ld)\n", written);
        return false;
    }
    
    // 等待响应
    usleep(100000);
    
    // 读取响应
    memset(response, 0, 32);
    
    fd_set readfds;
    struct timeval tv;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    
    int ret = select(fd + 1, &readfds, NULL, NULL, &tv);
    if (ret > 0) {
        ssize_t bytes_read = read(fd, response, 31);
        resp_len = bytes_read;
        
        if (bytes_read > 0) {
            // 打印接收到的响应帧
            printFrame(response, bytes_read, "接收");
            
            if (bytes_read >= 5 && response[0] == addr && response[1] == 0x03) {
                // 检查数据长度
                if (response[2] == 2 && bytes_read >= 5) {
                    // 解析风速数据（两个字节，大端模式）
                    unsigned short speed_raw = (response[3] << 8) | response[4];
                    
                    // 验证CRC
                    if (bytes_read >= 5) {
                        unsigned short recv_crc = (response[bytes_read-1] << 8) | response[bytes_read-2];
                        unsigned short calc_crc = calculateCRC(response, bytes_read - 2);
                        
                        if (recv_crc == calc_crc) {
                            // YN8588风速计算公式：实际值 = 寄存器值 × 0.1
                            speed = speed_raw * 0.1f;
                            return true;
                        } else {
                            printf("CRC校验失败: 收到0x%04X, 计算0x%04X\n", recv_crc, calc_crc);
                        }
                    }
                } else {
                    printf("数据长度错误: 期望5字节, 收到%ld字节\n", bytes_read);
                }
            } else {
                printf("响应格式错误: 地址或功能码不匹配\n");
            }
        }
    } else if (ret == 0) {
        printf("接收超时 (2秒)\n");
    } else {
        printf("select错误\n");
    }
    
    return false;
}

int main() {
    printf("================================================\n");
    printf("         YN8588 风速传感器单独测试\n");
    printf("================================================\n");
    printf("设备地址: 0x01 (1)\n");
    printf("波特率: 9600, 串口: /dev/ttyS0\n");
    printf("协议: Modbus RTU\n");
    printf("================================================\n\n");
    
    // 打开串口
    int fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        printf("错误: 无法打开串口 /dev/ttyS0 (errno=%d)\n", errno);
        return -1;
    }
    printf("串口打开成功, fd=%d\n\n", fd);
    
    // 配置串口
    struct termios options;
    if (tcgetattr(fd, &options) < 0) {
        printf("错误: 获取串口属性失败\n");
        close(fd);
        return -1;
    }
    
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    
    options.c_cflag &= ~PARENB;     // 无校验
    options.c_cflag &= ~CSTOPB;     // 1位停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;         // 8位数据位
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;       // 100ms超时
    
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        printf("错误: 设置串口属性失败\n");
        close(fd);
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    
    printf("开始采集数据...\n");
    printf("采样#  风速(m/s)  状态\n");
    printf("----------------------------------------\n");
    
    int sample_count = 0;
    int max_samples = 10;
    unsigned char device_addr = 0x01;
    
    while (sample_count < max_samples) {
        float speed = 0.0f;
        unsigned char response[32];
        int resp_len = 0;
        
        printf("\n[采样 %d]\n", sample_count + 1);
        
        if (readWindSpeed(fd, device_addr, speed, response, resp_len)) {
            printf("结果: %-6d  %-10.2f  成功\n", sample_count + 1, speed);
            sample_count++;
        } else {
            printf("结果: %-6d  --         失败\n", sample_count + 1);
        }
        
        // 等待2秒再进行下一次采样
        if (sample_count < max_samples) {
            usleep(2000000);
        }
    }
    
    close(fd);
    printf("\n================================================\n");
    printf("测试完成\n");
    return 0;
}