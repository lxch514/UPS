// g++ -o test_osa test_osa.cpp -std=c++11
// ./test_osa

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

int main() {
    printf("================================================\n");
    printf("         OSA-15N 风速传感器单独测试\n");
    printf("================================================\n");
    printf("设备地址: 0x31 (49)\n");
    printf("波特率: 9600, 串口: /dev/ttyS0\n");
    printf("================================================\n\n");
    
    // 打开串口
    int fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        printf("错误: 无法打开串口 /dev/ttyS0 (errno=%d)\n", errno);
        return -1;
    }
    printf("串口打开成功, fd=%d\n", fd);
    
    // 配置串口
    struct termios options;
    if (tcgetattr(fd, &options) < 0) {
        printf("错误: 获取串口属性失败\n");
        close(fd);
        return -1;
    }
    
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;
    
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        printf("错误: 设置串口属性失败\n");
        close(fd);
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    
    // 构建请求: 31 03 00 00 00 01 CRC
    unsigned char request[8];
    request[0] = 0x31;
    request[1] = 0x03;
    request[2] = 0x00;
    request[3] = 0x00;
    request[4] = 0x00;
    request[5] = 0x01;
    
    unsigned short crc = calculateCRC(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    
    printf("请求数据: ");
    for (int i = 0; i < 8; i++) {
        printf("%02X ", request[i]);
    }
    printf("\n\n");
    
    printf("开始采集数据...\n");
    printf("采样#  风速(m/s)  原始数据(HEX)\n");
    printf("----------------------------------------\n");
    
    int sample_count = 0;
    int max_samples = 10;
    
    while (sample_count < max_samples) {
        // 清空缓冲区
        tcflush(fd, TCIOFLUSH);
        usleep(50000);
        
        // 发送请求
        ssize_t written = write(fd, request, 8);
        if (written != 8) {
            printf("发送请求失败 (写入=%ld)\n", written);
            usleep(2000000);
            continue;
        }
        
        // 等待响应
        usleep(100000);
        
        // 读取响应
        unsigned char response[32];
        memset(response, 0, sizeof(response));
        
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        
        int ret = select(fd + 1, &readfds, NULL, NULL, &tv);
        if (ret > 0) {
            ssize_t bytes_read = read(fd, response, sizeof(response) - 1);
            
            if (bytes_read > 0) {
                // 检查响应
                if (bytes_read >= 5 && response[0] == 0x31 && response[1] == 0x03) {
                    if (response[2] == 2 && bytes_read >= 5) {
                        // 解析风速
                        unsigned short speed_raw = (response[3] << 8) | response[4];
                        float speed = speed_raw * 0.1f;
                        
                        printf("%-6d  %-10.2f  ", sample_count + 1, speed);
                        for (int i = 0; i < bytes_read; i++) {
                            printf("%02X ", response[i]);
                        }
                        printf("\n");
                        sample_count++;
                    } else {
                        printf("%-6d  数据错误        ", sample_count + 1);
                        for (int i = 0; i < bytes_read; i++) {
                            printf("%02X ", response[i]);
                        }
                        printf("\n");
                    }
                } else {
                    printf("%-6d  响应格式错误    ", sample_count + 1);
                    for (int i = 0; i < bytes_read; i++) {
                        printf("%02X ", response[i]);
                    }
                    printf("\n");
                }
            }
        } else {
            printf("%-6d  超时\n", sample_count + 1);
        }
        
        usleep(2000000);
    }
    
    close(fd);
    printf("\n测试完成\n");
    return 0;
}