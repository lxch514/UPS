// 编译 g++ -o wind_test wind_test.cpp -std=c++11 -lm
// 运行sudo ./wind_test

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <errno.h>
#include <sys/select.h>
#include <math.h>

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

// 配置串口
int configSerial(int fd, int baudrate) {
    struct termios options;
    if (tcgetattr(fd, &options) < 0) {
        return -1;
    }
    
    speed_t speed;
    switch (baudrate) {
        case 4800: speed = B4800; break;
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        default: speed = B9600;
    }
    
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    
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
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    return 0;
}

// 打开串口
int openSerial(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        printf("错误: 无法打开串口 %s (errno=%d)\n", port, errno);
        return -1;
    }
    return fd;
}

// 读取OSA-15N数据 (地址0x31)
int readOSA15N(int fd, float* speed) {
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
    
    // 清空缓冲区
    tcflush(fd, TCIOFLUSH);
    usleep(50000);
    
    // 发送请求
    ssize_t written = write(fd, request, 8);
    if (written != 8) {
        return -1;
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
    tv.tv_sec = 1;
    tv.tv_usec = 500000;
    
    int ret = select(fd + 1, &readfds, NULL, NULL, &tv);
    if (ret > 0) {
        ssize_t bytes_read = read(fd, response, sizeof(response) - 1);
        if (bytes_read >= 5 && response[0] == 0x31 && response[1] == 0x03) {
            if (response[2] == 2) {
                unsigned short speed_raw = (response[3] << 8) | response[4];
                *speed = speed_raw * 0.1f;
                return 0;
            }
        }
    }
    return -1;
}

// 读取UMini-WS数据 (地址0x01)
int readUMiniWS(int fd, float* speed, float* direction) {
    unsigned char request[8];
    request[0] = 0x01;
    request[1] = 0x03;
    request[2] = 0x00;
    request[3] = 0x00;
    request[4] = 0x00;
    request[5] = 0x02;
    
    unsigned short crc = calculateCRC(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    
    // 清空缓冲区
    tcflush(fd, TCIOFLUSH);
    usleep(50000);
    
    // 发送请求
    ssize_t written = write(fd, request, 8);
    if (written != 8) {
        return -1;
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
    tv.tv_sec = 1;
    tv.tv_usec = 500000;
    
    int ret = select(fd + 1, &readfds, NULL, NULL, &tv);
    if (ret > 0) {
        ssize_t bytes_read = read(fd, response, sizeof(response) - 1);
        if (bytes_read >= 7 && response[0] == 0x01 && response[1] == 0x03) {
            if (response[2] == 4) {
                unsigned short speed_raw = (response[3] << 8) | response[4];
                unsigned short dir_raw = (response[5] << 8) | response[6];
                *speed = speed_raw * 0.01f;
                *direction = dir_raw * 1.0f;
                return 0;
            }
        }
    }
    return -1;
}

int main() {
    printf("============================================================\n");
    printf("         双风速传感器数据采集程序\n");
    printf("============================================================\n");
    printf("OSA-15N   地址: 0x31, 串口: /dev/ttyS0\n");
    printf("UMini-WS  地址: 0x01, 串口: /dev/ttyS3\n");
    printf("============================================================\n\n");
    
    // 打开两个串口
    int fd_osa = openSerial("/dev/ttyS0");
    if (fd_osa < 0) return -1;
    
    int fd_umini = openSerial("/dev/ttyS3");
    if (fd_umini < 0) {
        close(fd_osa);
        return -1;
    }
    
    // 配置串口
    if (configSerial(fd_osa, 9600) < 0) {
        printf("配置 OSA-15N 串口失败\n");
        close(fd_osa);
        close(fd_umini);
        return -1;
    }
    
    if (configSerial(fd_umini, 9600) < 0) {
        printf("配置 UMini-WS 串口失败\n");
        close(fd_osa);
        close(fd_umini);
        return -1;
    }
    
    printf("======================================================================================\n");
    printf("%-6s %-16s %-16s %-12s %-12s %-12s %-12s\n", 
           "采样#", "OSA-15N风速(m/s)", "UMini-WS风速(m/s)", 
           "风向(°)", "风速差(m/s)", "相对差值(%)",    "状态");
    printf("======================================================================================\n");
    
    int sample_count = 0;
    int max_samples = 10;
    int fail_count = 0;
    const int max_fail = 5;
    
    float osa_speeds[10] = {0};
    float umini_speeds[10] = {0};
    float umini_dirs[10] = {0};
    float diff_percents[10] = {0};
    int valid_samples = 0;
    
    while (sample_count < max_samples && fail_count < max_fail) {
        float osa_speed = 0;
        float umini_speed = 0;
        float umini_dir = 0;
        
        // 读取OSA-15N
        int osa_ret = readOSA15N(fd_osa, &osa_speed);
        
        // 稍作延迟，让总线空闲
        usleep(100000);
        
        // 读取UMini-WS
        int umini_ret = readUMiniWS(fd_umini, &umini_speed, &umini_dir);
        
        sample_count++;
        
        if (osa_ret == 0 && umini_ret == 0) {
            float diff = fabs(osa_speed - umini_speed);
            // 计算相对差值：|OSA - UMini| / UMini * 100%
            float diff_percent = 0;
            if (umini_speed > 0.01) {  // 避免除零，风速大于0.01m/s时才计算
                diff_percent = (diff / umini_speed) * 100.0f;
            } else if (osa_speed > 0.01) {
                // 如果UMini风速接近0，用OSA作为参考
                diff_percent = (diff / osa_speed) * 100.0f;
            } else {
                // 两个都接近0，差值百分比设为0
                diff_percent = 0;
            }
            
            osa_speeds[valid_samples] = osa_speed;
            umini_speeds[valid_samples] = umini_speed;
            umini_dirs[valid_samples] = umini_dir;
            diff_percents[valid_samples] = diff_percent;
            valid_samples++;
            fail_count = 0;
            
            printf("%-6d %-16.2f %-16.2f %-12.1f %-12.3f %-12.2f %s\n",
                   sample_count, osa_speed, umini_speed, umini_dir, 
                   diff, diff_percent, "成功");
        } else {
            fail_count++;
            char status[20];
            if (osa_ret < 0 && umini_ret < 0) {
                sprintf(status, "两个都失败");
            } else if (osa_ret < 0) {
                sprintf(status, "OSA失败");
            } else {
                sprintf(status, "UMini失败");
            }
            
            printf("%-6d %-16s %-16s %-12s %-12s %-12s %s\n",
                   sample_count, "---", "---", "---", "---", "---", status);
        }
        
        // 等待1秒再采样
        sleep(1);
    }
    
    // 计算统计结果
    if (valid_samples > 0) {
        float sum_osa = 0, sum_umini = 0, sum_diff = 0, sum_diff_percent = 0;
        float max_diff = 0, min_diff = 9999;
        float max_diff_percent = 0, min_diff_percent = 9999;
        float sum_umini_dirs = 0;
        
        for (int i = 0; i < valid_samples; i++) {
            sum_osa += osa_speeds[i];
            sum_umini += umini_speeds[i];
            sum_umini_dirs += umini_dirs[i];
            
            float diff = fabs(osa_speeds[i] - umini_speeds[i]);
            sum_diff += diff;
            if (diff > max_diff) max_diff = diff;
            if (diff < min_diff) min_diff = diff;
            
            float diff_percent = diff_percents[i];
            sum_diff_percent += diff_percent;
            if (diff_percent > max_diff_percent) max_diff_percent = diff_percent;
            if (diff_percent < min_diff_percent) min_diff_percent = diff_percent;
        }
        
        float avg_osa = sum_osa / valid_samples;
        float avg_umini = sum_umini / valid_samples;
        float avg_diff = sum_diff / valid_samples;
        float avg_diff_percent = sum_diff_percent / valid_samples;
        float accuracy_diff = avg_umini - avg_osa;
        
        // 计算平均相对差值：平均差值绝对值 / UMini平均值 * 100%
        float avg_diff_percent_abs = 0;
        if (avg_umini > 0.01) {
            avg_diff_percent_abs = (avg_diff / avg_umini) * 100.0f;
        } else if (avg_osa > 0.01) {
            avg_diff_percent_abs = (avg_diff / avg_osa) * 100.0f;
        } else {
            avg_diff_percent_abs = 0;
        }
        
        printf("\n");
        printf("================================================================\n");
        printf("                   统计结果\n");
        printf("================================================================\n");
        printf("有效采样数:            %d\n", valid_samples);
        printf("OSA-15N平均风速:       %.2f m/s\n", avg_osa);
        printf("UMini-WS平均风速:      %.2f m/s\n", avg_umini);
        printf("UMini-WS平均风向:      %.1f °\n", sum_umini_dirs / valid_samples);
        printf("-----------------------------------------------------------------\n");
        printf("平均风速差:    %.3f m/s\n", avg_diff);
        printf("最大风速差:            %.3f m/s\n", max_diff);
        printf("最小风速差:            %.3f m/s\n", min_diff);
        printf("------------------------------------------------------------------\n");
        printf("相对平均差值平均相对差值:  %.2f %%\n", avg_diff_percent);
        printf("  ( |OSA-UMini| / UMini * 100%% 的平均值)\n");
        printf("相对平均差值: %.2f %%\n", avg_diff_percent_abs);
        printf("  (平均 |OSA-UMini| / UMini平均值 * 100%%)\n");
        printf("最大相对差值:          %.2f %%\n", max_diff_percent);
        printf("最小相对差值:          %.2f %%\n", min_diff_percent);
        printf("------------------------------------------------------------------------\n");
        printf("风速精度差:            %.3f m/s\n", accuracy_diff);
        printf("  (UMini-WS - OSA-15N)\n");
        printf("========================================================================\n");
    } else {
        printf("\n错误: 没有获取到任何有效数据\n");
    }
    
    close(fd_osa);
    close(fd_umini);
    return 0;
}