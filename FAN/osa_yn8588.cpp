// g++ -o osa_yn8588 osa_yn8588.cpp -std=c++11 -lm
// sudo ./osa_yn8588

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

// 读取OSA-15N数据 (地址0x31, ttyS3)
// 分辨率: 0.1m/s, 公式: 风速 = 寄存器值 × 0.1
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
                // 验证CRC
                unsigned short recv_crc = (response[bytes_read-1] << 8) | response[bytes_read-2];
                unsigned short calc_crc = calculateCRC(response, bytes_read - 2);
                if (recv_crc == calc_crc) {
                    unsigned short speed_raw = (response[3] << 8) | response[4];
                    *speed = speed_raw * 0.1f;  // OSA-15N: 倍率0.1
                    return 0;
                }
            }
        }
    }
    return -1;
}

// 读取YN8588数据 (地址0x01, ttyS0)
// 分辨率: 0.01m/s, 公式: 风速 = 寄存器值 × 0.01 (即倍率100)
int readYN8588(int fd, float* speed) {
    unsigned char request[8];
    request[0] = 0x01;
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
        if (bytes_read >= 5 && response[0] == 0x01 && response[1] == 0x03) {
            if (response[2] == 2) {
                // 验证CRC
                unsigned short recv_crc = (response[bytes_read-1] << 8) | response[bytes_read-2];
                unsigned short calc_crc = calculateCRC(response, bytes_read - 2);
                if (recv_crc == calc_crc) {
                    unsigned short speed_raw = (response[3] << 8) | response[4];
                    *speed = speed_raw * 0.01f;  // YN8588: 倍率100 (即×0.01)
                    return 0;
                }
            }
        }
    }
    return -1;
}

int main() {
    printf("============================================================\n");
    printf("         双风速传感器数据采集与对比程序\n");
    printf("============================================================\n");
    printf("OSA-15N   地址: 0x31, 串口: /dev/ttyS3, 倍率: 0.1 (分辨率0.1m/s)\n");
    printf("YN8588    地址: 0x01, 串口: /dev/ttyS0, 倍率: 0.01 (分辨率0.01m/s)\n");
    printf("============================================================\n\n");
    
    // 打开两个串口
    int fd_osa = openSerial("/dev/ttyS3");
    if (fd_osa < 0) return -1;
    
    int fd_yn8588 = openSerial("/dev/ttyS0");
    if (fd_yn8588 < 0) {
        close(fd_osa);
        return -1;
    }
    
    // 配置串口
    if (configSerial(fd_osa, 9600) < 0) {
        printf("配置 OSA-15N 串口失败\n");
        close(fd_osa);
        close(fd_yn8588);
        return -1;
    }
    
    if (configSerial(fd_yn8588, 9600) < 0) {
        printf("配置 YN8588 串口失败\n");
        close(fd_osa);
        close(fd_yn8588);
        return -1;
    }
    
    printf("======================================================================================\n");
    printf("%-6s %-16s %-16s %-12s %-12s %-12s %-12s\n", 
           "采样#", "OSA-15N风速(m/s)", "YN8588风速(m/s)", 
           "风速差(m/s)", "相对差值(%)", "累计平均差", "状态");
    printf("======================================================================================\n");
    
    int sample_count = 0;
    int max_samples = 10;
    int fail_count = 0;
    const int max_fail = 5;
    
    float osa_speeds[20] = {0};
    float yn8588_speeds[20] = {0};
    float diff_percents[20] = {0};
    float cumulative_avg_diff[20] = {0};
    int valid_samples = 0;
    
    while (sample_count < max_samples && fail_count < max_fail) {
        float osa_speed = 0;
        float yn8588_speed = 0;
        
        // 读取OSA-15N
        int osa_ret = readOSA15N(fd_osa, &osa_speed);
        
        // 稍作延迟，让总线空闲
        usleep(100000);
        
        // 读取YN8588
        int yn8588_ret = readYN8588(fd_yn8588, &yn8588_speed);
        
        sample_count++;
        
        if (osa_ret == 0 && yn8588_ret == 0) {
            float diff = fabs(osa_speed - yn8588_speed);
            // 计算相对差值
            float diff_percent = 0;
            if (yn8588_speed > 0.01) {
                diff_percent = (diff / yn8588_speed) * 100.0f;
            } else if (osa_speed > 0.01) {
                diff_percent = (diff / osa_speed) * 100.0f;
            } else {
                diff_percent = 0;
            }
            
            osa_speeds[valid_samples] = osa_speed;
            yn8588_speeds[valid_samples] = yn8588_speed;
            diff_percents[valid_samples] = diff_percent;
            
            // 计算累计平均差
            float sum_diff = 0;
            for (int i = 0; i <= valid_samples; i++) {
                sum_diff += fabs(osa_speeds[i] - yn8588_speeds[i]);
            }
            cumulative_avg_diff[valid_samples] = sum_diff / (valid_samples + 1);
            
            valid_samples++;
            fail_count = 0;
            
            printf("%-6d %-16.2f %-16.2f %-12.3f %-12.2f %-12.3f %s\n",
                   sample_count, osa_speed, yn8588_speed, 
                   diff, diff_percent, cumulative_avg_diff[valid_samples-1], "成功");
        } else {
            fail_count++;
            char status[20];
            if (osa_ret < 0 && yn8588_ret < 0) {
                sprintf(status, "两个都失败");
            } else if (osa_ret < 0) {
                sprintf(status, "OSA失败");
            } else {
                sprintf(status, "YN8588失败");
            }
            
            printf("%-6d %-16s %-16s %-12s %-12s %-12s %s\n",
                   sample_count, "---", "---", "---", "---", "---", status);
        }
        
        // 等待1秒再采样
        sleep(1);
    }
    
    // 计算统计结果
    if (valid_samples > 0) {
        float sum_osa = 0, sum_yn8588 = 0, sum_diff = 0, sum_diff_percent = 0;
        float max_diff = 0, min_diff = 9999;
        float max_diff_percent = 0, min_diff_percent = 9999;
        
        for (int i = 0; i < valid_samples; i++) {
            sum_osa += osa_speeds[i];
            sum_yn8588 += yn8588_speeds[i];
            
            float diff = fabs(osa_speeds[i] - yn8588_speeds[i]);
            sum_diff += diff;
            if (diff > max_diff) max_diff = diff;
            if (diff < min_diff) min_diff = diff;
            
            float diff_percent = diff_percents[i];
            sum_diff_percent += diff_percent;
            if (diff_percent > max_diff_percent) max_diff_percent = diff_percent;
            if (diff_percent < min_diff_percent) min_diff_percent = diff_percent;
        }
        
        float avg_osa = sum_osa / valid_samples;
        float avg_yn8588 = sum_yn8588 / valid_samples;
        float avg_diff = sum_diff / valid_samples;
        float avg_diff_percent = sum_diff_percent / valid_samples;
        float accuracy_diff = avg_yn8588 - avg_osa;
        
        // 计算平均相对差值
        float avg_diff_percent_abs = 0;
        if (avg_yn8588 > 0.01) {
            avg_diff_percent_abs = (avg_diff / avg_yn8588) * 100.0f;
        } else if (avg_osa > 0.01) {
            avg_diff_percent_abs = (avg_diff / avg_osa) * 100.0f;
        } else {
            avg_diff_percent_abs = 0;
        }
        
        printf("\n");
        printf("================================================================================\n");
        printf("                   统计结果\n");
        printf("================================================================================\n");
        printf("有效采样数:            %d\n", valid_samples);
        printf("OSA-15N平均风速:       %.2f m/s (分辨率0.1m/s)\n", avg_osa);
        printf("YN8588平均风速:        %.2f m/s (分辨率0.01m/s)\n", avg_yn8588);
        printf("--------------------------------------------------------------------------------\n");
        printf("平均风速差:            %.3f m/s\n", avg_diff);
        printf("最大风速差:            %.3f m/s\n", max_diff);
        printf("最小风速差:            %.3f m/s\n", min_diff);
        printf("--------------------------------------------------------------------------------\n");
        printf("相对平均差值(平均):    %.2f %%\n", avg_diff_percent);
        printf("相对平均差值(整体):    %.2f %%\n", avg_diff_percent_abs);
        printf("最大相对差值:          %.2f %%\n", max_diff_percent);
        printf("最小相对差值:          %.2f %%\n", min_diff_percent);
        printf("--------------------------------------------------------------------------------\n");
        printf("风速精度差:            %.3f m/s\n", accuracy_diff);
        printf("  (YN8588 - OSA-15N)\n");
        printf("================================================================================\n");
    } else {
        printf("\n错误: 没有获取到任何有效数据\n");
    }
    
    close(fd_osa);
    close(fd_yn8588);
    return 0;
}