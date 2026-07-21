// sudo ip link set can1 down
// sudo ip link set can1 type can bitrate 500000
// sudo ip link set can1 up
// ip link show can1        配置CAN总线

//g++ -std=c++11 -o hw_can hobbywing_can.cpp
//./hw_can can1 --select-can 0x01      切到CAN模式
//./hw_can can1 --ready    使能
//./hw_can can1 --run 0x01 200 0 0 0   切到CAN模式并给油门

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace hw {
constexpr uint8_t PRIORITY_HIGHEST = 0;
constexpr uint8_t PRIORITY_MEDIUM = 16;

constexpr uint16_t MSG_ESC_THR_READY_CONTROL = 20012; // 0x4E2C
constexpr uint16_t MSG_GET_ESC_ID = 20013;            // 0x4E2D
constexpr uint16_t MSG_RAW_COMMAND = 20100;           // 0x4E84

constexpr uint8_t SRV_SET_ID = 210;          // 0xD2
constexpr uint8_t SRV_THROTTLE_SELECT = 215; // 0xD7

constexpr uint8_t SRC_NODE_ID = 0x64;
constexpr int CONTROL_PERIOD_MS = 20; // 50Hz

uint32_t message_id(uint8_t priority, uint16_t msg_type_id, uint8_t source_node_id) {
    return (uint32_t(priority & 0x1F) << 24) |
           (uint32_t(msg_type_id) << 8) |
           (source_node_id & 0x7F);
}

uint32_t service_request_id(uint8_t priority,
                            uint8_t service_type_id,
                            uint8_t destination_node_id,
                            uint8_t source_node_id) {
    return (uint32_t(priority & 0x1F) << 24) |
           (uint32_t(service_type_id) << 16) |
           (1U << 15) |
           (uint32_t(destination_node_id & 0x7F) << 8) |
           (1U << 7) |
           (source_node_id & 0x7F);
}

uint8_t tail_byte(uint8_t transfer_id) {
    return uint8_t(0xC0 | (transfer_id & 0x1F));
}

std::vector<uint8_t> pack4_throttle(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3) {
    uint16_t v[4] = {
        uint16_t(ch0 & 0x3FFF),
        uint16_t(ch1 & 0x3FFF),
        uint16_t(ch2 & 0x3FFF),
        uint16_t(ch3 & 0x3FFF),
    };

    std::vector<int> bits;
    bits.reserve(56);

    for (int k = 0; k < 4; ++k) {
        uint8_t lo = uint8_t(v[k] & 0xFF);
        uint8_t hi = uint8_t((v[k] >> 8) & 0x3F);

        for (int b = 7; b >= 0; --b) bits.push_back((lo >> b) & 1);
        for (int b = 5; b >= 0; --b) bits.push_back((hi >> b) & 1);
    }

    std::vector<uint8_t> out(7, 0);
    for (int i = 0; i < 56; ++i) {
        out[i / 8] = uint8_t((out[i / 8] << 1) | bits[i]);
    }
    return out;
}
} // namespace hw

static std::atomic<bool> running{true};

void on_signal(int) {
    running = false;
}

bool stdin_line_ready() {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);

    struct timeval tv {};
    int ret = select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);
    return ret > 0 && FD_ISSET(STDIN_FILENO, &rfds);
}

int open_can(const std::string& ifname) {
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) throw std::runtime_error("socket(PF_CAN) failed");

    struct ifreq ifr {};
    std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", ifname.c_str());
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        close(fd);
        throw std::runtime_error("SIOCGIFINDEX failed for " + ifname);
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(fd);
        throw std::runtime_error("bind() failed for " + ifname);
    }
    return fd;
}

void send_ext_frame(int fd, uint32_t can_id_29, const std::vector<uint8_t>& data) {
    if (data.size() > 8) throw std::runtime_error("CAN payload cannot exceed 8 bytes");

    struct can_frame frame {};
    frame.can_id = (can_id_29 & CAN_EFF_MASK) | CAN_EFF_FLAG;
    frame.can_dlc = data.size();
    std::memcpy(frame.data, data.data(), data.size());

    for (;;) {
        ssize_t n = write(fd, &frame, sizeof(frame));
        if (n == sizeof(frame)) return;

        if (errno == ENOBUFS || errno == EAGAIN) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        throw std::runtime_error(std::string("write(can_frame) failed: ") + std::strerror(errno));
    }
}

uint16_t parse_throttle(const char* s) {
    int v = std::stoi(s, nullptr, 0);
    if (v < 0 || v > 8191) throw std::runtime_error("throttle must be 0..8191");
    return uint16_t(v);
}

uint8_t parse_node_id(const char* s) {
    int v = std::stoi(s, nullptr, 0);
    if (v < 1 || v > 0x7D) throw std::runtime_error("ESC ID must be 1..0x7D");
    return uint8_t(v);
}

void send_ready(int fd, uint8_t& tid, bool enable) {
    uint32_t id = hw::message_id(hw::PRIORITY_MEDIUM, hw::MSG_ESC_THR_READY_CONTROL, hw::SRC_NODE_ID);
    send_ext_frame(fd, id, {uint8_t(enable ? 0xAA : 0x55), hw::tail_byte(tid++)});
}

void send_select_can(int fd, uint8_t& tid, uint8_t esc_id) {
    uint32_t id = hw::service_request_id(hw::PRIORITY_MEDIUM, hw::SRV_THROTTLE_SELECT, esc_id, hw::SRC_NODE_ID);
    send_ext_frame(fd, id, {0x00, hw::tail_byte(tid++)});
}

void send_raw_once(int fd, uint8_t& tid, uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3) {
    uint32_t id = hw::message_id(hw::PRIORITY_HIGHEST, hw::MSG_RAW_COMMAND, hw::SRC_NODE_ID);
    auto payload = hw::pack4_throttle(ch0, ch1, ch2, ch3);
    payload.push_back(hw::tail_byte(tid++));
    send_ext_frame(fd, id, payload);
}

void usage(const char* argv0) {
    std::cerr
        << "用法:\n"
        << "  " << argv0 << " can0 --get-id\n"
        << "      查询电调当前 CAN 节点 ID 和油门通道\n\n"
        << "  " << argv0 << " can0 --ready\n"
        << "      发送油门使能命令，对应 ESCThrReadyControl = 0xAA\n\n"
        << "  " << argv0 << " can0 --disable-ready\n"
        << "      发送油门失能命令，对应 ESCThrReadyControl = 0x55\n\n"
        << "  " << argv0 << " can0 --select-can ESC_ID\n"
        << "      将指定电调切换为 CAN 数字油门模式，ESC_ID 例如 0x01 或 0x7D\n\n"
        << "  " << argv0 << " can0 --set-id OLD_ID NEW_ID CHANNEL\n"
        << "      修改电调节点 ID 和油门通道；建议总线上只接一个电调时使用\n"
        << "      例如: " << argv0 << " can0 --set-id 0x7D 0x01 1\n"
        << "      表示把默认 ID 0x7D 改成 0x01，并使用 RawCommand 第 1 通道\n\n"
        << "  " << argv0 << " can0 --zero\n"
        << "      持续发送零油门，用于让电调进入有效零油门/待命状态；按 Ctrl+C 停止\n\n"
        << "  " << argv0 << " can0 --throttle CH1 CH2 CH3 CH4\n"
        << "      持续发送四通道油门，范围 0~8191；按 Ctrl+C 停止时自动发送零油门\n"
        << "      例如: " << argv0 << " can0 --throttle 1000 1000 1000 1000\n\n"
        << "  " << argv0 << " can0 --run ESC_ID CH1 CH2 CH3 CH4\n"
        << "      推荐使用：一条命令完成 CAN 油门选择、油门使能、零油门等待、目标油门输出\n"
        << "      例如: " << argv0 << " can0 --run 0x01 1000 1000 1000 1000\n\n"
        << "  " << argv0 << " can0 --live ESC_ID1 [ESC_ID2 ESC_ID3 ESC_ID4] 可同时输入1~4组id\n"
        << "      Select CAN throttle source for 1~4 ESCs, then input CH1 CH2 CH3 CH4 to update throttle; Ctrl+C to stop\n\n"
        << "  " << argv0 << " --self-test-pack\n"
        << "      自检油门打包格式是否正确；1000 四通道应输出 E8 0F A0 3E 80 FA 03\n";
}

int main(int argc, char** argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    if (argc == 2 && std::string(argv[1]) == "--self-test-pack") {
        auto p = hw::pack4_throttle(1000, 1000, 1000, 1000);
        std::cout << "pack(1000,1000,1000,1000) = ";
        for (uint8_t b : p) std::printf("%02X ", b);
        std::cout << "\nExpected: E8 0F A0 3E 80 FA 03\n";
        return 0;
    }

    if (argc < 3) {
        usage(argv[0]);
        return 2;
    }

    const std::string ifname = argv[1];
    const std::string mode = argv[2];

    try {
        int fd = open_can(ifname);
        uint8_t tid = 0;

        if (mode == "--get-id") {
            uint32_t id = hw::message_id(hw::PRIORITY_MEDIUM, hw::MSG_GET_ESC_ID, hw::SRC_NODE_ID);
            send_ext_frame(fd, id, {0x00, hw::tail_byte(tid++)});
            std::cout << "Sent get ESC ID\n";

        } else if (mode == "--ready" || mode == "--disable-ready") {
            send_ready(fd, tid, mode == "--ready");
            std::cout << "Sent throttle " << (mode == "--ready" ? "enable" : "disable") << "\n";

        } else if (mode == "--select-can") {
            if (argc != 4) { usage(argv[0]); return 2; }
            uint8_t esc_id = parse_node_id(argv[3]);
            send_select_can(fd, tid, esc_id);
            std::cout << "Sent ThrottleSelect CAN to ESC 0x" << std::hex << int(esc_id) << std::dec << "\n";

        } else if (mode == "--set-id") {
            if (argc != 6) { usage(argv[0]); return 2; }

            uint8_t old_id = parse_node_id(argv[3]);
            uint8_t new_id = parse_node_id(argv[4]);
            int channel_i = std::stoi(argv[5], nullptr, 0);
            if (channel_i < 1 || channel_i > 8) throw std::runtime_error("CHANNEL must be 1..8");

            uint32_t id = hw::service_request_id(hw::PRIORITY_MEDIUM, hw::SRV_SET_ID, old_id, hw::SRC_NODE_ID);
            send_ext_frame(fd, id, {new_id, uint8_t(channel_i), hw::tail_byte(tid++)});

            std::cout << "Sent SetID old=0x" << std::hex << int(old_id)
                      << " new=0x" << int(new_id) << std::dec
                      << " channel=" << channel_i << "\n";

        } else if (mode == "--zero" || mode == "--throttle") {
            uint16_t ch[4] = {0, 0, 0, 0};

            if (mode == "--throttle") {
                if (argc != 7) { usage(argv[0]); return 2; }
                ch[0] = parse_throttle(argv[3]);
                ch[1] = parse_throttle(argv[4]);
                ch[2] = parse_throttle(argv[5]);
                ch[3] = parse_throttle(argv[6]);
            }

            std::cout << "Sending RawCommand at 50Hz. Press Ctrl+C to stop.\n";
            while (running) {
                send_raw_once(fd, tid, ch[0], ch[1], ch[2], ch[3]);
                std::this_thread::sleep_for(std::chrono::milliseconds(hw::CONTROL_PERIOD_MS));
            }

            send_raw_once(fd, tid, 0, 0, 0, 0);
            std::cout << "Stopped and sent zero throttle\n";

        } else if (mode == "--run") {
            if (argc != 8) { usage(argv[0]); return 2; }

            uint8_t esc_id = parse_node_id(argv[3]);
            uint16_t ch[4] = {
                parse_throttle(argv[4]),
                parse_throttle(argv[5]),
                parse_throttle(argv[6]),
                parse_throttle(argv[7]),
            };

            std::cout << "Select CAN throttle source...\n";
            send_select_can(fd, tid, esc_id);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            std::cout << "Enable throttle...\n";
            send_ready(fd, tid, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            std::cout << "Sending zero throttle for 2 seconds...\n";
            for (int i = 0; i < 100 && running; ++i) {
                send_raw_once(fd, tid, 0, 0, 0, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(hw::CONTROL_PERIOD_MS));
            }

            std::cout << "Sending target throttle at 50Hz. Press Ctrl+C to stop.\n";
            while (running) {
                send_raw_once(fd, tid, ch[0], ch[1], ch[2], ch[3]);
                std::this_thread::sleep_for(std::chrono::milliseconds(hw::CONTROL_PERIOD_MS));
            }

            std::cout << "Stopping...\n";
            for (int i = 0; i < 20; ++i) {
                send_raw_once(fd, tid, 0, 0, 0, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(hw::CONTROL_PERIOD_MS));
            }
            send_ready(fd, tid, false);
            std::cout << "Stopped, sent zero throttle and disabled throttle\n";

        } else if (mode == "--live") {
            if (argc < 4 || argc > 7) { usage(argv[0]); return 2; }

            std::vector<uint8_t> esc_ids;
            for (int i = 3; i < argc; ++i) {
                esc_ids.push_back(parse_node_id(argv[i]));
            }
            uint16_t ch[4] = {0, 0, 0, 0};

            std::cout << "Select CAN throttle source...\n";
            for (uint8_t esc_id : esc_ids) {
                send_select_can(fd, tid, esc_id);
                std::cout << "Selected ESC 0x" << std::hex << int(esc_id) << std::dec << "\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            std::cout << "Enable throttle...\n";
            send_ready(fd, tid, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            std::cout << "Sending zero throttle for 2 seconds...\n";
            for (int i = 0; i < 100 && running; ++i) {
                send_raw_once(fd, tid, 0, 0, 0, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(hw::CONTROL_PERIOD_MS));
            }

            std::cout << "Input throttle as: CH1 CH2 CH3 CH4, range 0..8191. Ctrl+C to stop.\n";
            while (running) {
                if (stdin_line_ready()) {
                    int v0, v1, v2, v3;
                    if (std::cin >> v0 >> v1 >> v2 >> v3) {
                        if (v0 >= 0 && v0 <= 8191 &&
                            v1 >= 0 && v1 <= 8191 &&
                            v2 >= 0 && v2 <= 8191 &&
                            v3 >= 0 && v3 <= 8191) {
                            ch[0] = uint16_t(v0);
                            ch[1] = uint16_t(v1);
                            ch[2] = uint16_t(v2);
                            ch[3] = uint16_t(v3);
                            std::cout << "Updated throttle: "
                                      << ch[0] << " " << ch[1] << " "
                                      << ch[2] << " " << ch[3] << "\n";
                        } else {
                            std::cout << "Ignored: throttle must be 0..8191\n";
                        }
                    } else {
                        break;
                    }
                }

                send_raw_once(fd, tid, ch[0], ch[1], ch[2], ch[3]);
                std::this_thread::sleep_for(std::chrono::milliseconds(hw::CONTROL_PERIOD_MS));
            }

            std::cout << "Stopping...\n";
            for (int i = 0; i < 20; ++i) {
                send_raw_once(fd, tid, 0, 0, 0, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(hw::CONTROL_PERIOD_MS));
            }
            send_ready(fd, tid, false);
            std::cout << "Stopped, sent zero throttle and disabled throttle\n";

        } else {
            usage(argv[0]);
            return 2;
        }

        close(fd);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
