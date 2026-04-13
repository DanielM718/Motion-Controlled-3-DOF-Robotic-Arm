#ifndef SERIAL_SENDER_H
#define SERIAL_SENDER_H

#include <string>
#include <termios.h>

class serial_sender {
private:
    int  fd_;
    bool open_;
    std::string rx_buffer_;
public:
    serial_sender(const std::string& port, speed_t baud = B115200);
    ~serial_sender();
    bool is_open() const;
    bool send_angles_arm(float base, float shoulder, float elbow);
    bool send_angles_wrist(float xWrist, float yWrist);
    bool send_angles_grip(float grip);
    std::string read_response();
};

#endif