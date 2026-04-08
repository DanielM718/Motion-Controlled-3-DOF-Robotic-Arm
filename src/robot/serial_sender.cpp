#include "robot/serial_sender.h"
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

serial_sender::serial_sender(const std::string& port, speed_t baud)
    : fd_(-1), open_(false)
{
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        fprintf(stderr, "[serial] Can't open %s: %s\n", port.c_str(), strerror(errno));
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd_, &tty);
    cfmakeraw(&tty);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd_, TCSANOW, &tty);
    tcflush(fd_, TCIOFLUSH);

    usleep(2000000); // Arduino resets on DTR — wait 2s
    open_ = true;
    fprintf(stderr, "[serial] Opened %s @ %lu\n", port.c_str(), baud);
}

serial_sender::~serial_sender() { if (fd_ >= 0) ::close(fd_); }
bool serial_sender::is_open() const { return open_; }

bool serial_sender::send_angles(float base, float shoulder, float elbow,
                                float wrist) {
    if (!open_) return false;
    char buf[96];
    int n = snprintf(buf, sizeof(buf), "<ARM:%.1f,%.1f,%.1f,%.1f>\n",
                     base, shoulder, elbow, wrist);
    ssize_t w = ::write(fd_, buf, n);
    tcdrain(fd_);
    return w > 0;
}

std::string serial_sender::read_response() {
    if (!open_) return "";
    char buf[256];
    ssize_t n = ::read(fd_, buf, sizeof(buf) - 1);
    if (n > 0) { buf[n] = '\0'; return std::string(buf); }
    return "";
}