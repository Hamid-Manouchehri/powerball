/*
Implementation of the ROBOTOUS RFT44-SB01 USB/UART sensor reader.

Protocol summary from the ROBOTOUS Linux sample:
    Command packet: 11 bytes, SOP 0x55, EOP 0xAA.
    Response packet: 19 bytes, SOP 0x55, EOP 0xAA.
    Default force divider: 50.
    Default torque divider: 2000.
*/

#include "rft44_usb_sensor.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

static const unsigned char RFT44_SOP = 0x55;
static const unsigned char RFT44_EOP = 0xAA;

static const int RFT44_COMMAND_PACKET_SIZE = 11;
static const int RFT44_RESPONSE_PACKET_SIZE = 19;
static const int RFT44_AXIS_COUNT = 6;

static const unsigned char RFT44_CMD_FT_CONT = 11;
static const unsigned char RFT44_CMD_FT_CONT_STOP = 12;
static const unsigned char RFT44_CMD_SET_BIAS = 17;

static speed_t rft44_get_baud_constant(int baud)
{
    if (baud == 115200) return B115200;
#ifdef B230400
    if (baud == 230400) return B230400;
#endif
#ifdef B460800
    if (baud == 460800) return B460800;
#endif
#ifdef B921600
    if (baud == 921600) return B921600;
#endif

    return B115200;
}

static unsigned char rft44_calculate_checksum(const unsigned char* packet,
                                              int packet_size)
{
    unsigned char checksum = 0;

    for (int i = 1; i < packet_size - 2; i++)
        checksum += packet[i];

    return checksum;
}

static int16_t rft44_read_signed_16_big_endian(unsigned char high_byte,
                                               unsigned char low_byte)
{
    uint16_t raw = ((uint16_t)high_byte << 8) | (uint16_t)low_byte;
    return (int16_t)raw;
}

Rft44UsbSensor::Rft44UsbSensor()
{
    serial_port_ = "/dev/ttyUSB0";
    baud_rate_ = 115200;
    fd_ = -1;
    force_divider_ = 50.0f;
    torque_divider_ = 2000.0f;
}

Rft44UsbSensor::~Rft44UsbSensor()
{
    close_sensor();
}

void Rft44UsbSensor::set_port(const std::string& port_name)
{
    serial_port_ = port_name;
}

void Rft44UsbSensor::set_baud_rate(int baud)
{
    baud_rate_ = baud;
}

void Rft44UsbSensor::set_dividers(float force_divider,
                                  float torque_divider)
{
    force_divider_ = force_divider;
    torque_divider_ = torque_divider;
}

bool Rft44UsbSensor::open_sensor()
{
    if (fd_ >= 0)
        return true;

    fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0)
    {
        printf("Failed to open %s: %s\n",
               serial_port_.c_str(),
               strerror(errno));
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0)
    {
        printf("tcgetattr failed: %s\n", strerror(errno));
        close_sensor();
        return false;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, rft44_get_baud_constant(baud_rate_));
    cfsetospeed(&tty, rft44_get_baud_constant(baud_rate_));

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;

#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        printf("tcsetattr failed: %s\n", strerror(errno));
        close_sensor();
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
}

void Rft44UsbSensor::close_sensor()
{
    if (fd_ >= 0)
    {
        close(fd_);
        fd_ = -1;
    }
}

bool Rft44UsbSensor::is_open() const
{
    return fd_ >= 0;
}

bool Rft44UsbSensor::write_all(const unsigned char* data, int size)
{
    int written_total = 0;

    while (written_total < size)
    {
        int written = write(fd_,
                            data + written_total,
                            size - written_total);
        if (written < 0)
        {
            printf("Serial write failed: %s\n", strerror(errno));
            return false;
        }

        written_total += written;
    }

    return true;
}

bool Rft44UsbSensor::send_command(unsigned char command,
                                  unsigned char value)
{
    unsigned char packet[RFT44_COMMAND_PACKET_SIZE];
    memset(packet, 0, sizeof(packet));

    packet[0] = RFT44_SOP;
    packet[1] = command;
    packet[2] = value;
    packet[RFT44_COMMAND_PACKET_SIZE - 2] =
        rft44_calculate_checksum(packet, RFT44_COMMAND_PACKET_SIZE);
    packet[RFT44_COMMAND_PACKET_SIZE - 1] = RFT44_EOP;

    return write_all(packet, RFT44_COMMAND_PACKET_SIZE);
}

bool Rft44UsbSensor::start_streaming()
{
    return send_command(RFT44_CMD_FT_CONT, 0);
}

bool Rft44UsbSensor::stop_streaming()
{
    return send_command(RFT44_CMD_FT_CONT_STOP, 0);
}

bool Rft44UsbSensor::bias()
{
    return send_command(RFT44_CMD_SET_BIAS, 1);
}

bool Rft44UsbSensor::read_byte_with_timeout(unsigned char* value,
                                            int timeout_ms)
{
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(fd_, &read_set);

    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int result = select(fd_ + 1, &read_set, NULL, NULL, &timeout);
    if (result <= 0)
        return false;

    int received = read(fd_, value, 1);
    return received == 1;
}

bool Rft44UsbSensor::read_response_packet(unsigned char* packet,
                                          int timeout_ms)
{
    unsigned char byte_value = 0;

    while (true)
    {
        if (!read_byte_with_timeout(&byte_value, timeout_ms))
            return false;

        if (byte_value == RFT44_SOP)
            break;
    }

    packet[0] = RFT44_SOP;

    for (int i = 1; i < RFT44_RESPONSE_PACKET_SIZE; i++)
    {
        if (!read_byte_with_timeout(&packet[i], timeout_ms))
            return false;
    }

    if (packet[RFT44_RESPONSE_PACKET_SIZE - 1] != RFT44_EOP)
        return false;

    unsigned char expected_checksum =
        rft44_calculate_checksum(packet, RFT44_RESPONSE_PACKET_SIZE);

    if (packet[RFT44_RESPONSE_PACKET_SIZE - 2] != expected_checksum)
        return false;

    return true;
}

bool Rft44UsbSensor::read_sample(Rft44SensorSample* sample,
                                 int timeout_ms)
{
    unsigned char packet[RFT44_RESPONSE_PACKET_SIZE];

    if (!read_response_packet(packet, timeout_ms))
        return false;

    if (packet[1] != RFT44_CMD_FT_CONT)
        return false;

    float ft[RFT44_AXIS_COUNT] = {0.0f, 0.0f, 0.0f,
                                  0.0f, 0.0f, 0.0f};

    for (int axis = 0; axis < RFT44_AXIS_COUNT; axis++)
    {
        int byte_index = 2 + 2 * axis;
        int16_t raw_value =
            rft44_read_signed_16_big_endian(packet[byte_index],
                                            packet[byte_index + 1]);

        if (axis < 3)
            ft[axis] = (float)raw_value / force_divider_;
        else
            ft[axis] = (float)raw_value / torque_divider_;
    }

    sample->fx = ft[0];
    sample->fy = ft[1];
    sample->fz = ft[2];
    sample->tx = ft[3];
    sample->ty = ft[4];
    sample->tz = ft[5];
    sample->overload_status = packet[14];

    return true;
}

bool rft44_argument_requests_bias(const char* text)
{
    if (strcmp(text, "bias") == 0) return true;
    if (strcmp(text, "tare") == 0) return true;
    if (strcmp(text, "zero") == 0) return true;
    if (strcmp(text, "1") == 0) return true;

    return false;
}
