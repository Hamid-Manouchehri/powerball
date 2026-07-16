/*
Small ROBOTOUS RFT44-SB01 USB/UART sensor reader.

The ROBOTOUS USB interface appears on Linux as a virtual serial port,
usually /dev/ttyUSB0. This class opens that serial port, starts
continuous force/torque output, and parses Fx, Fy, Fz, Tx, Ty, Tz.

Inputs:
    USB serial port name and baud rate.

Outputs:
    Rft44SensorSample with force [N] and torque values from the sensor.
*/

#ifndef RFT44_USB_SENSOR_H
#define RFT44_USB_SENSOR_H

#include <string>

struct Rft44SensorSample
{
    float fx;
    float fy;
    float fz;
    float tx;
    float ty;
    float tz;
    unsigned char overload_status;
};

class Rft44UsbSensor
{
public:
    Rft44UsbSensor();
    ~Rft44UsbSensor();

    void set_port(const std::string& port_name);
    void set_baud_rate(int baud);
    void set_dividers(float force_divider, float torque_divider);

    bool open_sensor();
    void close_sensor();
    bool is_open() const;

    bool start_streaming();
    bool stop_streaming();
    bool bias();

    bool read_sample(Rft44SensorSample* sample, int timeout_ms);

private:
    std::string serial_port_;
    int baud_rate_;
    int fd_;
    float force_divider_;
    float torque_divider_;

    bool send_command(unsigned char command, unsigned char value);
    bool write_all(const unsigned char* data, int size);
    bool read_response_packet(unsigned char* packet, int timeout_ms);
    bool read_byte_with_timeout(unsigned char* value, int timeout_ms);
};

bool rft44_argument_requests_bias(const char* text);

#endif
