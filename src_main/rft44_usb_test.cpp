/*
Simple ROBOTOUS RFT44-SB01 USB/UART read test.

The ROBOTOUS USB interface appears on Linux as a virtual serial port,
usually /dev/ttyUSB0. This program uses Rft44UsbSensor to start
continuous force/torque output and print Fx, Fy, Fz, Tx, Ty, Tz.

Inputs:
    USB serial port from the sensor, e.g. /dev/ttyUSB0.

Outputs:
    Printed force/torque samples in the terminal.

Usage:
    ./rft44_usb_test
    ./rft44_usb_test /dev/ttyUSB0
    ./rft44_usb_test /dev/ttyUSB0 115200 200
    ./rft44_usb_test /dev/ttyUSB0 115200 200 bias
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

#include "rft44_usb_sensor.h"

// -------------------- User Settings --------------------
std::string serial_port = "/dev/ttyUSB0";  // TODO USB serial port
int baud_rate = 115200;  // TODO ROBOTOUS default UART baud rate
int sample_count = 1000;  // TODO number of samples to print
bool bias_at_start = false;  // TODO true: tare/bias sensor at start
int read_timeout_ms = 1000;  // TODO serial read timeout [ms]

float force_divider = 50.0f;  // TODO ROBOTOUS sample default
float torque_divider = 2000.0f;  // TODO ROBOTOUS sample default

int main(int argc, char** argv)
{
    if (argc >= 2)
        serial_port = argv[1];
    if (argc >= 3)
        baud_rate = atoi(argv[2]);
    if (argc >= 4)
        sample_count = atoi(argv[3]);
    if (argc >= 5)
        bias_at_start = rft44_argument_requests_bias(argv[4]);

    printf("ROBOTOUS RFT44-SB01 USB test\n");
    printf("Port: %s\n", serial_port.c_str());
    printf("Baud: %d\n", baud_rate);
    printf("Samples: %d\n", sample_count);
    printf("Bias at start: %s\n", bias_at_start ? "yes" : "no");

    Rft44UsbSensor sensor;
    sensor.set_port(serial_port);
    sensor.set_baud_rate(baud_rate);
    sensor.set_dividers(force_divider, torque_divider);

    if (!sensor.open_sensor())
        return 1;

    if (bias_at_start)
    {
        printf("Sending bias/tare command...\n");
        sensor.bias();
        usleep(1000 * 1000);
    }

    printf("Starting continuous force/torque output...\n");
    if (!sensor.start_streaming())
        return 1;

    printf("Fx[N], Fy[N], Fz[N], Tx, Ty, Tz, overload\n");

    int printed_count = 0;
    while (printed_count < sample_count)
    {
        Rft44SensorSample sample;
        if (!sensor.read_sample(&sample, read_timeout_ms))
        {
            printf("No valid packet received.\n");
            continue;
        }

        printf("% .3f, % .3f, % .3f, % .4f, % .4f, % .4f, 0x%02X\n",
               sample.fx, sample.fy, sample.fz,
               sample.tx, sample.ty, sample.tz,
               sample.overload_status);

        printed_count++;
    }

    printf("Stopping continuous output...\n");
    sensor.stop_streaming();
    sensor.close_sensor();

    return 0;
}
