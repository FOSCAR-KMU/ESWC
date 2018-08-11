/*******************************************************************************
 *  INCLUDE FILES
 *******************************************************************************
 */
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>
#include "car_lib.h"

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */
//#define BAUDRATE B115200
//#define MODEDEVICE "/dev/ttyS0" // "/dev/ttyS0" is used for debug terminal

#define BAUDRATE        B19200
#define SERIAL_DEVICE   "/dev/ttyS2"  // ttyHS0, ttyHS1, ttyHS3 are available
#define I2C_DEVICE      "/dev/i2c-2"

static int uart_fd;
static int i2c_fd;

/*******************************************************************************
 *  Functions
 *******************************************************************************
 */
void CarControlInit(void)
{
  char fd_serial[20];
  struct termios oldtio, newtio;

  char *dev = I2C_DEVICE;
  int addr = 0x4b;
  int r;

  // UART configuration
  strcpy(fd_serial, SERIAL_DEVICE); //FFUART

  uart_fd = open(fd_serial, O_RDWR | O_NOCTTY );
  if (uart_fd <0) {
    printf("Serial %s  Device Err\n", fd_serial );
    exit(1);
  }
  printf("CarControlInit(void) Uart Device : %s\n", SERIAL_DEVICE);

  tcgetattr(uart_fd,&oldtio); /* save current port settings */
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD; // | CRTSCTS;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
  newtio.c_cc[VMIN]     = 1;   /* blocking read until 8 chars received */

  tcflush(uart_fd, TCIFLUSH);
  tcsetattr(uart_fd,TCSANOW,&newtio);

  // I2C configuration
  i2c_fd = open(dev, O_RDWR );
  if(i2c_fd < 0)
  {
    perror("Opening i2c device node\n");
    exit(1);
  }

  r = ioctl(i2c_fd, I2C_SLAVE, addr);
  if(r < 0)
  {
    perror("Selecting i2c device\n");
  }
}

void CarLight_Write(char status)
{
    unsigned char buf[8];

    buf[0] = 0xa0;
    if(status == ALL_ON)
    {
        buf[1] = 0x03; //code length (1) + 2
        buf[2] = 0x01; //write(1)
        buf[3] = ALL_ON;    // all on = 3
        buf[4] = 0xa7; //checksum = a0 + 03 + 01 + 03

        printf("Front and Rear Light ON\n");
    }
    else if(status == FRONT_ON)
    {
        buf[1] = 0x03; //code length (1) + 2
        buf[2] = 0x01; //write(1)
        buf[3] = FRONT_ON;    // front on = 1
        buf[4] = 0xa5; //checksum = a0 + 03 + 01 + 01

        printf("Front Light ON\n");
    }
    else if(status == REAR_ON)
    {
        buf[1] = 0x03; //code length (1) + 2
        buf[2] = 0x01; //write(1)
        buf[3] = REAR_ON;    // rear on = 2
        buf[4] = 0xa6; //checksum = a0 + 03 + 01 + 02

        printf("Rear Light ON\n");
    }
    else
    {
        buf[1] = 0x03; //code length (1) + 2
        buf[2] = 0x01; //write(1)
        buf[3] = ALL_OFF;    // all off = 0
        buf[4] = 0xa4; //checksum = a0 + 03 + 01 + 00

        printf("Front and Rear Light OFF\n");
    }
    write(uart_fd, &buf[0], 5);
}

void Alarm_Write(char status)
{
    unsigned char buf[8];

    buf[0] = 0xa2;
    if(status == ON)
    {
        buf[1] = 0x03; //code length (1) + 2
        buf[2] = 0x01; //write(1)
        buf[3] = 0x64; //0.01s (1sec = 100 = 0x64)
        buf[4] = 0x0a; //checksum = a2 + 03 + 01 + 64 = 10a

        printf("Alarm ON\n");
    }
    else
    {
        buf[1] = 0x03; //code length (1) + 2
        buf[2] = 0x01; //write(1)
        buf[3] = 0x00; //0.00s
        buf[4] = 0xa6; //checksum = a2 + 03 + 01 + 00

        printf("Alarm OFF\n");
    }
    write(uart_fd, &buf[0], 5);
}

void Winker_Write(char status)
{
    unsigned char buf[8];

    buf[0] = 0xa1;
    if(status == ALL_ON)
    {
        buf[1] = 0x03; //code length (1) + 2
        buf[2] = 0x01; //write(1)
        buf[3] = ALL_ON;    // all on = 3
        buf[4] = 0xa8; //checksum = a1 + 03 + 01 + 03

        printf("Left and Right Winker ON\n");
    }
    else if(status == RIGHT_ON)
    {
        buf[1] = 0x03; //code length (1) + 2
        buf[2] = 0x01; //write(1)
        buf[3] = RIGHT_ON;    // right on = 1
        buf[4] = 0xa6; //checksum = a1 + 03 + 01 + 01

        printf("Right Winker ON\n");
    }
    else if(status == LEFT_ON)
    {
        buf[1] = 0x03; //code length (1) + 2
        buf[2] = 0x01; //write(1)
        buf[3] = LEFT_ON;    // left on = 2
        buf[4] = 0xa7; //checksum = a1 + 03 + 01 + 02

        printf("Left Winker ON\n");
    }
    else
    {
        buf[1] = 0x03; //code length (1) + 2
        buf[2] = 0x01; //write(1)
        buf[3] = ALL_OFF;    // all off = 0
        buf[4] = 0xa5; //checksum = a1 + 03 + 01 + 00

        printf("Left and Right Winker OFF\n");
    }
    write(uart_fd, &buf[0], 5);
}

char SpeedControlOnOff_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0x90;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0x94; //checksum = 90 + 02 + 02

    //  printf("Read Speed Control OnOff\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 4);

    return read_buf[2];
}

void SpeedControlOnOff_Write(char status)
{
    unsigned char buf[8];

    buf[0] = 0x90;
    buf[1] = 0x03; //code length (1) + 2
    buf[2] = 0x01; //write(1)
    buf[3] = status; //UNCONTROL=0 CONTROL=1
    buf[4] = 0x94 + buf[3]; //checksum = 90 + 03 + 01 + buf[3]

    //printf("SpeedControlOnOff_Write(void) = %d\n", buf[3]);
    write(uart_fd, &buf[0], 5);
}

signed short DesireSpeed_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0x91;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0x95; //checksum = 91 + 02 + 02

//  printf("Read Desire Speed\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 5);

    return ((signed short)(read_buf[3]<<8) + (signed short)(read_buf[2]));
}

void DesireSpeed_Write(signed short speed)
{
    unsigned char buf[8];

    buf[0] = 0x91;
    buf[1] = 0x04; //code length (2) + 2
    buf[2] = 0x01; //write(1)
    buf[3] = speed & 0x00ff; //bottom byte
    buf[4] = (speed >> 8) & 0x00ff; //top byte
    buf[5] = 0x96 + buf[3] + buf[4]; //checksum

    //printf("DesireSpeed_Write(void) = %d\n", speed);
    write(uart_fd, &buf[0], 6);
}

unsigned char SpeedPIDProportional_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0x92;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0x96; //checksum = 92 + 02 + 02

//  printf("Read Speed PID Proportional\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 4);

    return read_buf[2];
}

void SpeedPIDProportional_Write(unsigned char gain)
{
    unsigned char buf[8];

    buf[0] = 0x92;
    buf[1] = 0x03; //code length (1) + 2
    buf[2] = 0x01; //write(1)
    buf[3] = gain;
    buf[4] = 0x96 + buf[3]; //checksum = 90 + 03 + 01 + buf[3]

    printf("SpeedPIDProportional_WriteH = %d\n", buf[3]);
    write(uart_fd, &buf[0], 5);
}

unsigned char SpeedPIDIntegral_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0x93;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0x97; //checksum = 93 + 02 + 02

//  printf("Read Speed PID Integral\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 4);

    return read_buf[2];
}

void SpeedPIDIntegral_Write(unsigned char gain)
{
    unsigned char buf[8];

    buf[0] = 0x93;
    buf[1] = 0x03; //code length (1) + 2
    buf[2] = 0x01; //write(1)
    buf[3] = gain;
    buf[4] = 0x97 + buf[3]; //checksum = 93 + 03 + 01 + buf[3]

    printf("SpeedPIDIntegral_Write(void) = %d\n", buf[3]);
    write(uart_fd, &buf[0], 5);
}

unsigned char SpeedPIDDifferential_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0x94;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0x98; //checksum = 94 + 02 + 02

//  printf("Read Speed PID Differential\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 4);

    return read_buf[2];
}

void SpeedPIDDifferential_Write(unsigned char gain)
{
    unsigned char buf[8];

    buf[0] = 0x94;
    buf[1] = 0x03; //code length (1) + 2
    buf[2] = 0x01; //write(1)
    buf[3] = gain;
    buf[4] = 0x98 + buf[3]; //checksum = 94 + 03 + 01 + buf[3]

    printf("SpeedPIDDifferential_Write(void) = %d\n", buf[3]);
    write(uart_fd, &buf[0], 5);
}

char PositionControlOnOff_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0x96;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0x9a; //checksum = 96 + 02 + 02

//  printf("Read Position Control OnOff\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 4);

    return read_buf[2];

}

void PositionControlOnOff_Write(char status)
{
    unsigned char buf[8];

    buf[0] = 0x96;
    buf[1] = 0x03; //code length (1) + 2
    buf[2] = 0x01; //write(1)
    buf[3] = status; //UNCONTROL=0 CONTROL=1
    buf[4] = 0x9a + buf[3]; //checksum = 9a + 03 + 01 + buf[3]

    printf("PositionControlOnOff_Write(void) = %d\n", buf[3]);
    write(uart_fd, &buf[0], 5);
}

unsigned char PositionProportionPoint_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0x98;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0x9c; //checksum = 98 + 02 + 02

//  printf("Read Position Proportion Point\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 4);

    return read_buf[2];
}

void PositionProportionPoint_Write(unsigned char gain)
{
    unsigned char buf[8];

    buf[0] = 0x98;
    buf[1] = 0x03; //code length (1) + 2
    buf[2] = 0x01; //write(1)
    buf[3] = gain;
    buf[4] = 0x9c + buf[3]; //checksum = 98 + 03 + 01 + buf[3]

    printf("PositionProportionPoint_Write(void) = %d\n", buf[3]);
    write(uart_fd, &buf[0], 5);
}

signed int DesireEncoderCount_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0x97;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0x9b; //checksum = 97 + 02 + 02

//  printf("Read Desire position\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 7);

    return ((signed int)(read_buf[5]<<24) + (signed int)(read_buf[4]<<16) + (signed int)(read_buf[3]<<8) + (signed int)(read_buf[2]));
}

void DesireEncoderCount_Write(signed int position)
{
    unsigned char buf[8];

    buf[0] = 0x97;
    buf[1] = 0x06; //code length (6) + 2
    buf[2] = 0x01; //write(1)
    buf[3] = position & 0x000000ff; //bottom byte
    buf[4] = (position >> 8) & 0x000000ff; //3rd byte
    buf[5] = (position >> 16) & 0x000000ff; //2nd byte
    buf[6] = (position >> 24) & 0x000000ff; //top byte
    buf[7] = 0x9e + buf[3] + buf[4] + buf[5] + buf[6]; //checksum

    //printf("DesireEncoderCount_Write(void) = %d\n", position);
    write(uart_fd, &buf[0], 8);
}

signed int EncoderCounter_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0xb0;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0xb4; //checksum = b0 + 02 + 02

//  printf("Read Encoder Counter\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 7);

    return ((signed int)(read_buf[5]<<24) + (signed int)(read_buf[4]<<16) + (signed int)(read_buf[3]<<8) + (signed int)(read_buf[2]));
}

void EncoderCounter_Write(signed int position)
{
    unsigned char buf[8];

    buf[0] = 0xb0;
    buf[1] = 0x06; //code length (6) + 2
    buf[2] = 0x01; //write(1)
    buf[3] = position & 0x000000ff; //bottom byte
    buf[4] = (position >> 8) & 0x000000ff; //3rd byte
    buf[5] = (position >> 16) & 0x000000ff; //2nd byte
    buf[6] = (position >> 24) & 0x000000ff; //top byte
    buf[7] = 0xb7 + buf[3] + buf[4] + buf[5] + buf[6]; //checksum

    //printf("EncoderCounter_Write(void) = %d\n", position);
    write(uart_fd, &buf[0], 8);
}

signed short SteeringServoControl_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0xa3;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0xa7; //checksum = a3 + 02 + 02

//  printf("Read Steering Servo Angle\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 5);

    return ((signed short)(read_buf[3]<<8) + (signed short)(read_buf[2]));
}

void SteeringServoControl_Write(signed short angle)
{
    unsigned char buf[8];

    buf[0] = 0xa3;
    buf[1] = 0x04; //code length (2) + 2
    buf[2] = 0x01; //write(1)
    buf[3] = angle & 0x00ff; //bottom byte
    buf[4] = (angle >> 8) & 0x00ff; //top byte
    buf[5] = 0xa8 + buf[3] + buf[4]; //checksum

    //printf("SteeringServoControl_Write(void) = %d\n", angle);
    write(uart_fd, &buf[0], 6);
}

signed short CameraXServoControl_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0xa5;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0xa9; //checksum = a5 + 02 + 02

//  printf("Read Camera X Servo Angle\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 5);

    return ((signed short)(read_buf[3]<<8) + (signed short)(read_buf[2]));
}

void CameraXServoControl_Write(signed short angle)
{
    unsigned char buf[8];

    buf[0] = 0xa5;
    buf[1] = 0x04; //code length (2) + 2
    buf[2] = 0x01; //write(1)
    buf[3] = angle & 0x00ff; //bottom byte
    buf[4] = (angle >> 8) & 0x00ff; //top byte
    buf[5] = 0xaa + buf[3] + buf[4]; //checksum

    //printf("CameraXServoControl_Write(void) = %d\n", angle);
    write(uart_fd, &buf[0], 6);
}

signed short CameraYServoControl_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0xa7;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0xab; //checksum = a7 + 02 + 02

//  printf("Read Camera Y Angle\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 5);

    return ((signed short)(read_buf[3]<<8) + (signed short)(read_buf[2]));
}

void CameraYServoControl_Write(signed short angle)
{
    unsigned char buf[8];

    buf[0] = 0xa7;
    buf[1] = 0x04; //code length (2) + 2
    buf[2] = 0x01; //write(1)
    buf[3] = angle & 0x00ff; //bottom byte
    buf[4] = (angle >> 8) & 0x00ff; //top byte
    buf[5] = 0xac + buf[3] + buf[4]; //checksum

    //printf("CameraYServoControl_Write(void) = %d\n", angle);
    write(uart_fd, &buf[0], 6);
}

unsigned char LineSensor_Read(void)
{
    unsigned char buf[8];
    unsigned char read_buf[8];

    buf[0] = 0xb1;
    buf[1] = 0x02; //code length (0) + 2
    buf[2] = 0x02; //read(2)
    buf[3] = 0xb5; //checksum = b1 + 02 + 02

//  printf("Read Line Trace Sensor\n");
    write(uart_fd, &buf[0], 4);
    read(uart_fd, &read_buf[0], 4);

    return read_buf[2];
}

int DistanceSensor(int channel)
{
    unsigned char buf[8];
    unsigned char command;
    unsigned char value[2];
    useconds_t delay = 2000;
    int data;
    int r;

    switch(channel)
    {
    case 1 : command = 0x8c; break;
    case 2 : command = 0xcc; break;
    case 3 : command = 0x9c; break;
    case 4 : command = 0xdc; break;
    case 5 : command = 0xac; break;
    case 6 : command = 0xec; break;
    default : printf("channel error.\n"); break;
    }
    r = write(i2c_fd, &command, 1);
    usleep(delay);

    r = read(i2c_fd, value, 2);
    if(r != 2)
    {
        perror("reading i2c device\n");
    }
    usleep(delay);

    data = (int)((value[0] & 0b00001111)<<8)+value[1];

    return data;
}
