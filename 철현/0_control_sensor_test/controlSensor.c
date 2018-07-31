/*******************************************************************************
 *  INCLUDE FILES
 *******************************************************************************
 */
#include <stdio.h>
#include "car_lib.h"

int data_transform(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*******************************************************************************
 *  Defines
 *******************************************************************************
 */
#define LIGHT_BEEP       // to test light and beep
#define POSITION_CONTROL  // to test postion control
#define SPEED_CONTROL     // to test speed control
#define SERVO_CONTROL     // to test servo control(steering & camera position)
#define LINE_TRACE              // to test line trace sensor
#define DISTANCE_SENSOR     // to test distance sensor

/*******************************************************************************
 *  Functions
 *******************************************************************************
 */
void main(void)
{
    unsigned char status;
    short speed;
    unsigned char gain;
    int position, posInit, posDes, posRead;
    short angle;
    int channel;
    int data;
    char sensor;
    int i, j;
    int tol;
    char byte = 0x80;

    CarControlInit();
/*
#ifdef LIGHT_BEEP
    //0. light and beep Control --------------------------------------------------
    printf("\n\n 0. light and beep control\n");
    CarLight_Write(ALL_ON);
    usleep(1000000);
    CarLight_Write(ALL_OFF);

    Alarm_Write(ON);
    usleep(100000);
    Alarm_Write(OFF);

    CarLight_Write(FRONT_ON);
    usleep(1000000);
    CarLight_Write(ALL_OFF);
    CarLight_Write(REAR_ON);
    usleep(1000000);
    CarLight_Write(ALL_OFF);

    Alarm_Write(ON);
    usleep(100000);
    Alarm_Write(OFF);

    Winker_Write(ALL_ON);
    usleep(1000000);
    Winker_Write(ALL_OFF);
    Winker_Write(LEFT_ON);
    usleep(1000000);
    Winker_Write(ALL_OFF);
    Winker_Write(RIGHT_ON);
    usleep(1000000);
    Winker_Write(ALL_OFF);
#endif

#ifdef POSITION_CONTROL
     // 1. position control -------------------------------------------------------
    printf("\n\n 1. position control\n");

    //jobs to be done beforehand;
    SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
    speed = 50; // speed set     --> speed must be set when using position controller
    DesireSpeed_Write(speed);

    //control on/off
    status = PositionControlOnOff_Read();
    printf("PositionControlOnOff_Read() = %d\n", status);
    PositionControlOnOff_Write(CONTROL);

    //position controller gain set
    gain = PositionProportionPoint_Read();    // default value = 10, range : 1~50
    printf("PositionProportionPoint_Read() = %d\n", gain);
    gain = 30;
    PositionProportionPoint_Write(gain);

    //position write
    posInit = 0;  //initialize
    EncoderCounter_Write(posInit);

    //position set
    posDes = 500;
    position = posInit+posDes;
    DesireEncoderCount_Write(position);

    position=DesireEncoderCount_Read();
    printf("DesireEncoderCount_Read() = %d\n", position);

    tol = 100;    // tolerance
    while(abs(posRead-position)>tol)
    {
        posRead=EncoderCounter_Read();
        printf("EncoderCounter_Read() = %d\n", posRead);
    }
    sleep(1);
#endif

#ifdef SPEED_CONTROL
    // 2. speed control ----------------------------------------------------------
    printf("\n\n 2. speed control\n");

    //jobs to be done beforehand;
    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!

    //control on/off
    status=SpeedControlOnOff_Read();
    printf("SpeedControlOnOff_Read() = %d\n", status);
    SpeedControlOnOff_Write(CONTROL);

    //speed controller gain set
    //P-gain
    gain = SpeedPIDProportional_Read();        // default value = 10, range : 1~50
    printf("SpeedPIDProportional_Read() = %d \n", gain);
    gain = 20;
    SpeedPIDProportional_Write(gain);

    //I-gain
    gain = SpeedPIDIntegral_Read();        // default value = 10, range : 1~50
    printf("SpeedPIDIntegral_Read() = %d \n", gain);
    gain = 20;
    SpeedPIDIntegral_Write(gain);

    //D-gain
    gain = SpeedPIDDifferential_Read();        // default value = 10, range : 1~50
    printf("SpeedPIDDefferential_Read() = %d \n", gain);
    gain = 20;
    SpeedPIDDifferential_Write(gain);

    //speed set
    speed = DesireSpeed_Read();
    printf("DesireSpeed_Read() = %d \n", speed);
    speed = -10;
    DesireSpeed_Write(speed);

    sleep(2);  //run time

    speed = DesireSpeed_Read();
    printf("DesireSpeed_Read() = %d \n", speed);

    speed = 0;
    DesireSpeed_Write(speed);
    sleep(1);
#endif

#ifdef SERVO_CONTROL
    // 3. servo control ----------------------------------------------------------
    printf("\n\n 3. servo control\n");
    //steer servo set
    angle = SteeringServoControl_Read();
    printf("SteeringServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

    angle = 1200;
    SteeringServoControl_Write(angle);

    //camera x servo set
    angle = CameraXServoControl_Read();
    printf("CameraXServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

    angle = 1400;
    CameraXServoControl_Write(angle);

    //camera y servo set
    angle = CameraYServoControl_Read();
    printf("CameraYServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

    angle = 1400;
    CameraYServoControl_Write(angle);

    sleep(1);
    angle = 1500;
    SteeringServoControl_Write(angle);
    CameraXServoControl_Write(angle);
    CameraYServoControl_Write(angle);
#endif
*/
#ifdef LINE_TRACE
    // 4. line trace sensor --------------------------------------------------------
    sensor = LineSensor_Read();        // black:1, white:0
    printf("LineSensor_Read() = ");
    for(i=0; i<8; i++)
    {
        if((i % 4) ==0) printf(" ");
        if((sensor & byte)) printf("1");
        else printf("0");
        sensor = sensor << 1;
    }
    printf("\n");
    printf("LineSensor_Read() = %d \n", sensor);
#endif
/*
#ifdef DISTANCE_SENSOR
    // 5. distance sensor --------------------------------------------------------
    printf("\n\n 4. distance sensor\n");
    for(i=0; i<1000; i++)
    {
        printf("Please input ADC channel number\n");
        scanf("%d", &channel);
        for(j=0; j<10000; j++)
        {
            data = DistanceSensor(channel);

            int volt = data_transform(data, 0 , 4095 , 0 , 5000);

            printf("channel = %d, distance = 0x%04X(%d) \n", channel, data, data);
            int debug = (27.61 / (volt - 0.1696))*1000;

            printf("%d cm \n", debug);
            usleep(100000);
        }
    }
#endif
*/
}
