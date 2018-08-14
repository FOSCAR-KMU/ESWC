#ifndef _PARAM_HANDLE_
#define _PARAM_HANDLE_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */
#define OFF         0x00
#define ON          0xFF

// Light
#define ALL_OFF     0x00
#define REAR_ON     0x02
#define FRONT_ON    0x01
#define ALL_ON      0x03
#define RIGHT_ON    0x01
#define LEFT_ON     0x02

// Speed and position controller setting
#define UNCONTROL   0x00
#define CONTROL     0x01

/*******************************************************************************
 *  Functions
 *******************************************************************************
 */
void CarControlInit(void);
void CarLight_Write(char status);
void Alarm_Write(char status);
void Winker_Write(char status);
char SpeedControlOnOff_Read(void);
void SpeedControlOnOff_Write(char status);
signed short DesireSpeed_Read(void);
void DesireSpeed_Write(signed short speed);
unsigned char SpeedPIDProportional_Read(void);
void SpeedPIDProportional_Write(unsigned char gain);
unsigned char SpeedPIDIntegral_Read(void);
void SpeedPIDIntegral_Write(unsigned char gain);
unsigned char SpeedPIDDifferential_Read(void);
void SpeedPIDDifferential_Write(unsigned char gain);
char PositionControlOnOff_Read(void);
void PositionControlOnOff_Write(char status);
unsigned char PositionProportionPoint_Read(void);
void PositionProportionPoint_Write(unsigned char gain);
signed int DesireEncoderCount_Read(void);
void DesireEncoderCount_Write(signed int position);
signed int EncoderCounter_Read(void);
void EncoderCounter_Write(signed int position);
signed short SteeringServoControl_Read(void);
void SteeringServoControl_Write(signed short angle);
signed short CameraXServoControl_Read(void);
void CameraXServoControl_Write(signed short angle);
signed short CameraYServoControl_Read(void);
void CameraYServoControl_Write(signed short angle);
unsigned char LineSensor_Read(void);
int DistanceSensor(int channel);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif


/*@}*/
