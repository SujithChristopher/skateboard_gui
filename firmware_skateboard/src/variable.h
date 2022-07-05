
//#include "Arduino.h"
//#include "CustomDS.h"
//#include "SerialReader.h"
//#include "variable.h"
//
//#include "Wire.h"
//#include "MPU6050_light_modified.h"
MPU6050 mpu1(Wire);


// 


//Sensor data


float gyrox1=(mpu1.getGyroX());
float gyroy1=(mpu1.getGyroY());
float gyroz1=(mpu1.getGyroZ());

float accelx1=(mpu1.getAccX());
float accely1=(mpu1.getAccY());
float accelz1=(mpu1.getAccZ());


unsigned short _hr;
unsigned short _mn;
unsigned short _se;

unsigned long _mils;

bool pinflip = false;


long p_fr;
long p_fl;
long p_rr;
long p_rl;

//uint64_t _rawTime;
unsigned long long _rawTime;

char _sync;
bool _s;

//unsigned long _mils;

unsigned short time_s[3];
// Out data buffer
::OutDataBuffer4Float outPayload;
