#ifndef EZ_IMU_h
#define EZ_IMU_h

#include <Arduino.h>
#include <Wire.h>

class EZ_IMU{
public:
//variables
const int MPU = 0x68;
const int Gyro_register = 0x43;
float Gx, Gy, Gz;
float Roll = 180.0;
float Pitch = 180.0;
float Yaw = 180.0;
float Xerror, Yerror, Zerror = 0.0;
float current, past, dt = 0.0;
float avg_x, avg_y, avg_z = 0.0;
//accel variables
const int accel_register = 0x3B;
float ax, ay, az = 0.0;
float x_angle, y_angle, last_x_angle, last_y_angle = 0.0;
int sample_rate = 100;


//constructor 
//none
//functions
void begin();
void read_gyro();
void read_accel();
float my_gyro_filter(float val, float last);
float my_accel_filter(float val, float last);
};




#endif
