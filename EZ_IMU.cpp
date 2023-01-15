#include "EZ_IMU.h"
#include <Wire.h>

void EZ_IMU::begin(){

  Wire.beginTransmission(MPU);  
  Wire.write(0x6B);            
  Wire.write(0x00);             
  Wire.endTransmission(true);


  //calc_gyro_error();
    int sample_size = 10000;

  for (int i = 0; i < sample_size; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(Gyro_register);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    Xerror += (Wire.read() << 8 | Wire.read()) / 131.0;
    Yerror += (Wire.read() << 8 | Wire.read()) / 131.0;
    Zerror += (Wire.read() << 8 | Wire.read()) / 131.0;
  }
  Xerror /= sample_size;
  Yerror /= sample_size;
  Zerror /= sample_size;

//calculate accel error


  return;
}






void EZ_IMU::read_gyro() {
  Wire.beginTransmission(MPU);
  Wire.write(Gyro_register);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  past = current;
  current = millis();
  dt = (current - past) / 1000;

  Gx = ((Wire.read() << 8 | Wire.read()) / 131.0) - Xerror;
  Gy = ((Wire.read() << 8 | Wire.read()) / 131.0) - Yerror;
  Gz = ((Wire.read() << 8 | Wire.read()) / 131.0) - Zerror;

  //filter out tiny fluctuations
  Roll = my_gyro_filter(Roll + Gx * dt, Roll); 
if(Roll > 360)
    Roll = 0;
if(Roll < 0)
    Roll = 360;
  Pitch = my_gyro_filter(Pitch + Gy * dt, Pitch); 
if(Pitch > 360)
    Pitch = 0;
if(Pitch < 0)
    Pitch = 360;
  Yaw = my_gyro_filter(Yaw + Gz * dt, Yaw);
if(Yaw > 360)
    Yaw = 0;
if(Yaw < 0)
    Yaw = 360;

  return;
}

//read accel
void EZ_IMU::read_accel(){
  //reset angle
  x_angle = 0.0;
  y_angle = 0.0;
  //take average of readings
  for (int i = 0; i < sample_rate; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(accel_register);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    ax = (Wire.read() << 8 | Wire.read()) / 16384.0;
    ay = (Wire.read() << 8 | Wire.read()) / 16384.0;
    az = (Wire.read() << 8 | Wire.read()) / 16384.0;
    //calculate angle from each reading and add up
    x_angle += (atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * 180 / PI);
    y_angle += (atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI);
  }
  //take average of angle measurements
  x_angle = my_accel_filter(x_angle / sample_rate, last_x_angle);
  y_angle = my_accel_filter(y_angle / sample_rate, last_y_angle);
  //update last angle
  last_x_angle = x_angle;
  last_y_angle = y_angle;

  return;

}






float EZ_IMU::my_gyro_filter(float val, float last) {
  //start result at 180 so start angles are always level
  float result = 180.0;
  float cutoff = 0.03;
  //acceptable values if 0 -> 360, or 360 -> 0, big change
  //angle decrease
  if (val < last) {
    if (last - val < cutoff)
      result = last;
    else
      result = val;
  }
  //angle increase
  if (val > last) {
    if (val - last < cutoff)
      result = last;
    else
      result = val;
  }
  return result;
}

//accel filter
float EZ_IMU::my_accel_filter(float angle, float last_angle) {

  float result = 0.0;
  float difference = abs(angle - last_angle);
  float cutoff = 0.25;
  if (difference > cutoff) {
    result = angle;
  } else {
    result = last_angle;
  }
  //if result close to 0 just make it so
  if(abs(result) < 0.2)
  result = 0.0;
  return result;
}
