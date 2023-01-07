/*
by PhotonSplitter7

*applicable for fixed wing drones
*reads a gyro and outputs angles 0-360
*180 degrees is level
*simple cutoff filter gets rid of small noise
*device needs to be stable on startup to calculate error rates for drift

*/

#include <Wire.h>

const int MPU = 0x68;
const int Gyro_register = 0x43;
float Gx, Gy, Gz;
float Roll = 180.0;
float Pitch = 180.0;
float Yaw = 180.0;
float Xerror, Yerror, Zerror = 0.0;
float current, past, dt = 0.0;
float avg_x, avg_y, avg_z = 0.0;

void setup() {
  Serial.begin(19200);
  Wire.beginTransmission(MPU);  
  Wire.write(0x6B);            
  Wire.write(0x00);             
  Wire.endTransmission(true);
  calc_IMU_error();
}

void loop() {
  read_gyro();
}


//read gyro updates global variables
void read_gyro() {
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
  Roll = my_filter(Roll + Gx * dt, Roll);
  Roll > 360 ? Roll = 0 : Roll < 0 ? Roll = 360
                                   : Pitch = my_filter(Pitch + Gy * dt, Pitch);
  Pitch > 360 ? Pitch = 0 : Pitch < 0 ? Pitch = 360
                                      : Yaw = my_filter(Yaw + Gz * dt, Yaw);
  Yaw > 360 ? Yaw = 0 : Yaw < 0 ? Yaw = 360
                                :

                                Serial.print(Roll);
  Serial.print("/");
  Serial.print(Pitch);
  Serial.print("/");
  Serial.print(Yaw);
  Serial.println("/");

  return;
}


//error finder for startup
void calc_IMU_error() {

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

  return;
}

//simple filter to get rid of small noise
float my_filter(float val, float last) {
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
