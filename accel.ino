/*
*by PhotonSplitter 7
*reads angles from accelerometer, taking 100 samples per reading and taking average
*ignores tiny changes
*this is meant to help flight controller determine horizon in slower fasion. Gyro handles short term corrections.
*/


#include <Wire.h>

const int MPU = 0x68;
const int accel_register = 0x3B;
float ax, ay, az = 0.0;
float x_angle, y_angle, last_x_angle, last_y_angle = 0.0;
int sample_rate = 100;

//start wire and serial
void setup() {
  Serial.begin(19200);
  Wire.begin();                 // Initialize comunication
  Wire.beginTransmission(MPU);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);             // Talk to the register 6B
  Wire.write(0x00);             // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);
}

void loop() {

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
  x_angle = my_filter(x_angle / sample_rate, last_x_angle);
  y_angle = my_filter(y_angle / sample_rate, last_y_angle);
  //update last angle
  last_x_angle = x_angle;
  last_y_angle = y_angle;

  Serial.print(x_angle);
  Serial.print("/");
  Serial.print(y_angle);
  Serial.println("/");
}


//cutoff filter 
float my_filter(float angle, float last_angle) {

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
