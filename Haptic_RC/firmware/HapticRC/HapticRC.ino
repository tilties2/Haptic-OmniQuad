/*
  Project: hatpic device
  Description: This sketch handles the interface of the smart servo with
  the computer throug serial communication.

  Authors: Julien Mellet, and Simon Le Berre
  Date:   May 26, 2024
  Review: August 30, 2024
*/

// - - - IMU Config - - -
#include "SparkFun_BMI270_Arduino_Library.h"
#include <Wire.h>
BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_SEC_ADDR;         // 0x69

float dt;
unsigned long current_time, prev_time;

float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev = 0.0;
float GyroY_prev = 0.0;
float GyroZ_prev = 0.0;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;        //Magnetometer LP filter parameter

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
float MagErrorX = 0.0;
float MagErrorY = 0.0; 
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX  = 0.0;
float AccErrorY  = 0.0;
float AccErrorZ  = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY = 0.0;
float GyroErrorZ = 0.0;

int mappedPitch, mappedRoll;

// - - - Core config - - -
TaskHandle_t Task1;                  // Task working on core 1 - wifi

// - - - Smart Servo Config
#include <SCServo.h> 
SMS_STS motor; 
#define RXD2 16 
#define TXD2 17 
 
int pos_M1 = 1000;
int pos_M2 = 1000;
int pos_M3 = 1000;
int pos_M4 = 1000;

int offset_M1 = 20;
int offset_M2 = -110;
int offset_M3 = 20;
int offset_M4 = 100;
int tmp;

// Torque control parameters
const float Kp = -1500.0; // Proportional gain
const float Ki = 21.0; // Integral gain
const float Ke = 0.7; // Elastic coefficient

// Control loop variables
float torqueSetpoint1 = 0.0;
float torqueSetpoint2 = 0.0;
float torqueSetpoint3 = 0.0;
float torqueSetpoint4 = 0.0;

float torqueFeedback1 = 0.0;
float torqueFeedback2 = 0.0;
float torqueFeedback3 = 0.0;
float torqueFeedback4 = 0.0;

float integralTerm1 = 0.0;
float integralTerm2 = 0.0;
float integralTerm3 = 0.0;
float integralTerm4 = 0.0;

float integralLimit = 2000.0;  // Adjust as needed

// Base motor speed (adjust as needed)
int acc = 150;
int baseSpeed = 100; // Set your base speed value
int interval = 10;

unsigned long loopEndTime = micros();
unsigned long loopStartTime = micros();

char serial_data;
String serial_trame;
unsigned long data_a, data_b, data_c, data_d;
int tmp_data_a, tmp_data_b, tmp_data_c, tmp_data_d;

void setup() {
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("STS3032 Torque control");
  
  //- imu -
  Wire.begin(); delay(100);
  while(imu.beginI2C(i2cAddress) != BMI2_OK){
    Serial.println("IMU not connected");
    delay(1000);
  }

  Serial.println("IMU connected");
  xTaskCreatePinnedToCore(get_imu,"Task1",10000, NULL,1,&Task1,1);
  
  //- motor -
  Serial2.begin(1000000, SERIAL_8N1, RXD2, TXD2);
  motor.pSerial = &Serial2;
  delay(1000);
 
  Serial.print("Current pos motor 1: "); Serial.println(motor.ReadPos(1));
  pos_M1 = motor.ReadPos(1);
  Serial.print("Current pos motor 1: "); Serial.println(pos_M1);
  Serial.print("torque motor 1: ");      Serial.println(motor.FeedBack(1));

  Serial.print("Current pos motor 2: "); Serial.println(motor.ReadPos(2));
  pos_M2 = motor.ReadPos(2);
  Serial.print("Current pos motor 2: "); Serial.println(pos_M2);
  Serial.print("torque motor 2: ");      Serial.println(motor.FeedBack(2));

  Serial.print("Current pos motor 3: "); Serial.println(motor.ReadPos(3));
  pos_M3 = motor.ReadPos(3);
  Serial.print("Current pos motor 3: "); Serial.println(pos_M3);
  Serial.print("torque motor 3: ");      Serial.println(motor.FeedBack(3));

  Serial.print("Current pos motor 4: "); Serial.println(motor.ReadPos(4));
  pos_M4 = motor.ReadPos(4);
  Serial.print("Current pos motor 4: "); Serial.println(pos_M4);
  Serial.print("torque motor 4: ");      Serial.println(motor.FeedBack(4));

  delay(2000);
 
  //Wheel mode for ID1
  //motor.WriteMode(1, 1);
  //motor.WheelMode(1);
  //motor.WriteTorqueLimit(1, 1);
  //motor.EnableTorque(1, 0);  // 0 to disconnect the motor     1 to activate torque 

}

void loop() {
  if (Serial.available()) {
    serial_data = Serial.read();
    if (String(serial_data) == "i"){
      serial_trame = "i";
      while (String(serial_data)!= "o") {
        serial_data = Serial.read();
        serial_trame = serial_trame + String(serial_data);
      }
      check_trame(serial_trame);
    }
    else{Serial.flush();}
  }
  // Record the start time of the control loop in milli
  loopStartTime = micros();

  // Read torque feedback from the sensor
  torqueFeedback1 = motor.ReadLoad(1);
  torqueFeedback2 = motor.ReadLoad(2);
  torqueFeedback3 = motor.ReadLoad(3);
  torqueFeedback4 = motor.ReadLoad(4);

  // Calculate torque error
  float torqueError1 = torqueSetpoint1 - torqueFeedback1;
  float torqueError2 = torqueSetpoint2 - torqueFeedback2;
  float torqueError3 = torqueSetpoint3 - torqueFeedback3;
  float torqueError4 = torqueSetpoint4 - torqueFeedback4;

  // Calculate integral term
  integralTerm1 += Ki * torqueError1;
  integralTerm2 += Ki * torqueError2;
  integralTerm3 += Ki * torqueError3;
  integralTerm4 += Ki * torqueError4;

  // Anti-windup - limit the integral term to prevent excessive accumulation
  integralTerm1 = constrain(integralTerm1, -integralLimit, integralLimit);
  integralTerm2 = constrain(integralTerm2, -integralLimit, integralLimit);
  integralTerm3 = constrain(integralTerm3, -integralLimit, integralLimit);
  integralTerm4 = constrain(integralTerm4, -integralLimit, integralLimit);

  // Calculate Ke dynamically based on motor position
  float dynamicKe1 = calculateKe(motor.ReadPos(1));
  float dynamicKe2 = calculateKe(motor.ReadPos(2));
  float dynamicKe3 = calculateKe(motor.ReadPos(3));
  float dynamicKe4 = calculateKe(motor.ReadPos(4));

  // Calculate spring effect with dynamic Ke
  torqueSetpoint1 = dynamicKe1 * (motor.ReadPos(1) - (1000 + offset_M1));
  torqueSetpoint2 = dynamicKe2 * (motor.ReadPos(2) - (1000 + offset_M2));
  torqueSetpoint3 = dynamicKe3 * (motor.ReadPos(3) - (1000 + offset_M3));
  torqueSetpoint4 = dynamicKe4 * (motor.ReadPos(4) - (1000 + offset_M4));
  //torqueSetpoint1 = Ke * (motor.ReadPos(1) - (1000 + offset_M1));
  //torqueSetpoint2 = Ke * (motor.ReadPos(2) - (1000 + offset_M2));
  //torqueSetpoint3 = Ke * (motor.ReadPos(3) - (1000 + offset_M3));
  //torqueSetpoint4 = Ke * (motor.ReadPos(4) - (1000 + offset_M4));

  // Adjust torque setpoint based on force sensor feedback
  torqueSetpoint1 += tmp_data_a;
  torqueSetpoint2 += tmp_data_b;
  torqueSetpoint3 += tmp_data_c;
  torqueSetpoint4 += tmp_data_d;
  
  //Serial.print("torque setpoint: ");
  //Serial.println(String(torqueSetpoint));

  // Calculate speed control output using PI control
  float speedControlOutput1 = Kp * torqueError1 + Ki * integralTerm1;
  float speedControlOutput2 = Kp * torqueError2 + Ki * integralTerm2;
  float speedControlOutput3 = Kp * torqueError3 + Ki * integralTerm3;
  float speedControlOutput4 = Kp * torqueError4 + Ki * integralTerm4;

  // Limitation values
  speedControlOutput1 = constrain(speedControlOutput1, -20000, 20000);
  speedControlOutput2 = constrain(speedControlOutput2, -20000, 20000);
  speedControlOutput3 = constrain(speedControlOutput3, -20000, 20000);
  speedControlOutput4 = constrain(speedControlOutput4, -20000, 20000);

  // Write speeds to motors
  motor.WriteSpe(1, speedControlOutput1, acc);
  motor.WriteSpe(2, speedControlOutput2, acc);
  motor.WriteSpe(3, speedControlOutput3, acc);
  motor.WriteSpe(4, speedControlOutput4, acc);

  if (loopStartTime - loopEndTime > interval) { 
    loopEndTime = micros();
    // Flush -1 when cannot read motor position
    tmp = motor.ReadPos(1);
    if (tmp != -1) {
       pos_M1 = tmp;}
    tmp = motor.ReadPos(2);
    if (tmp != -1) {
       pos_M2 = tmp;}
    tmp = motor.ReadPos(3);
    if (tmp != -1) {
       pos_M3 = tmp;}
    tmp = motor.ReadPos(4);
    if (tmp != -1) {
       pos_M4 = tmp;}
  send_cmd();
  }
}

void check_trame(String trame){
  if (trame.substring(0,2) == "ia"){trame2data(trame);}
  else {Serial.println("error");}
}

void trame2data(String trame){
  int aIndex = trame.indexOf('a');
  int bIndex = trame.indexOf('b');
  int cIndex = trame.indexOf('c');
  int dIndex = trame.indexOf('d');
  int oIndex = trame.indexOf('o');

  if((aIndex != -1) and (bIndex != -1) and (cIndex != -1) and (dIndex != -1) and (oIndex != -1)) {
    data_a = trame.substring(aIndex + 1, bIndex).toInt();
    tmp_data_a = data_a - 1000;
    data_b = trame.substring(bIndex + 1, cIndex).toInt();
    tmp_data_b = data_b - 1000;
    data_c = trame.substring(cIndex + 1, dIndex).toInt();
    tmp_data_c = data_c - 1000;
    data_d = trame.substring(dIndex + 1, oIndex).toInt();
    tmp_data_d = data_d - 1000;
  }
}

void send_cmd(){
  // Send adjusted position with offset subtracted to maintain range 0-2000 and center at 1000
  int adjustedPosM1 = pos_M1 - offset_M1;
  int adjustedPosM2 = pos_M2 - offset_M2;
  int adjustedPosM3 = pos_M3 - offset_M3;
  int adjustedPosM4 = pos_M4 - offset_M4;

  // Ensure the values sent are between 0 and 2000
  adjustedPosM1 = constrain(adjustedPosM1, 0, 2000);
  adjustedPosM2 = constrain(adjustedPosM2, 0, 2000);
  adjustedPosM3 = constrain(adjustedPosM3, 0, 2000);
  adjustedPosM4 = constrain(adjustedPosM4, 0, 2000);

  Serial.print("ia");
  Serial.print(String(adjustedPosM1));
  Serial.print("b");
  Serial.print(String(adjustedPosM2));
  Serial.print("c");
  Serial.print(String(adjustedPosM3));
  Serial.print("d");
  Serial.print(String(adjustedPosM4));
  Serial.print("p");  // attention reverse angle
  Serial.print(String(mappedRoll));
  Serial.print("r");
  Serial.print(String(mappedPitch));
  Serial.println("o");
}


// -----------------------------------IMU
void get_imu(void * pvParameters){
  while (true) {
    prev_time = current_time;      
    current_time = millis();      
    dt = (current_time - prev_time)/1000.0;
    //current_time = millis();      
    //dt = (current_time - prev_time)/1000.0;
    imu.getSensorData();
    AccX = imu.data.accelX;
    AccY = imu.data.accelY;
    AccZ = imu.data.accelZ;
    GyroX = imu.data.gyroX;
    GyroY = imu.data.gyroY;
    GyroZ = imu.data.gyroZ;
    Madgwick6DOF(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, dt);
    vTaskDelay(15);
  }
}

float calculateKe(int position) {
  // Define the center position
  int centerPosition = 1000;

  // Define the range around the center position where the notch effect is felt
  int notchRange = 50;  // Total width of the notch effect (symmetrically around center)
  int deadZoneRange = 50;  // Dead zone range around the center where no torque is applied

  // Define minimum and maximum values for Ke
  float minKe = 0.3;  // Minimum elastic coefficient (when far from center)
  float maxKe = 0.7;  // Maximum elastic coefficient (at the notch boundaries)

  // Calculate distance from the center
  int distanceFromCenter = abs(position - centerPosition);

  // Calculate Ke value based on distance from center position
  if (distanceFromCenter < deadZoneRange) {
    // Inside the dead zone: No torque is generated
    return 0.0;
  } else if (distanceFromCenter <= notchRange) {
    // Within the notch range: Apply maximum Ke value for noticeable effect
    return maxKe;
  } else {
    // Outside the notch range: Smoothly decrease Ke value
    return minKe + (maxKe - minKe) * (1.0 - float(distanceFromCenter - notchRange) / (1000 - notchRange));
  }
}


void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU  =  atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951;  //degrees
  pitch_IMU = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951;               //degrees
  yaw_IMU   = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951;  //degrees

  //Mapped angles
  if (roll_IMU < 0) {mappedRoll = map(roll_IMU,  -90, -180, 0, 1000);}
  else              {mappedRoll = map(roll_IMU,  180, 90, 1000, 2000);}
  mappedPitch = map(pitch_IMU, -90, 90, 0, 2000);

  // Border case 
  if (mappedRoll < 0)     {mappedRoll = 0;}
  if (mappedRoll > 2000)  {mappedRoll = 2000;}

  if (mappedPitch < 0)    {mappedPitch = 0;}
  if (mappedPitch > 2000) {mappedPitch = 2000;}
}
  
float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter

  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}