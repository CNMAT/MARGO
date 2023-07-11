/*

Copyright (c) 2022. The Center for New Music and Audio Technologies,
University of California, Berkeley.  Copyright (c) 2008-22, The Regents of
the University of California (Regents).

Permission to use, copy, modify, distribute, and distribute modified versions
of this software and its documentation without fee and without a signed
licensing agreement, is hereby granted, provided that the above copyright
notice, this paragraph and the following two paragraphs appear in all copies,
modifications, and distributions.  Contact The Office of
Technology Licensing, UC Berkeley, 2150 Shattuck Avenue, Suite 510, Berkeley,
CA 94720-1620, (510) 643-7201, for commercial licensing opportunities.

Written by Jeremy Wagner, The Center for New Music & Audio Technologies, 
University of California, Berkeley.  Madgwick filter implementation is drawn 
from Sebastian O.H. Madgwick (2010) 'An efficient orientation filter for 
inertial/magnetic sensor arrays.'

IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING
OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS
BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

*/


// Load Wi-Fi library
#include <WiFi.h>
#include <WiFiUDP.h>
#include <SPI.h>
#include <OSCBundle.h>
#include <OSCTiming.h>
#include <Wire.h>
//#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LIS3DH.h>
//#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>

float deltat = .025;
float prev_time, now_time;
float psi, theta, phi;  //for Euler angles, yaw, roll, pitch, respectively
//#define deltat 0.001f
#define gyroMeasError 3.14159265358979 * (5.0f / 180.0f)  //assuming gyro error of 5 degrees / sec
#define gyroMeasDrift 3.14159265358979 * (0.2f / 180.0f)  //assuming 0.2 degrees of drift per second per second
#define beta sqrt(3.0f / 4.0f) * gyroMeasError            //compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift            //compute zeta

//Adafruit_LSM6DS3TRC lsm6ds3trc;
Adafruit_LSM6DSOX lsm6dsox;

Adafruit_LIS3DH lis3dh;
//#define LIS3MDL_CLK 13
//#define LIS3MDL_MISO 12
//#define LIS3MDL_MOSI 11
//#define LIS3MDL_CS 10

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

struct quaternion {
  float q1;
  float q2;
  float q3;
  float q4;
};

struct quaternion q_est = { 1, 0, 0, 0 };  // initialize with as unit vector with real component  = 1

struct quaternion quat_mult(struct quaternion L, struct quaternion R) {


  struct quaternion product;
  product.q1 = (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3) - (L.q4 * R.q4);
  product.q2 = (L.q1 * R.q2) + (L.q2 * R.q1) + (L.q3 * R.q4) - (L.q4 * R.q3);
  product.q3 = (L.q1 * R.q3) - (L.q2 * R.q4) + (L.q3 * R.q1) + (L.q4 * R.q2);
  product.q4 = (L.q1 * R.q4) + (L.q2 * R.q3) - (L.q3 * R.q2) + (L.q4 * R.q1);

  return product;
}

struct madgwickFilter {
  float a_x, a_y, a_z;                               //accelerometer measurements
  float w_x, w_y, w_z;                               //gyro measurements
  float m_x, m_y, m_z;                               //mag measurements
  float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0;  //estimated quaternion
  float b_x = 1, b_z = 0;                            //reference directions of flux in earth frame
  float w_bx = 0, w_by = 0, w_bz = 0;
} madgwickFilter;

// Replace with your desired network credentials; this ESP32 will function as an AP
const char* ssid = "CNMAT-MARGO-4";
const char* password = "123456789";

#define UDP_TX_PACKET_MAX_SIZE 8192

char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged\r\n";        // a string to send back
char error_buffer[] = "";

IPAddress IP(192, 168, 1, 1);
IPAddress destIP(192, 168, 1, 255);

unsigned int localPort = 1750;  //port to listen to
unsigned int destPort = 1751;   //port to send to
float loopDelay = 100;

WiFiUDP Udp;

int val = 0;

//function declaration
float* hamiltonProduct(float* a, float* b);


void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  while (!Udp.begin(localPort)) {
    Serial.println("Connecting to UDP...");
  };
  Serial.print("UDP listening on port: ");
  Serial.println(localPort);

  //set up the LSM6DS3 IMU
  if (!lsm6dsox.begin_I2C(0x6B)) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS)) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("LSM6DSOX Found!");

  // lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (lsm6dsox.getAccelRange()) {
    case LSM6DS_ACCEL_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case LSM6DS_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case LSM6DS_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case LSM6DS_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }

  // lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (lsm6dsox.getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS:
      Serial.println("125 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_250_DPS:
      Serial.println("250 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
    case ISM330DHCX_GYRO_RANGE_4000_DPS:
      break;  // unsupported range for the DS33
  }

  // lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6dsox.getAccelDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");
      break;
  }

  // lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (lsm6dsox.getGyroDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");
      break;
  }

  lsm6dsox.configInt1(false, false, true);  // accelerometer DRDY on INT1
  lsm6dsox.configInt2(false, true, false);  // gyro DRDY on INT2

  //now set up the Magnetometer
  if (!lis3dh.begin(0x19)) {
    Serial.println("Failed to find LIS3DH chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3DH Found!");

  // lis3dh.setPerformanceMode() setPerformanceMode(LIS3DH_MEDIUMMODE);
  // Serial.print("Performance mode set to: ");
  // switch (lis3dh.getPerformanceMode()) {
  //   case LIS3DH_ LOWPOWERMODE: Serial.println("Low"); break;
  //   case LIS3DH_MEDIUMMODE: Serial.println("Medium"); break;
  //   case LIS3DH_HIGHMODE: Serial.println("High"); break;
  //   case LIS3DH_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  // }

  // lis3dh.setOperationMode(LIS3DH_CONTINUOUSMODE );
  // Serial.print("Operation mode set to: ");
  // // Single shot mode will complete conversion and go into power down
  // switch (lis3dh.getOperationMode()) {
  //   case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
  //   case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
  //   case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  // }

  lis3dh.setDataRate(LIS3DH_DATARATE_200_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Data rate set to: ");
  switch (lis3dh.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

  }

  lis3dh.setRange(LIS3DH_RANGE_4_G);
  Serial.print("Range set to: ");
  switch (lis3dh.getRange()) {
    case LIS3DH_RANGE_2_G : Serial.println("+-2 gauss"); break;
    case LIS3DH_RANGE_4_G : Serial.println("+-4 gauss"); break;
    case LIS3DH_RANGE_8_G : Serial.println("+-8 gauss"); break;
    case LIS3DH_RANGE_16_G : Serial.println("+-16 gauss"); break;
  }

  //lis3dh.setIntThreshold(500);
  // lis3dh.configInterrupt(false, false, true,  // enable z axis
  //                         true,                // polarity
  //                         false,               // don't latch
  //                         true);               // enabled!

  //now set some values for use with madgwick filter
  filter.begin(25);
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();

  //initializa quaternion to identity
  float quaternion[4] = { 1.0, 0, 0, 0 };

  prev_time = micros() / 1000000.0f;
}

void loop() {
  //Serial.println("DEBUG: HERE");
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    destIP = Udp.remoteIP();  //we set our destination IP to the source of the incoming msg
    OSCBundle bundleIn;
    while (packetSize--) {
      bundleIn.fill(Udp.read());
    }
    if (!bundleIn.hasError()) {
      bundleIn.dispatch("/setPort", setDestPort);
      bundleIn.dispatch("/setIP", setDestIP);
      bundleIn.dispatch("/setLoopFreq", setLoopFreq);
      bundleIn.dispatch("/calibrate", calibrate);
      //bundleIn.dispatch("/q_est_calibrate", q_est_calibrate);
      //add methods for setting delta time, calibration, etc.
    }
  }

  OSCBundle bndl;
  uint64_t timetag;
  lis3dh.read();
  bndl.add("/test").add(val++);
  //bndl.add("/test/time").add(timetag);
  bndl.add("/millis").add((int)millis());


  bndl.add("/Mag_XYZ").add(lis3dh.x).add(lis3dh.y).add(lis3dh.z);
  sensors_event_t event;
  lis3dh.getEvent(&event);
  bndl.add("/uTesla_XYZ").add(event.magnetic.x).add(event.magnetic.y).add(event.magnetic.z);

  // Get a new normalized sensor event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6dsox.getEvent(&accel, &gyro, &temp);
  //float gyroX = convertRawGyro(gyro.gyro.x);
  //float gyroY = convertRawGyro(gyro.gyro.y);
  //float gyroZ = convertRawGyro(gyro.gyro.z);
  bndl.add("/temp").add(temp.temperature);                                                              //send temp in degrees C
  bndl.add("/accel").add(-accel.acceleration.x).add(-accel.acceleration.y).add(-accel.acceleration.z);  //send accel in m/s^2
  bndl.add("/gyro").add(gyro.gyro.x).add(gyro.gyro.y).add(gyro.gyro.z);                                                   //send gyro in radians / sec

  //calculate quaternion
  unsigned long microsNow = micros();
  unsigned long micros_delta = microsNow - microsPrevious;


  //Serial.print("micros_delta: ");
  //Serial.println(micros_delta);
  //filter.begin((float)micros_delta / 1000000.0);  //this only updates the sample frequency
  // filter.updateIMU(
  //   gyroX,
  //   gyroY,
  //   gyroZ,
  //   convertRawAcceleration(-accel.acceleration.x),
  //   convertRawAcceleration(-accel.acceleration.y),
  //   convertRawAcceleration(-accel.acceleration.z)
  //   );



  // bndl.add("/Madgwick").add(filter.getPitch()).add(filter.getRoll()).add(filter.getYaw());

  //calculate angular rate quaternion by integrating gyro
  float delT = micros_delta / 1000000.0;
  // Serial.println(gyro.gyro.x);
  // quaternion omega = { 0.0, gyroX, gyroY, gyroZ };
  // quaternion qDot = quat_mult(q_est, omega);
  // for (int i = 0; i < 4; i++) {
  //   qDot.q1 *= 0.5;
  //   qDot.q2 *= 0.5;
  //   qDot.q3 *= 0.5;
  //   qDot.q4 *= 0.5;
  //   q_est.q1 += qDot.q1 * delT;
  //   q_est.q2 += qDot.q2 * delT;
  //   q_est.q3 += qDot.q3 * delT;
  //   q_est.q4 += qDot.q4 * delT;

  //   //normalize
  //   float norm = sqrt(pow(q_est.q1, 2) + pow(q_est.q2, 2) + pow(q_est.q3, 2) + pow(q_est.q4, 2));
  //   q_est.q1 /= norm;
  //   q_est.q2 /= norm;
  //   q_est.q3 /= norm;
  //   q_est.q4 /= norm;
  // }

  // bndl.add("/q_est_omega").add(q_est.q1).add(q_est.q2).add(q_est.q3).add(q_est.q4);
  
  
  //for (int i = 0; i < 100; i++) {
    now_time = micros() / 1000000.0f;
    //deltat = now_time - prev_time;
    filterUpdate(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, lis3dh.x, lis3dh.y, lis3dh.z);
    prev_time = now_time;
  //}

  bndl.add("/Madgwick").add(madgwickFilter.SEq_1).add(madgwickFilter.SEq_2).add(madgwickFilter.SEq_3).add(madgwickFilter.SEq_4);
  calculateEuler();
  bndl.add("/Euler").add(phi).add(theta).add(psi);  //pitch, roll, yaw


  Udp.beginPacket(destIP, destPort);
  bndl.setTimetag(oscTime());
  bndl.send(Udp);
  Udp.endPacket();
  bndl.empty();


  //Serial.println(loopDelay - (float)(micros()-microsPrevious)/1000.0);
  delay(loopDelay);  //TODO: subtract execution time


  // send a reply, to the IP address and port that sent us the packet we received
  //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  //Udp.write(ReplyBuffer);
  //Udp.endPacket();

  microsPrevious = microsNow;
}

void calibrate(OSCMessage& msg){
  //re-initiate quaternion and related vars
  madgwickFilter.SEq_1 = 1.0f;
  madgwickFilter.SEq_2 = madgwickFilter.SEq_3 = madgwickFilter.SEq_4 = 0.0f;
  madgwickFilter.b_x = 1.0f;
  madgwickFilter.b_z = 0.0f;
  madgwickFilter.w_bx = madgwickFilter.w_by = madgwickFilter.w_bz;
}

void setDestPort(OSCMessage& msg) {
  int a = msg.getInt(0);
  if (a > 0 && a <= 65536) {
    destPort = a;

  } else {
    OSCBundle Err;
    Err.add("/ERROR").add("ERROR SETTING DESTINATION PORT");
    Udp.beginPacket(destIP, destPort);
    Err.setTimetag(oscTime());
    Err.send(Udp);
    Udp.endPacket();
    Err.empty();
  }
}

void setLoopFreq(OSCMessage& msg) {
  float a = (float)msg.getInt(0);
  if (a>0.) {
    loopDelay = 1000.0 / a;
    deltat = 1.000f / a;
    Serial.print("Delta T set to: ");
    Serial.println(deltat);
  }
}

void setDestIP(OSCMessage& msg) {
  IPAddress dIP = IPAddress(msg.getInt(0), msg.getInt(1), msg.getInt(2), msg.getInt(3));
  if (dIP) {
    destIP = dIP;
  } else {
    OSCBundle Err;
    Err.add("/ERROR").add("ERROR SETTING DESTINATION IP");
    Udp.beginPacket(destIP, destPort);
    Err.setTimetag(oscTime());
    Err.send(Udp);
    Udp.endPacket();
    Err.empty();
  }
}

// void q_est_calibrate(OSCMessage& msg) {
//   //TODO: add a case to set quaternion
//   //if (!msg.getFloat(3)) {
//   q_est.q1 = 1.0;
//   q_est.q2 = q_est.q3 = q_est.q4 = 0.0;
//   //} else {
//   // q_est.q1 = msg.getFloat(0);
//   // q_est.q2 = msg.getFloat(1);
//   // q_est.q2 = msg.getFloat(2);
//   // q_est.q3 = msg.getFloat(3);
//   //}
// }

//add methods for setting delta time

//conversion methods for Madgwick Filter
float convertRawAcceleration(int aRaw) {
  // since we are using 2 g range
  // -2 g maps to a raw value of -32768
  // +2 g maps to a raw value of 32767

  float a = (aRaw * lsm6dsox.getAccelRange()) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  //find range setting

  float a;
  switch (lsm6dsox.getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS:
      a = 125.0;
      break;
    case LSM6DS_GYRO_RANGE_250_DPS:
      a = 250.0;
      break;
    case LSM6DS_GYRO_RANGE_500_DPS:
      a = 500;
      break;
    case LSM6DS_GYRO_RANGE_1000_DPS:
      a = 1000;
      break;
    case LSM6DS_GYRO_RANGE_2000_DPS:
      a = 2000;
      break;
    case ISM330DHCX_GYRO_RANGE_4000_DPS:
      break;  // unsupported range for the DS33
  }
  a*= 0.017453292519943;  

  float g =  ((float)gRaw * a) / 32768.0f;  //convert to radians
  return g;
}

//MARG filter from Seb Madgwick's 2010 paper 'An efficient orientation filter for inertial and inertial/magnetic sensor arrays'
void filterUpdate(float wx, float wy, float wz, float ax, float ay, float az, float mx, float my, float mz) {
  //local system vars
  float norm;                                                                                                                        //vector norm
  float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;                                                              //quaternion rate from gyro elements
  float f_1, f_2, f_3, f_4, f_5, f_6;                                                                                                //objective function elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64;  //objective function Jacobian elements
  float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;                                                                          //estimated direction of the gyro error
  float w_err_x, w_err_y, w_err_z;                                                                                                   //estimated direction of the gyro error (angular)
  float h_x, h_y, h_z;                                                                                                               //computed flux in the earth frame

  //auxilliary vars
  float halfSEq_1 = 0.5 * madgwickFilter.SEq_1;
  float halfSEq_2 = 0.5 * madgwickFilter.SEq_2;
  float halfSEq_3 = 0.5 * madgwickFilter.SEq_3;
  float halfSEq_4 = 0.5 * madgwickFilter.SEq_4;
  float twoSEq_1 = 2.0f * madgwickFilter.SEq_1;
  float twoSEq_2 = 2.0f * madgwickFilter.SEq_2;
  float twoSEq_3 = 2.0f * madgwickFilter.SEq_3;
  float twoSEq_4 = 2.0f * madgwickFilter.SEq_4;
  float twob_x = 2.0f * madgwickFilter.b_x;
  float twob_z = 2.0f * madgwickFilter.b_z;
  float twob_xSEq_1 = 2.0f * madgwickFilter.b_x * madgwickFilter.SEq_1;
  float twob_xSEq_2 = 2.0f * madgwickFilter.b_x * madgwickFilter.SEq_2;
  float twob_xSEq_3 = 2.0f * madgwickFilter.b_x * madgwickFilter.SEq_3;
  float twob_xSEq_4 = 2.0f * madgwickFilter.b_x * madgwickFilter.SEq_4;
  float twob_zSEq_1 = 2.0f * madgwickFilter.b_z * madgwickFilter.SEq_1;
  float twob_zSEq_2 = 2.0f * madgwickFilter.b_z * madgwickFilter.SEq_2;
  float twob_zSEq_3 = 2.0f * madgwickFilter.b_z * madgwickFilter.SEq_3;
  float twob_zSEq_4 = 2.0f * madgwickFilter.b_z * madgwickFilter.SEq_4;
  float SEq_1SEq_2;
  float SEq_1SEq_3 = madgwickFilter.SEq_1 * madgwickFilter.SEq_3;
  float SEq_1SEq_4;
  float SEq_2SEq_3;
  float SEq_2SEq_4 = madgwickFilter.SEq_2 * madgwickFilter.SEq_4;
  float SEq_3SEq_4;
  float twom_x = 2.0f * madgwickFilter.m_x;
  float twom_y = 2.0f * madgwickFilter.m_y;
  float twom_z = 2.0f * madgwickFilter.m_z;

  //normalize accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  ax /= norm;
  ay /= norm;
  az /= norm;

  //normalize magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  mx /= norm;
  my /= norm;
  mz /= norm;

  //compute the objective function and Jacobian

  f_1 = twoSEq_2 * madgwickFilter.SEq_4 - twoSEq_1 * madgwickFilter.SEq_3 - ax;
  f_2 = twoSEq_1 * madgwickFilter.SEq_2 + twoSEq_3 * madgwickFilter.SEq_4 - ay;
  f_3 = 1.0f - twoSEq_2 * madgwickFilter.SEq_2 - twoSEq_3 * madgwickFilter.SEq_3 - az;  //double check this one
  f_4 = twob_x * (0.5f - madgwickFilter.SEq_3 * madgwickFilter.SEq_3 - madgwickFilter.SEq_4 * madgwickFilter.SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - mx;
  f_5 = twob_x * (madgwickFilter.SEq_2 * madgwickFilter.SEq_3 - madgwickFilter.SEq_1 * madgwickFilter.SEq_4) + twob_z * (madgwickFilter.SEq_1 * madgwickFilter.SEq_2 + madgwickFilter.SEq_3 * madgwickFilter.SEq_4) - my;
  f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - madgwickFilter.SEq_2 * madgwickFilter.SEq_2 - madgwickFilter.SEq_3 * madgwickFilter.SEq_3) - mz;

  J_11or24 = twoSEq_3;
  J_12or23 = 2.0f * madgwickFilter.SEq_4;
  J_13or22 = twoSEq_1;
  J_14or21 = twoSEq_2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;
  J_41 = twob_zSEq_3;
  J_42 = twob_zSEq_4;
  J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1;
  J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;
  J_51 = twob_xSEq_4 - twob_zSEq_2;
  J_52 = twob_xSEq_3 + twob_zSEq_1;
  J_53 = twob_xSEq_2 + twob_zSEq_4;
  J_54 = twob_xSEq_1 - twob_zSEq_3;
  J_61 = twob_xSEq_3;
  J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
  J_63 = twob_xSEq_1 - 2.0f * twob_xSEq_3;
  J_64 = twob_xSEq_2;

  //compute the gradient (matrix multiplication)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
  SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
  SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

  //normalize the gradient to estimate direction of the gyroscope error
  norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
  SEqHatDot_1 /= norm;
  SEqHatDot_2 /= norm;
  SEqHatDot_3 /= norm;
  SEqHatDot_4 /= norm;

  //compute angular estimated direction of the gyroscope error
  w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
  w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
  w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;

  //compute and remove the gyro biases
  madgwickFilter.w_bx += w_err_x * deltat * zeta;
  madgwickFilter.w_by += w_err_y * deltat * zeta;
  madgwickFilter.w_bz += w_err_z * deltat * zeta;
  wx -= madgwickFilter.w_bx;
  wy -= madgwickFilter.w_by;
  wz -= madgwickFilter.w_bz;

  //compute the quaternion rate measured by the gyro
  SEqDot_omega_1 = -halfSEq_2 * wx - halfSEq_3 * wy - halfSEq_4 * wz;
  SEqDot_omega_2 = halfSEq_1 * wx + halfSEq_3 * wz - halfSEq_4 * wy;
  SEqDot_omega_3 = halfSEq_1 * wy - halfSEq_2 * wz + halfSEq_4 * wx;
  SEqDot_omega_4 = halfSEq_1 * wz + halfSEq_2 * wy - halfSEq_3 * wx;

  //compute then integrate the estimated quaternion rate
  madgwickFilter.SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
  madgwickFilter.SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
  madgwickFilter.SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
  madgwickFilter.SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

  //normalize quaternion
  norm = sqrt(madgwickFilter.SEq_1 * madgwickFilter.SEq_1 + madgwickFilter.SEq_2 * madgwickFilter.SEq_2 + madgwickFilter.SEq_3 * madgwickFilter.SEq_3 + madgwickFilter.SEq_4 * madgwickFilter.SEq_4);
  madgwickFilter.SEq_1 /= norm;
  madgwickFilter.SEq_2 /= norm;
  madgwickFilter.SEq_3 /= norm;
  madgwickFilter.SEq_4 /= norm;

  //compute flux in earth frame
  SEq_1SEq_2 = madgwickFilter.SEq_1 * madgwickFilter.SEq_2;
  SEq_1SEq_3 = madgwickFilter.SEq_1 * madgwickFilter.SEq_3;
  SEq_1SEq_4 = madgwickFilter.SEq_1 * madgwickFilter.SEq_4;
  SEq_3SEq_4 = madgwickFilter.SEq_3 * madgwickFilter.SEq_4;
  SEq_2SEq_3 = madgwickFilter.SEq_2 * madgwickFilter.SEq_3;
  SEq_2SEq_4 = madgwickFilter.SEq_2 * madgwickFilter.SEq_4;

  h_x = twom_x * (0.5f - madgwickFilter.SEq_3 * madgwickFilter.SEq_3 - madgwickFilter.SEq_4 * madgwickFilter.SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
  h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - madgwickFilter.SEq_2 * madgwickFilter.SEq_2 - madgwickFilter.SEq_4 * madgwickFilter.SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
  h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 - SEq_1SEq_2) + twom_z * (0.5f - madgwickFilter.SEq_2 * madgwickFilter.SEq_2 - madgwickFilter.SEq_3 * madgwickFilter.SEq_3);

  //normalize the flux vector to have only components in x and z
  madgwickFilter.b_x = sqrt((h_x * h_x) + (h_y * h_y));
  madgwickFilter.b_z = h_z;
}

void calculateEuler(){
  //calculate yaw
  psi = atan2(2*(madgwickFilter.SEq_2 * madgwickFilter.SEq_3 - madgwickFilter.SEq_1 * madgwickFilter.SEq_4),
  2*(madgwickFilter.SEq_1 * madgwickFilter.SEq_1 + madgwickFilter.SEq_2 * madgwickFilter.SEq_2)-1);

  //roll
  theta = -asin(2*(madgwickFilter.SEq_2 * madgwickFilter.SEq_4 + madgwickFilter.SEq_1 * madgwickFilter.SEq_3));

  //pitch
  phi = atan2(2*(madgwickFilter.SEq_3 * madgwickFilter.SEq_4 - madgwickFilter.SEq_1 * madgwickFilter.SEq_2),
  2*(madgwickFilter.SEq_1 * madgwickFilter.SEq_1 + madgwickFilter.SEq_4 * madgwickFilter.SEq_4)-1); 

  
}