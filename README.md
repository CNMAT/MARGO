# CNMAT MARGO: An ESP32-based gesture control platform with OSC

This is an example project that implements an affordable gesture control platform using an ESP32 and Inertial Measurement Unit (IMU) with Magnetometer connected over I2C.  The examples in this project illustrate ways to set up wireless networking functionality on the ESP32, how to access sensor data on connected I2C devices, how to process that data and transmit it to a network client as an OSC bundle.  The MARGO.ino file allows the end user to set up the ESP32 as a named wireless access point.  After connecting to this AP and transmitting any message to it via broadcast UDP, the MARGO will automatically begin sending its data stream to the IP address from which the request was initiated.

The examples in this repo were developed with the LSM6DS3 + LIS3MDL magnetic, angular rate and gravity (MARG) sensor.  Popular versions of this sensor incorporate a thermometer, thereby offering 10 degrees of freedom.  Sensor integration is accomplished via an implementation of Sebastian Madgwick's gradient descent orientation filter, which yeilds a stable quaternion orientation from which is derived the orientation in Euler angles.  Raw values of the sensor data, as well as the quaternion and Euler angles are transmitted as the following OSC bundle:

- /millis : {number of milliseconds since ESP32 boot}
- /Mag_XYZ : {magnetic vector in the sensor frame X,Y,Z (Gauss)} 
- /uTesla : {magnetic vector in sensor frame X,Y,Z (micro-Teslas)}
- /accel : {acceleration in sensor frame X,Y,Z (m/s^2)}
- /gyro : {sensor's angular rate of rotation around sensor's X, Y, Z (radians / second)
- /temp : {sensor's measured temperature (degrees C)
- /Madgwick : {unit quaternion describing rotation from sensor frame to Earth frame [q_1, q_2, q_3, q_4]}
- /Euler : {conversion of Madgwick quaternion to Euler angle representation (radians) [pitch, roll, yaw]

(ESP32 will print useful debugging data in the Arduino Serial Monitor console when connected to Arduino IDE at boot; see serial monitor for conversion factors of Magnetometer, accelerometer & gyro; )

The MARGO also accepts OSC messages for configuration on the fly:

- /setPort : {int port}  ---  sets destination port
- /setIP : {int[4] aaa, bbb, ccc, ddd}   --- sets destination IP
- /setLoopFreq : {int freq}   --- sets the refresh rate (how often the sensor is read) (Hertz)
- /calibrate : {null}  --- reset the orientation quaternion to [1,0,0,0] and re-initialize related parameters
