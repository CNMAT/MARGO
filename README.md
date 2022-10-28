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

# Getting Started
0. Obtain an ESP32 and a LSM6DS3 + LIS3MDL magnetic, angular rate and gravity (MARG) sensor.  You will also need:
    - The Arduino IDE
    - Max (available from Cycling74.com)
    - CNMAT's ODOT package (in Max, go to File > Show Package Manager > search odot, click on "odot" then install. Restart Max.)
1. Download this repo to your computer.  We recommend using GitHub desktop if you are uncomfortable with git.
    - Be sure to occasionally fetch changes to keep up with latest updates.
2. Connect the LSM6DS3 + LIS3MDL sensor to the ESP32:
    - Connect 3V3 pin on sensor to 3V3 pin on ESP32
    - Connect GND pin on sensor to GND pin on ESP32
    - Connect CLK pin on sensor to CLK pin (pin 22) on ESP32
    - Connect SDA pin on sensor to SDA pin (pin 21) on ESP32
3. Open the Arduino IDE
    - Be sure to install your ESP variant in the Tools>Board>Boards Manager (we used the ESP32-Devkitc-V4, which works with the ESP32-WROOM-DA Module profile)
4. Open the MARGO.ino file.
    - Change the ssid in the code to a name you wish to use. This will becoome the name of the wireless network set up by the MARGO. 
    - Make any desired customizations.
    - Connect ESP32 to computer with micro-USB cable.
    - Assign the correct port in Tools>Port> ... (the correct port will look something like "/dev/cu.usbserial-xxxx")
    - Upload the sketch to the ESP32 (click upload arrow in Arduino IDE window (or shift-command-R on Mac)
    - (ESP32 code takes a minute to compile and upload, be patient).
    - Switch to Serial Monitor in the Arduino IDE; Reboot the ESP32 by pressing either of its buttons.  If everything is working, you should see something like the following in the Serial Monitor:
    ```
    Setting AP (Access Point)…AP IP address: 192.168.4.1
    UDP listening on port: 1750
    LSM6DS3TR-C Found!
    Accelerometer range set to: +-4G
    Gyro range set to: 2000 degrees/s
    Accelerometer data rate set to: 104 Hz
    Gyro data rate set to: 104 Hz
    LIS3MDL Found!
    Performance mode set to: Medium
    Operation mode set to: Continuous
    Data rate set to: 155 Hz
    Range set to: +-4 gauss
    ```
4. Once the ESP32 is powered and running, it should appear as a wireless network (We named ours CNMAT-MARGO-1). Your credentials are those defined in the MARGO.ino as password (we used '123456789').
    
5. Open the 00_MARGO_START_HERE.maxpat
  - By default, the MARGO sketch will begin sending OSC packets to the IP address of the last message it recieved, so we just need to send it a packet--any packet--to get it sending sensor values to our Max patch.
