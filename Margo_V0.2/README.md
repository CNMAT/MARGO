The files in this directory were created in Autodesk Fusion 360. They describe a daughter board for an ESP-32-Devkit-C that securely and reliably mount peripherals for the MARGO implementation.  This prototype, tentatively called MARGO 0.2, is an attempt to miniaturize the footprint of the MARGO into a solid-stated device of 1" x 2" in size.

Margo v0.2 Board
![IMG_2468](https://github.com/CNMAT/MARGO/assets/878883/89cb73f4-ddd5-4420-b3a9-117407046a17)

Mounted to ESP32-DevKit-C, charging cable attached
![IMG_2471](https://github.com/CNMAT/MARGO/assets/878883/3bf1bb87-907d-47a7-859b-c1acc5f7ea9c)
![IMG_2469](https://github.com/CNMAT/MARGO/assets/878883/675c88e3-9316-428b-a60a-67e891a81e97)



# Features
* LSM6DSOTRLGA and LIS3DHTRL sensors for Magnetometer, Acceleration, Rotation & Gyroscope Sensors. These have been upgraded due to supply chain concerns but do require slightly different addressing to run.
* incorporates a power switch and on-board battery charger.

# TO DO
* Include a voltage divider to monitor battery charge via ADC pins.
* Consolidate onto single board with ESP32-WROOM-32U
* Relocate battery connector
* Consider flipping board for back-to-back mounting. 
