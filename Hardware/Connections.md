# Connecting everything

There are lots of ways of connecting up the individual bits of hardware, but here is how we did it.  For the prototype, we soldered pin-headers to all of the boards and used F/F jumper wires.  The end user may wish to solder these connections on a prototyping board:

| Hardware | Pin | Connect to | Hardware | Pin |
| -------------------------------------------- |
| LSM6DS3  | 3Vo | ---------> | ESP32    | 3v3 |
| LSM6DS3  | GND | ---------> | ESP32    | GND |
| LSM6DS3  | SCL | ---------> | ESP32    | SCL (Pin 22) |
| LSM6DS3  | SDA | ---------> | ESP32    | SDA (Pin 21) |
| Micro-Lipo | BAT | ---------> | ESP32    | 5V |
| Micro-Lipo | GND | ---------> | ESP32    | GND |

(Plug battery into lipo port on Micro-Lipo)
