# Documentation

## High-level software

ToDo

## Hardware and low-level software

### Rear motor speeds

The speed of the back wheels is given by:

![equation](/docs/eq_rear_wheel_speeds.jpg)

where v1 and v2 are the front right and left motor speeds, v3 and v4 are the rear left and right motor speeds and Df and Dr are the circumferences of the front and rear wheels. Since the rear wheels do not provide the main drive for the robot, the PWM-signal can be turned down by a factor k = 80%.

### RasPi <-> Teensy protocol

Communication is via I2C. The Teensy is the secondary device (Address 0x2a) and acts as a motor controller.

| Command | ID | Data | Data type[s] | Description |
| - | - | - | - | - |
| Set Status | 0x01 | Status value | uint8 | General-purpose status packet |
| Set Speed | 0x02 | Left and right speeds | int8 [2] | Set front left and right speed. Teensy handles rear wheel speeds |
| Stop | 0x03 | - | - | Stop the drive motors |
| Servo | 0x04 | Servo ID and angle in degrees | uint8 [2] | Move servo |