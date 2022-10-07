# Documentation

## High-level software

ToDo

## Hardware and low-level software

### Rear motor speeds

The speed of the back wheels is given by:

![equation](http://www.sciweavers.org/tex2img.php?eq=v_%7B3%2F4%7D%20%3D%20k%20%5Cfrac%7BD_f%7D%7BD_b%7D%20%5Cleft%5B%5Cfrac%7Bv_1%20%2B%20v_2%7D%7B2%7D%20%5Cpm%20%28v_2%20-%20v_1%29%5Cright%5D%2C&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0)

where v1 and v2 are the front right and left motor speeds, v3 and v4 are the back left and right motor speeds and Df and Db are the circumferences of the front and back wheels. Since the back wheels do not provide the main drive for the robot, the PWM-signal can be turned down by another factor k = 80%.

### RasPi <-> Teensy protocol

Communication via SPI (mode 0).

The Teensy has to acknowledge the packet by returning the command ID. If the Teensy has to wait for data (e.g. orientation data), it sends 0xFF to indicate that it is waiting and the Pi has to resend the packet after a small delay.
Since the Teensy can only answer while the Pi is sending, the Pi has to transmit empty bytes, which it fills with zeros. Packets look as follows:

| Num bytes | Content |
| - | - |
| 1 | Command (Packet ID) |
| 0-4 | Data |
| 0-4 | Answer data |
| 1 | Acknowledgement |

| Command | ID | Data | Data type | Answer data | Answer data type | Description |
| - | - | - | - | - | - | - |
| Confirm | 0x01 | - | - | Value | uint8_t | Confirmation packet |
| Set Speed | 0x02 | Left and right speed | int8 [2] | - | - | Set front left and right speed. Teensy handles back wheel speeds |
| Turn | 0x03 | Angle in radians | float32 | - | - | Turn by a specified amount |
| Turn to | 0x04 | Heading in radians | float32 | - | - | Turn to a (previously saved) heading |
| Stop | 0x05 | Duty cycle (Brake) | uint8 | - | - | Stop the drive motors. Optional braking (shorting of motors) |
| Servo | 0x06 | Servo ID and angle in degrees | uint8 [2] | - | - | Move servo to specified position |
| Get distance | 0x07 | Sensor ID | uint8 | Distance in millimeters | uint16 | Get value of a distance sensor |
| Get orientation | 0x08 | Axis number (yaw/heading, pitch, roll) | uint8 | Angle in radians | float32 | Get euler angles from orientation sensor |

**Confirmation Packet:** The Pi uses the confirmation packet to ask if a task has finished. If a task (e.g. turning) is finished, the Teensy will send 0xFF as the value. In order for the Teensy to know that the packet has been received, the Pi sends another confirmation packet with a positive value, instead of the usual zero, as an acknowledgement.

Range of values is from the lowest to the highest possible number, if no units are specified.