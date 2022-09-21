# Documentation

## High-level software

ToDo

## Hardware and low-level software

ToDo

### RasPi <-> Teensy protocol

Communication via SPI (mode 0).

The Teensy has to acknowledge the packet by returning the command ID. If the Teensy has to wait for data (e.g. orientation data), it sends 0xFF to indicate that it is waiting and the Pi has to resend the packet after a small delay.
Since the Teensy can only answer while the Pi is sending, the Pi has to transmit empty bytes. Packets look as follows:

| Num bytes | Content |
| - | - |
| 1 | Command ID |
| 0-4 | Data |
| 1 | Acknowledgement |
| 0-4 | Answer data |

| Command | ID | Data | Data type | Answer data | Answer data type | Description |
| - | - | - | - | - | - | - |
| Acknowledge | 0x01 | - | - | - | - | General purpose acknowledgement packet (e.g. to check if a task is finished) |
| Set Speed | 0x02 | Left and right speed | int8 [2] | - | - | Set front left and right speed. Teensy handles back wheel speeds |
| Turn | 0x03 | Angle in radians | float32 | - | - | Turn by a specified amount |
| Turn to | 0x04 | Heading in radians | float32 | - | - | Turn to a (previously saved) heading |
| Stop | 0x05 | Duty cycle (Brake) | uint8 | - | - | Stop the drive motors. Optional braking (shorting of motors) |
| Servo | 0x06 | Servo ID and angle in degrees | uint8 [2] | - | - | Move servo to specified position |
| Get distance | 0x07 | Sensor ID | uint8 | Distance in millimeters | uint16 | Get value of a distance sensor |
| Get orientation | 0x08 | Axis number (yaw/heading, pitch, roll) | uint8 | Angle in radians | float32 | Get euler angles from orientation sensor |

Range of values is from the lowest to the highest possible number, if no units are specified.