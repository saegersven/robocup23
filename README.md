# Robocup Junior Rescue Line 2023
### Team BitFlip


* [Youtube Channel](https://www.youtube.com/channel/UCC9BH-tkFcYVH9Up8JBV4LQ)
* [Website](http://kraemer123.de/)
* [2022 Repository](https://github.com/saegersven/robocup)

### <u>Our ToDo List:</u>

#### Software

- [ ] rescue kit: sometimes arm won't move up/down (maybe send servo CMD twice?)
- [ ] use camera to check when unloading victim

#### CAD

- [ ] redesign gripper with DC motor

#### 3D Printing

### Current Problems based on runs in St Augustin (descending order of importance):
- [ ] USB/I2C instead of SPI communication?! Maybe all three for increased redundancy

- [ ] missed black line during obstacle
- [ ] VL53L0X instead of HCSR04
- [ ] Gyro sensor (turning, ramps up and down, adjust speed accordingly)
- [ ] corner approach: check contour to avoid false positives
- [ ] intersection: turn 60 degs with gyro and rest with camera until line angle = 0
- [ ] intersection: confused right with left turn (vid 4, 1:45)
- [ ] obstacle: line after obstacle pls fix
- [ ] prefer living victims when when !ignore_dead 
- [ ] oscillating during linefollowing (even when following straight lines)
- [ ] random thread(?) stop during rescue. Maybe add pull down resistor or check for press  duration
- [ ] more labeled victim data to avoid losing victim during search
- [ ] more ml data with multiple victim overlapping (and far away)
- [ ] beginning of victim search: why frame lost?
- [ ] ignore upper frame area when looking for victims
- [ ] find corner more precise (more ml data of corners far away)
- [ ] readjust robot position relative to corner when driving towards it- [ ] investigate: vid 4, 3:05 random thread crash
- [ ] turn line angle before obstacle
- [ ] check if victim has been rescued successfully (look at black corner)
- [ ] rescue kit: close gripper while driving forward 
- [ ] rewrite find_center using VL53L0X and Gyro