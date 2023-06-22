# Robocup Junior Rescue Line 2023
### Team BitFlip


* [Youtube Channel](https://www.youtube.com/channel/UCC9BH-tkFcYVH9Up8JBV4LQ)
* [Website](http://kraemer123.de/)
* [2022 Repository](https://github.com/saegersven/robocup)

### <u>Our ToDo List:</u>

#### Software
- [ ] robot->turn() CMD_TURN_DONE not properly received (maybe endian problem? Convert to litte endian to try out)
- [ ] review changes from kraemerlukas314 in corner_ml.cpp (marked with @saegersven)
- [ ] robot->button() not working (somehow always true) (maybe endian problem? Convert to litte endian to try out)
#### CAD
- [ ] modify gripper for usage of threaded inserts (Gewindeeinsätze)
...
#### 3D Printing
- [ ] print ALL replacement parts
...
### Problems based on runs in St Augustin (descending order of importance, will probably never be implemented)
- [ ] increase motor speed when frame is the same of long time (indicating the robot got stuck)
- [ ] turn lineangle before gap? Maybe drive backwards a few times to allow for slight corrections
- [ ] missed black line during obstacle
- [ ] corner approach: check contour to avoid false positives
- [ ] intersection: confused right with left turn (vid 4, 1:45)
- [ ] obstacle: line after obstacle pls fix
- [ ] prefer living victims when when !ignore_dead 
- [ ] find exit
- [ ] random thread(?) stop during rescue. Maybe add pull down resistor or check for press  duration
- [ ] more labeled victim data to avoid losing victim during search
- [ ] more ml data with multiple victim overlapping (and far away)
- [ ] beginning of victim search: why frame lost?
- [ ] ignore upper frame area when looking for victims
- [ ] find corner more precise (more ml data of corners far away)
- [ ] reajust robot position relative to corner when driving towards it
- [ ] investigate: vid 4, 3:05 random thread crash
- [ ] turn line angle before obstacle
- [ ] check if victim has been rescued successfully (look at black corner)
- [ ] rescue kit: close gripper while driving forward 
- [X] rewrite find_center using VL53L0X and Gyro
- [X] VL53L0X instead of HCSR04
- [X] Gyro sensor (turning, ramps up and down, adjust speed accordingly)
- [X] intersection: turn 60 degs with gyro and rest with camera until line angle = 0
- [X] oscillating during linefollowing (even when following straight lines)
- [X] drive towards rescue kit so its always in the same y position (not only x) to facilitate pickup