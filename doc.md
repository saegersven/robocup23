# Software

Our robot has two programs: The main program runs on a Raspberry Pi 4B+, and is written in C++ with OpenCV and tensorflow as the main dependencies. The secondary program runs on a Teensy 3.6, that acts as a motor controller and receives commands from the main program.

The main program follows the line, navigates intersections, detects obstacles, picks up the rescue kit and rescues the victims using mostly a single Raspberry Pi Camera v2.1 mounted high up at the front of the robot. It is mounted on a servo so it can be rotated upwards to look for victims. During line following, it looks almost straight down.

The main program is written in C++ instead of Python because we perform custom operations on individual pixels, which can't be easily done in Python without significantly sacrificing performance.

## Line following

The main library we use for our Computer Vision tasks is OpenCV. We grab frames from the camera using the library's `cv::VideoCapture` class. We get a `cv::Mat` object which contains a single image. The camera captures in a very low resolution of 80x48 pixels, which combined with optimized algorithms allows the robot to adjust its motor speeds about 100 times per second.

During line following, a frame might look like this:

![](/docs/linefollowing_example_frame.png)

Now we use a simple binary thresholding operation to find every black pixel. We now iterate over each of the black pixels and calculate the angle from the bottom center of the image to the pixel:

![](/docs/linefollowing_example_frame_pixel.png)

This angle is weighed according to its distance from the bottom center since we don't want pixels near the bottom center to contribute as much to the line angle. Pixels very far away also have lower weights. The pixels are also weighed according to how close their angle is to the angle of the last line. This prevents the robot from following the wrong lines and also helps it navigate T-shaped intersections. The weights are calculated when starting the program and saved in maps for quick access. We now use a weighted average of all the angles of the black pixels to obtain the line angle, which might look like this:

![](/docs/linefollowing_example_frame_angle.png)

This angle is then multiplied by a sensitivity factor and simply added or subtracted from the motor speeds. There is no need for integration or differentiation. This algorithm, combined with a center of rotation of the robot around the front axis, allows the robot to follow even complicated lines very quickly.

### Intersections

During line following, the frame is continuously sampled for green pixels by calculating the ratio of the green component of a pixel to the sum of the blue and red. If there are enough green pixels in the frame, we find contours using a simple recursive algorithm. Contours which are too small are discarded. The only contours that remain are green points.

Now we cut out a square part of the image around the green points and use the binary image from before to iterate over all the black pixels in that cutout. We calculate the average of all the black pixels (depicted below in blue). Now the quadrant of the average black pixel tells us where the robot needs to go. This has proven to be a very simple and robust algorithm.

![](/docs/green_example.png)

### Rescue kit

Similarly to the intersection algorithm, we continuously look for blue pixels using the same metric as for green. When we find a contour that is big enough, we calculate the angle from the bottom center to that contour, turn in that direction, pick the rescue kit up and return to following the line.

### Detecting the entrance to the evacuation zone

Detecting a reflecting strip of tape is surprisingly difficult using just a camera. We have tried different algorithms in the past, such as comparing the image to a pre-saved image of the tape, and detecting the reflection of red LEDs mounted near the camera. We now use a small machine learning model trained in Keras/Tensorflow on a few thousand images, which runs simultaneously in a separate thread.

## Evacuation zone

### Strategy

TODO

### Detecting victims

Similarly to the reflective tape at the entrance, two of the victims are reflective as well. This fact as well as debris scattered around the evacuation zone make it difficult to use traditional circle-detection algorithms such as Hough transforms to detect victims. At least we never got it to work reliably. Instead, we are using a machine learning model trained on thousands of images. We have labelled them in Microsoft VoTT. The bounding boxes are assigned one of two tags: "living" or "dead". The neural network is similar to the popular "Yolo" architecture for object detection, but it is greatly simplified.

The image is divided up chunks. For each chunk we want to know if there is a living or a dead victim, or none. This means that the neural net has to essentially output an image with two channels, containing probabilities. Below you can see an input image with 5x4 chunks and two numbers in each chunk, which are probabilities for our two classes "living" and "dead".

![](/docs/victim_ml_target_example.png)

5x4 chunks is of course not fine enough to pick up victims accurately. We use 32x20 chunks in our model. When visualizing the target images, dead victims appear blue, and living victims green.

The model is created in Python using Keras' `Sequential` class as follows:

```python
model = Sequential([
    layers.Rescaling(1./255, input_shape=(input_height, input_width, 1)),
    layers.Conv2D(8, 5, padding='same', activation='relu'),
    layers.MaxPooling2D(8),
    layers.Conv2D(16, 3, padding='same', activation='relu'),
    layers.Dropout(0.1),
    layers.Flatten(),
    layers.Dense(Ys * Xs * Cs, activation='linear'),
    layers.Reshape((Ys, Xs, Cs))
])
```

Below you can see two training images and their respective targets (left) and two test images on unknown data with output (right).

![](/docs/victim_ml_example.png)

Now we just need to extract the victim positions from the output using simple thresholding operations and contour finding. The neural net runs at about 5 fps on the Raspberry Pi and detects victims accurately with almost no false-positives.

![](/docs/victim_ml_output_example.png)

# Hardware and low-level software

## Rear motor speeds

The speed of the rear wheels is given by:

![equation](/docs/eq_rear_wheel_speeds.jpg)

Where k is a factor less than one which reduces power to the rear wheels to account for inaccuracies.

## RasPi <-> Teensy protocol

Communication is via I2C. The Teensy is the secondary device (Address 0x2a) and acts as a motor controller.

| Command | ID | Data | Data type[s] | Description |
| - | - | - | - | - |
| Motor | 11 | Motor speeds | int8[2] | Sets front motor speeds. Rear motor speeds are set automatically |
| Stop | 21 | - | - | Stops all four drive motors |
| Servo | 31 | Servo ID and angle | uint8[2] | Move servo to angle |
| Ready | 41 | - | - | Notify Teensy that the main program is ready to be started |
| Home Servos | 51 | - | - | Set Servos to home positions when resetting the robot |
| Arm Down | 61 | - | - | Move arm down |
| Arm up | 71 | - | - | Move arm up |
| Unload | 91 | - | - | Unload victims and rescue kit |
| Pick up | 101 | - | - | Pick victim up |

Command IDs are non-sequential because communication is very noisy for some reason.