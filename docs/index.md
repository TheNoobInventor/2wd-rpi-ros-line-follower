# 2 Wheeled Drive Raspberry Pi Line Follower running ROS1 Noetic
A 2-wheel drive car, controlled by a Raspberry Pi 4 running Robot Operating System (ROS) 1 Noetic, is used as a line following robot; it follows a black line against a white background, with the aid of a 5 channel infrared sensor.

## Hardware

### Components
The components used in this project are as follows:

- 2 wheeled drive robot chassis
- Raspberry Pi 4
- Motor driver HAT (Waveshare)
- 2 x standoff spacers
- 5-channel infrared sensor (Waveshare)
- 9V battery
- 2 x alligator clip cables 
- Powerbank rated at least 5V and 3A with USB type-A to USB-type C cable to power RPi
- Wires - to connect motors to motor driver HAT
- Track(s) for robot to follow
- Screwdriver set
- Double-sided tape
- Duck tape

The 5-channel infrared sensor from Waveshare are shown in the figures below:
<p float="center">
  <img src=images/ir_front.jpg width="400">
  <img src=images/ir_back.jpg width="400">
</p>

### Assembly

The 2 wheeled robot chassis is assembled using the guide shown [here](https://www.ardu.dk/wp-content/uploads/2019/11/ARDU-2WD-robot-car-instructions.pdf). The motor driver HAT is firmly attached to the GPIO pins on the Raspberry Pi with standoff spacers for added stability. The 5 channel infrared sensor is mounted underneath the robot chassis and connected to the RPi GPIO pins via the motor driver HAT in this order:

| Infrared sensor | Cable color | GPIO.BOARD | GPIO.BCM |
| ------ | ------ | ------ | ------ |
| IR1 | Blue | 19 | GPIO16 |
| IR2 | Yellow | 21 | GPIO17 |
| IR3 | Orange | 23 | GPIO18 |
| IR4 | Green | 22 | GPIO13 |
| IR5 | White | 24 | GPIO19 |

The red and black cables are connected to a 5V and ground (GND) GPIO pins respectively.

The RPi was placed on a 3D printed platform using double-sided tape, which was mounted to the chassis also with double-sided tape. 

One end of the cables from the motors were soldered with the other end connected to the motor driver HAT. The motor driver HAT is powered using the 9V battery via the alligator cables. 

Finally, the raspberry pi is powered by the power bank, via a USB type-A to USB type-C cable, which is mounted on the chassis with double-sided tape.

The fully assembled robot is shown in the following images:

<p float="center">
  <img src=images/side.jpg width="400">
  <img src=images/top.jpg width="400">
  <img src=images/bottom.jpg width="400">
</p>

## Software

### Software architecture
**Raspberry Pi OS** is the operating system used on the Raspberry Pi 4. The download and installation procedure can be found [here](https://www.raspberrypi.org/software/). ROS 1 Noetic is the version of ROS used in this project and can be installed following this [guide](https://www.hackster.io/shahizat005/lidar-integration-with-ros-noetic-on-raspberry-pi-os-8ea140).

### Package install and setup

Once Raspberry Pi OS and ROS Noetic have been successfully setup, the next step is to download and setup the ROS package in this repository. 

Open up a terminal window and navigate to the source folder in your ROS workspace (```ros_catkin_ws/src``` in my case) and clone this repository by executing the following command in the terminal:
```
git clone https://github.com/TheNoobInventor/2wd-rpi-ros-line-follower.git
```
The motor driver HAT uses Inter-Integrated Circuit (I2C) serial interface to communicate with the Raspberry Pi. This interface needs to be enabled in Raspberry Pi OS, as it is disabled by default. To do this, click on the ***RPi Home*** button, navigate to ***Preferences***, then ***Raspberry Pi Configuration*** to ***I2C*** and finally click on the ***Enable*** radio button. Then reboot the RPi for this change to be effected.

#### Build package
---
To build the package, move up one level to the main ROS workspace (```ros_catkin_ws```) and run the following command in the terminal:

```
catkin_make
```
#### Custom message
---
A custom message - ```IrSensor.msg``` in the ```msg``` folder -  of type int32, was created for the different channels of the infrared sensor to for the infrared sensor data

#### Run package nodes
---
Explin rospkg (not everyone might need it), smbus (what for?)
```
pip3 install rospkg && pip3 install smbus
```
Talk about a bit about the directory structure...where the files can be found...which one is the main what the other files do etc

written in **Python 3**

chmod the executables
```
chmod +x ir_sensor_node.py rpi_car_main_node.py PCA9685.py motor.py
```

echo topic before cutting to the car video (might need a brief video showing the sensors work over a white and black surface or?)

make use of launch file


launch file--say where it is, what nodes it launches and all. or should we just output the file contents here?

```
roslaunch rpi_car_line_follower rpi_car.launch
```

rostopic echo /infra_readings

### Rqt graph

<p align="center">
  <img src=images/rosgraph.png>
</p>

## Tracks

<p float="left">
  <img src=images/round_track.jpg width="400">
  <img src=images/fig_8.jpg width="400">
</p>

## Video demonstration

## Observations

Initial width of line was too thin, had to widen it using duck tape to enable the ir sensor to read the line well.

## Future work/suggestions
- Move project to ROS2
- Better power solution using 18560 batteries?

## References
Reference motor drive and code used, sensors used etc.

https://www.waveshare.com/wiki/Motor_Driver_HAT

(To be completed)
