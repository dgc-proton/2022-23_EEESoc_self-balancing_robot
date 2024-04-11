# Self Balancing Robot

This repository contains files relating to the final version of a self-balancing robot built by Dave & Anoop to enter the 2022-23 Aberdeen University Electrical & Electronic Engineering Society Robotics League; a self balancing robot competition. There are some pictures of the robot competing ([self-balancing for time](competition_pic_1.jpg), [hill-climbing with weight](competition_pic_2.jpg), [hill-climbing without weight](competition_pic_3.jpg)), the [robot chassis design](chasis.SLDPRT) in CAD (Solidworks), and the [source code](second_chassis_v3.1.ino) for the microcontroller.

The competition had 7 events which all competitors had to compete in:

- Self-balancing for time
- Self-balancing for time with added weight
- Maximum distance in a straight line
- Maximum distance in a straight line with added weight
- Maximum distance going up an incline
- Maximum distance going up an incline with added weight
- Robot wrestling (this was a suprise event that was revealed during the competition)

Overall our robot came in second place in the competition. It performed particularly well in the suprise wrestling round due to our remote control implementation.


# Parts List

The parts allowed were restricted to the below as part of the competition:

- 3D printed chassis
- Arduino Uno microcontroller unit (or clone)
- L298N motor driver
- MPU6050 based 3-axis gyroscope & accelerometer (6DOF) sensor
- TT motor x 2 (3-6V, gearing 1:48 plastic)
- Elegoo 7.4V lithium battery pack
- Elegoo IR remote control
- IR sensor
- wheels x 2
