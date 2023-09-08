#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from ColorDetection import ColorHDR

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

def DetectingLineColor(sensor: ColorSensor) -> bool:
    # Read the color and reflection
    color = ColorHDR.fromColorSensor(sensor)

    return color.ambient() > 10

# Create your objects here.
ev3 = EV3Brick()

# Motor definitions
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Sensor definitions
left_light = ColorSensor(Port.S2)
right_light = ColorSensor(Port.S3)

# Write your program here.

linecolor = ColorHDR.fromColorSensor(left_light)
ev3.speaker.beep()

wait(5000)

def straightUntilLine(linecolor: ColorHDR):
    distance = 0
    while ColorHDR.fromColorSensor(left_light) != linecolor and ColorHDR.fromColorSensor(right_light) != linecolor:
        distance = robot.distance()
        robot.drive(100, 0)
        wait(1)
    
    robot.stop()
    if distance > 0:
        print(distance - robot.distance())
        robot.drive((distance - robot.distance()) * 4, 0)
        wait(250)
    else: 
        return
    robot.stop()

def rotate(linecolor: ColorHDR):
    if ColorHDR.fromColorSensor(left_light) == linecolor:
        angle = robot.angle()
        robot.drive(70, 0)
        wait(1001)
        while ColorHDR.fromColorSensor(right_light) != linecolor:
            robot.drive(0, -75)

            wait(1)

        robot.stop()
        print(robot.angle() - angle)
        robot.drive(0, 45)
        wait(1001)
        robot.stop()
    
    elif ColorHDR.fromColorSensor(right_light) == linecolor:
        angle = robot.angle()
        robot.drive(70, 0)
        wait(1001)
        while ColorHDR.fromColorSensor(left_light) != linecolor:
            robot.drive(0, 75)

            wait(1)

        robot.stop()
        print(robot.angle() - angle)
        robot.drive(0, -45)
        wait(1001)
        robot.stop()
    elif ColorHDR.fromColorSensor(left_light) == linecolor and ColorHDR.fromColorSensor(right_light) == linecolor:
        robot.drive(0, 0)

def straighten(linecolor: ColorHDR):
    angle = 0
    while ColorHDR.fromColorSensor(left_light) != linecolor:
        robot.drive(0, 75)
        wait(1)
    robot.stop()
    angleleft = robot.angle()
    robot.reset()
    while ColorHDR.fromColorSensor(right_light) != linecolor:
        robot.drive(0, -75)
        wait(1)
    robot.stop()
    angleright = robot.angle()
    robot.reset()
    angle = angleright + angleleft
    robot.drive(0, angle / 2)
    
    wait(1000)

#straighten(linecolor)

#while True:
    #straightUntilLine(linecolor)
    #rotate(linecolor)





print(ColorHDR.fromColorSensor(left_light).rgb())
print(ColorHDR.fromColorSensor(right_light).rgb())
print(ColorHDR.fromColorSensor(left_light).reflection())
print(ColorHDR.fromColorSensor(right_light).reflection())
print(ColorHDR.fromColorSensor(left_light).ambient())
print(ColorHDR.fromColorSensor(right_light).ambient())