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

col1 = ColorHDR((0, 0, 0), 2, 6)
col2 = ColorHDR((2, 3, 2), 0, 0)

print(col1 == col2)
input()


def DetectingLineColor(sensor: ColorSensor) -> bool:
    # Read the color and reflection
    color = ColorHDR.fromColorSensor(sensor)

    return color.ambient() > 80

# Create your objects here.
ev3 = EV3Brick()

# Motor definitions
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Sensor definitions
left_light = ColorSensor(Port.S2)
right_light = ColorSensor(Port.S1)

# Write your program here.
if DetectingLineColor(left_light):
    ev3.speaker.beep()
ev3.speaker.beep()

#hejhej
