#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from ColorDetection import ColorHDR, ColorCompareUsage
import math

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

def GetAverageColor(sensor: ColorSensor) -> ColorHDR:
    sum = ColorHDR((0, 0, 0), 0, 0)
    for x in range(20):
        sum.add(ColorHDR.fromColorSensor(sensor))
        print(ColorHDR.fromColorSensor(sensor).Color)
        wait(1)
    sum.divide(20)
    return sum

# Write your program here.
def FindLineColor(floorcolor: ColorHDR) -> ColorHDR:
    while ColorHDR.fromColorSensor(right_light) == floorcolor:
        robot.drive(0, -45)
        wait(1)
    robot.stop()
    return ColorHDR.fromColorSensor(right_light)

floorcolor = ColorHDR.fromColorSensor(left_light)
linecolor = FindLineColor(floorcolor)
#print(floorcolor.average())
#print(linecolor.average())
compare = ColorHDR.compare(floorcolor, linecolor)
print(compare.UseRGB)
print(compare.UseReflection)
print(compare.UseAmbient)
ev3.speaker.beep()

#exit()
#print(floorcolor.Color)
#print(linecolor.Color)
#exit()

#wait(5000)

def straightUntilLine(floorcolor: ColorHDR, linecolor: ColorHDR):
    distance = 0
    leftDetectLine = linecolor.almostEqual(ColorHDR.fromColorSensor(left_light, compare), floorcolor)
    rightDetectLine = linecolor.almostEqual(ColorHDR.fromColorSensor(right_light, compare), floorcolor)
    while not leftDetectLine and not rightDetectLine:
        distance = robot.distance()
        robot.drive(100, 0)
        leftDetectLine = linecolor.almostEqual(ColorHDR.fromColorSensor(left_light, compare), floorcolor)
        rightDetectLine = linecolor.almostEqual(ColorHDR.fromColorSensor(right_light, compare), floorcolor)
        wait(1)
    
    robot.stop()
    return leftDetectLine
    if distance > 0:
        print(distance - robot.distance())
        robot.drive((distance - robot.distance()) * 4, 0)
        wait(250)
    else:
        return
    robot.stop()

def GetAngleBetween(angle1: int, angle2: int) -> int:
    if math.copysign(1, angle1) != math.copysign(1, angle2):
        return abs(angle1) + abs(angle2)
    else:
        return abs(abs(angle1) - abs(angle2))

def AlignTowardsLine(floorcolor: ColorHDR, linecolor: ColorHDR, compare: ColorCompareUsage):
    while not linecolor.almostEqual(ColorHDR.fromColorSensor(left_light, compare), floorcolor):
        robot.drive(0, 90)
        wait(1)
    robot.stop()
    startAngle = robot.angle()
    print(ColorHDR.fromColorSensor(right_light, compare).Color)
    print(linecolor.Color)
    while not linecolor.almostEqual(ColorHDR.fromColorSensor(right_light, compare), floorcolor):
        robot.drive(0, -90)
        wait(1)
    robot.stop()
    robot.turn(int(GetAngleBetween(startAngle, robot.angle()) / 2))

def Align(floorcolor: ColorHDR, linecolor: ColorHDR, compare: ColorCompareUsage):
    AlignTowardsLine(floorcolor, linecolor, compare)
    #robot.drive(68.5, 0)
    robot.drive(70, 0)
    wait(1000)
    AlignTowardsLine(floorcolor, linecolor, compare)

def rotate(floorcolor: ColorHDR, linecolor: ColorHDR, compare, leftTriggered):
    if leftTriggered or ColorHDR.fromColorSensor(left_light) == linecolor:
        angle = robot.angle()
        robot.drive(50, 0)
        # 70, 0
        wait(1001)
        while not linecolor.almostEqual(ColorHDR.fromColorSensor(right_light), floorcolor):
            robot.drive(0, -75)
            wait(1)

        robot.stop()
        print(robot.angle() - angle)
        robot.drive(0, 45)
        #AlignTowardsLine(floorcolor, linecolor, compare)
        wait(1001)
        robot.stop()
    
    elif not leftTriggered or ColorHDR.fromColorSensor(right_light) == linecolor:
        angle = robot.angle()
        robot.drive(50, 0)
        wait(1001)
        while not linecolor.almostEqual(ColorHDR.fromColorSensor(left_light), floorcolor):
            robot.drive(0, 75)
            wait(1)
        
        robot.stop()
        #print(robot.angle() - angle)
        robot.drive(0, -45)
        #AlignTowardsLine(floorcolor, linecolor, compare)
        wait(1001)
        robot.stop()
    elif ColorHDR.fromColorSensor(left_light) == linecolor and ColorHDR.fromColorSensor(right_light) == linecolor:
        robot.drive(0, 0)

#AlignTowardsLine(floorcolor, linecolor, compare)
Align(floorcolor, linecolor, compare)
#exit()
wait(500)
while True:
    leftTriggered = straightUntilLine(floorcolor, linecolor)
    rotate(floorcolor, linecolor, compare, leftTriggered)
    #Align(floorcolor, linecolor, compare)

#while True:
#    left_color = ColorHDR.fromColorSensor(left_light)
#    right_color = ColorHDR.fromColorSensor(right_light)
#    if left_color != linecolor and right_color != linecolor:
#        robot.drive(70, 0)
#        wait(1)
#    elif left_color == linecolor:
#        while ColorHDR.fromColorSensor(left_light) == linecolor:
#            robot.drive(0, -50)
#            wait(1)
#    elif right_color == linecolor:
#        while ColorHDR.fromColorSensor(right_light) == linecolor:
#            robot.drive(0, 50)
#            wait(1)
