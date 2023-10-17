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

def GetAngleBetween(angle1: int, angle2: int) -> int:
    if math.copysign(1, angle1) != math.copysign(1, angle2):
        return abs(angle1) + abs(angle2)
    else:
        return abs(abs(angle1) - abs(angle2))

def SearchLineColor(floorcolor: ColorHDR) -> ColorHDR:
    color = ColorHDR.fromColorSensor(right_light)
    while color == floorcolor:
        robot.drive(100, 0)
        print(color.Color)
        color.updateFromColorSensor(right_light)
        if color.NoneColor() or ColorHDR.fromColorSensor(left_light).NoneColor():
            robot.drive(-100, 0)
            wait(1000)
            robot.turn(90)
            return SearchLineColor(floorcolor)
        wait(1)
    #robot.stop()
    robot.drive(-10, 0)
    wait(1)

    return color

def FindLineColor(floorcolor: ColorHDR) -> ColorHDR:
    color = ColorHDR.fromColorSensor(right_light)
    #startAngle = robot.angle()
    #while (color == floorcolor) and GetAngleBetween(robot.angle(), startAngle) <= 360:
    #    robot.drive(0, -65)
    #    color.updateFromColorSensor(right_light)
    #    wait(1)
    #robot.stop()

    if (color == floorcolor):
        return SearchLineColor(floorcolor)

    return color

ev3.screen.draw_image(0, 0, ImageFile.AWAKE)
floorcolor = ColorHDR.fromColorSensor(left_light)
ev3.speaker.beep()
linecolor = FindLineColor(floorcolor)
compare = ColorHDR.compare(floorcolor, linecolor)
print(linecolor.Color)
print(floorcolor.Color)
print(compare.UseRGB)
print(compare.UseReflection)
ev3.speaker.play_file(SoundFile.READY)

def AlignSingleSensor(floorcolor: ColorHDR, linecolor: ColorHDR, lineSensor: ColorSensor, compare: ColorCompareUsage):
    sensorColor = ColorHDR.fromColorSensor(lineSensor)
    while linecolor.almostEqual(sensorColor, floorcolor):
        robot.drive(100, 0)
        sensorColor.updateFromColorSensor(linecolor)
        wait(1)
    
    rotMultiplier = 1
    if lineSensor == left_light:
        rotMultiplier = -1

    while not linecolor.almostEqual(sensorColor, floorcolor):
        robot.drive(0, rotMultiplier * 90)
        sensorColor.updateFromColorSensor(linecolor)
        wait(1)

def AlignTowardsLine(floorcolor: ColorHDR, linecolor: ColorHDR, compare: ColorCompareUsage):
    while not linecolor.almostEqual(ColorHDR.fromColorSensor(left_light, compare), floorcolor):
        robot.drive(0, 90)
        wait(1)
    robot.stop()
    robot.reset()
    startAngle = robot.angle()

    rotated = 0
    while not linecolor.almostEqual(ColorHDR.fromColorSensor(right_light, compare), floorcolor):
        robot.drive(0, -90)
        rotated += 90 / 1000
        wait(1)
    robot.stop()

    robot.turn(int(GetAngleBetween(startAngle, robot.angle()) / 2))

def Align(floorcolor: ColorHDR, linecolor: ColorHDR, compare: ColorCompareUsage):
    AlignTowardsLine(floorcolor, linecolor, compare)
    robot.drive(75, 0)
    wait(1000)
    AlignTowardsLine(floorcolor, linecolor, compare)
    robot.drive(-75, 0)
    wait(1000)
    AlignTowardsLine(floorcolor, linecolor, compare)
    robot.drive(75, 0)
    wait(1000)
    AlignTowardsLine(floorcolor, linecolor, compare)

def RotateAtIntersection(floorcolor: ColorHDR, linecolor: ColorHDR, compare: ColorCompareUsage, leftTriggered: bool):
    if leftTriggered:
        ev3.screen.draw_image(0, 0, ImageFile.LEFT)
        robot.drive(60, 0)
        wait(1000)

        while not linecolor.almostEqual(ColorHDR.fromColorSensor(right_light, compare), floorcolor):
            robot.drive(0, -75)
            wait(1)

        robot.stop()
        robot.reset()
        while not linecolor.almostEqual(ColorHDR.fromColorSensor(left_light, compare), floorcolor):
            robot.drive(0, 75)
            wait(1)

        newAngle = -int(GetAngleBetween(0, robot.angle()) / 2)
        if newAngle < -90:
            robot.turn(newAngle)
        else:
            robot.turn(-45)
    
    elif not leftTriggered:
        ev3.screen.draw_image(0, 0, ImageFile.RIGHT)
        robot.drive(60, 0)
        wait(1000)

        while not linecolor.almostEqual(ColorHDR.fromColorSensor(left_light, compare), floorcolor):
            robot.drive(0, 75)
            wait(1)
        
        robot.stop()
        robot.reset()
        while not linecolor.almostEqual(ColorHDR.fromColorSensor(right_light, compare), floorcolor):
            robot.drive(0, -75)
            wait(1)

        turnAngle = int(GetAngleBetween(0, robot.angle()) / 2)
        if turnAngle < 90:
            robot.turn(turnAngle)
        else:
            robot.turn(45)

    robot.stop()

def StraightUntilLine(floorcolor: ColorHDR, linecolor: ColorHDR) -> bool:
    ev3.screen.draw_image(0, 0, ImageFile.FORWARD)

    leftDetectLine = linecolor.almostEqual(ColorHDR.fromColorSensor(left_light, compare), floorcolor)
    rightDetectLine = linecolor.almostEqual(ColorHDR.fromColorSensor(right_light, compare), floorcolor)
    while not leftDetectLine and not rightDetectLine:
        robot.drive(100, 0)
        leftDetectLine = linecolor.almostEqual(ColorHDR.fromColorSensor(left_light, compare), floorcolor)
        rightDetectLine = linecolor.almostEqual(ColorHDR.fromColorSensor(right_light, compare), floorcolor)
        wait(1)
    
    robot.stop()

    if leftDetectLine and rightDetectLine:
        robot.drive(-60, 0)
        wait(1000)
        AlignTowardsLine(floorcolor, linecolor, compare)
        return StraightUntilLine(floorcolor, linecolor)

    return leftDetectLine

floorcolorReflection = floorcolor.reflection()
linecolorReflection = linecolor.reflection()
lineHasLowerReflection = linecolorReflection < floorcolorReflection
def IsLineColor(reflection: int) -> bool:
    return reflection <= linecolorReflection + 10
    return reflection > floorcolorReflection or (lineHasLowerReflection and reflection <= linecolorReflection)

def DriveAndTurnFast():
    left_reflection = left_light.reflection()
    right_reflection = right_light.reflection()
    if not IsLineColor(left_reflection) and not IsLineColor(right_reflection):
        robot.drive(200, 0)
        wait(1)
    elif IsLineColor(left_reflection):
        while IsLineColor(left_light.reflection()):
            robot.drive(100, -140)
            wait(1)
    elif IsLineColor(right_reflection):
        while IsLineColor(right_light.reflection()):
            robot.drive(100, 140)
            wait(1)

#AlignSingleSensor(floorcolor, linecolor, right_light, compare)
#Align(floorcolor, linecolor, compare)
ev3.speaker.play_file(SoundFile.KUNG_FU)

raise Exception("DONE")

def Start(Speedmode: bool):
    if Speedmode: # Speedmode
        while True:
            DriveAndTurnFast()
    else: # Linemode
        while True:
            leftTriggered = StraightUntilLine(floorcolor, linecolor)
            RotateAtIntersection(floorcolor, linecolor, compare, leftTriggered)

Start(Speedmode=False)
