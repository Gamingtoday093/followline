#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from ColorDetection import ColorHDR, ColorCompareUsage, ColorDistanceRGBSqr, ValueDistanceSqr
import math

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()

# Motor definitions
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
sonar = UltrasonicSensor(Port.S4)

# Sensor definitions
left_light = ColorSensor(Port.S2)
right_light = ColorSensor(Port.S3)

def GetAverageColor(sensor: ColorSensor) -> ColorHDR:
    sum = ColorHDR((0, 0, 0), 0)
    lowest = -1
    highest = -1
    for x in range(20):
        newColor = ColorHDR.fromColorSensor(sensor)
        if lowest == -1 or newColor.reflection() < lowest:
            lowest = newColor.reflection()
        if highest == -1 or newColor.reflection() > highest:
            highest = newColor.reflection()
        sum.add(ColorHDR.fromColorSensor(sensor))
        robot.drive(10, 0)
        wait(1)
    sum.divide(20)
    print(lowest)
    print(highest)
    print(sum.reflection())
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
        color.updateFromColorSensor(right_light)
        if color.NoneColor() or ColorHDR.fromColorSensor(left_light).NoneColor() or sonar.distance() < 100:
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
    startAngle = robot.angle()
    while (color == floorcolor) and GetAngleBetween(robot.angle(), startAngle) <= 360:
        robot.drive(0, -65)
        color.updateFromColorSensor(right_light)
        wait(1)
    robot.stop()

    if (color == floorcolor):
        return SearchLineColor(floorcolor)

    return color

def GetVector(pointEnd: tuple, pointStart: tuple) -> tuple:
    return (pointEnd[0] - pointStart[0], pointEnd[1] - pointStart[1])

def GetMagnitude(point: tuple) -> float:
    return math.sqrt((point[0] * point[0]) + (point[1] * point[1]))

def GetNormalized(point: tuple) -> tuple:
    factor = GetMagnitude(point)
    if factor == 0:
        return point
    return (point[0] / factor, point[1] / factor)

def DotVector(point1: tuple, point2: tuple) -> float:
    return (point1[0] * point2[0]) + (point1[1] * point2[1])

def GetAngle(point1: tuple, point2: tuple) -> float:
    MagXMag = GetMagnitude(point1) * GetMagnitude(point2)
    if MagXMag == 0:
        MagXMag = 0.00000001
    return math.degrees(math.acos(DotVector(point1, point2) / MagXMag))

#targets = [(0, 0), (-100, 100), (-120, 120), (150, 150)]
targets = [(0, 0), (100, 0), (100, 100), (0, 100)]
currentTarget = 0

def GoToNextPosition(index: int):
    forwardVector = GetVector(targets[index], targets[currentTarget])
    alignVector = GetNormalized(GetVector(targets[currentTarget], targets[index - 1]))

    turnAngle = GetAngle(GetNormalized(forwardVector), alignVector)
    driveDistance = GetMagnitude(forwardVector)

    robot.turn(turnAngle)
    wait(500)
    robot.straight(driveDistance)

#while True:
#    for i in range(1, len(targets)):
#        GoToNextPosition(i)
#        currentTarget = i
#    currentTarget = 0

ev3.screen.draw_image(0, 0, ImageFile.AWAKE)
floorcolor = ColorHDR.fromColorSensor(left_light)
ev3.speaker.beep()
linecolor = FindLineColor(floorcolor)
compare = ColorHDR.compare(floorcolor, linecolor)
print("Line: RGB {0} Reflection ({1})".format(linecolor.rgb(), linecolor.reflection()))
print("Floor: RGB {0} Reflection ({1})".format(floorcolor.rgb(), floorcolor.reflection()))
print("Use RGB: {}".format(compare.UseRGB))
print("Use Reflection: {}".format(compare.UseReflection))
print("-- READY --")
#ev3.speaker.play_file(SoundFile.READY)


def AlignSingleSensor():
    robot.turn(10)

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

floorcolorRGB = floorcolor.rgb()
floorcolorReflection = floorcolor.reflection()
linecolorRGB = linecolor.rgb()
linecolorReflection = linecolor.reflection()
lineHasLowerReflection = linecolorReflection < floorcolorReflection

def IsLineColor(reflection: int) -> bool:
    return reflection > floorcolorReflection + 5
    return reflection + 5 < floorcolorReflection
    return reflection <= linecolorReflection + 10
    return reflection > floorcolorReflection or (lineHasLowerReflection and reflection <= linecolorReflection)

lineRGB = linecolor.rgb()
def isLineColorRGB(rgb):
    for i in range(len(rgb)):
        if not ColorHDR.equals(rgb[i], lineRGB[i]):
            return False
    return True

def IsLineColorDistance(sensor: ColorSensor, compare: ColorCompareUsage):
    if compare.UseRGB:
        rgb = sensor.rgb()
        return ColorDistanceRGBSqr(rgb, linecolorRGB) < ColorDistanceRGBSqr(rgb, floorcolorRGB)
    else:
        reflection = sensor.reflection()
        return ValueDistanceSqr(reflection, linecolorReflection) < ValueDistanceSqr(reflection, floorcolorReflection)


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


def invLerp(a, b, v):
    return (v - a) / (b - a)

def SingleSensorDrive():
    speed = 100
    turning_rate = 60

    distance = sonar.distance()
    if distance < 350:
        speed *= invLerp(100, 350, distance)

    if speed <= 10:
        robot.stop()
        wait(500)
        return

    if IsLineColorDistance(left_light, compare):
        if Park():
            wait(5000)
            UnPark()
    elif IsLineColorDistance(right_light, compare):
        #while IsLineColor(right_light.reflection()):
        while IsLineColorDistance(right_light, compare):
            robot.drive(speed * 0.5, turning_rate + 10)
            wait(1)
    else:
        robot.drive(speed, -turning_rate)
        wait(1)

def Park():
    robot.stop()
    robot.turn(-50)
    wait(500)
    robot.stop()
    if sonar.distance() >= 350: # Parking spot Empty
        robot.turn(50)
        wait(500)

        # Validate Parking spot
        startPosition = robot.distance()
        robot.drive(90, 0)
        wait(300)
        foundLineAgain = IsLineColorDistance(left_light, compare)
        while not ColorHDR.fromColorSensor(left_light).NoneColor() and not foundLineAgain and robot.distance() - startPosition < 250:
            robot.drive(90, -30)
            if IsLineColorDistance(right_light, compare):
                robot.turn(10)
            wait(1)
            foundLineAgain = IsLineColorDistance(left_light, compare)

        if not foundLineAgain:
            #robot.drive(-100, 0)
            #wait(1800)
            return False
        
        robot.straight(-(robot.distance() - startPosition))
        # - - -

        robot.drive(90, 0)
        wait(1800)
        robot.stop()
        robot.turn(-90)
        wait(500)
        #if sonar.distance() < 350: # Deep Parked, Parking spot Taken
        #    robot.turn(90)
        #    wait(500)
        #    return False
        robot.drive(100, 0)
        wait(2000)
        robot.stop()
        return True
    else: # Parking spot Taken
        robot.turn(50)
        wait(500)
        robot.drive(90, 0)
        wait(1000)
        robot.stop()
        return False

def UnPark():
    robot.drive(-100, 0)
    wait(2750)
    robot.stop()
    robot.turn(90)
    wait(500)
    robot.turn(-10)
    robot.stop()

#Align(floorcolor, linecolor, compare)
AlignSingleSensor()
#ev3.speaker.play_file(SoundFile.KUNG_FU)

#def Start(Speedmode: bool):
#    if Speedmode: # Speedmode
#        while True:
#            DriveAndTurnFast()
#    else: # Linemode
#        while True:
#            leftTriggered = StraightUntilLine(floorcolor, linecolor)
#            RotateAtIntersection(floorcolor, linecolor, compare, leftTriggered)

def Start():
    while True:
        SingleSensorDrive()

#oldIsLine = False
#while True:
#    newIsLine = IsLineColorDistance(right_light, compare)
#    if newIsLine != oldIsLine:
#        print(newIsLine)
#        oldIsLine = newIsLine
#
#    SingleSensorDrive()


Start()
