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


floorcolorRGB = floorcolor.rgb()
floorcolorReflection = floorcolor.reflection()
linecolorRGB = linecolor.rgb()
linecolorReflection = linecolor.reflection()

def IsLineColorDistance(sensor: ColorSensor, compare: ColorCompareUsage):
    if compare.UseRGB:
        rgb = sensor.rgb()
        return ColorDistanceRGBSqr(rgb, linecolorRGB) < ColorDistanceRGBSqr(rgb, floorcolorRGB)
    else:
        reflection = sensor.reflection()
        return ValueDistanceSqr(reflection, linecolorReflection) < ValueDistanceSqr(reflection, floorcolorReflection)


def AlignSingleSensor():
    robot.turn(10)

def AlignTowardsLine(floorcolor: ColorHDR, linecolor: ColorHDR, compare: ColorCompareUsage):
    while not IsLineColorDistance(left_light, compare):
        robot.drive(0, 90)
        wait(1)
    robot.stop()
    robot.reset()
    startAngle = robot.angle()

    rotated = 0
    while not IsLineColorDistance(right_light, compare):
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
    

def invLerp(a, b, v):
    return (v - a) / (b - a)

global lastLine
lastLine = None

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
        global lastLine
        
        if lastLine is not None and robot.distance() - lastLine > 120 and robot.distance() - lastLine < 250:
            print(robot.distance() - lastLine)
            if TryPark(robot.distance() - lastLine):
                wait(5000)
                UnPark()
        
        lastLine = robot.distance()
    
    if IsLineColorDistance(right_light, compare):
        while IsLineColorDistance(right_light, compare):
            robot.drive(speed * 0.5, turning_rate + 10)
            wait(1)
    else:
        robot.drive(speed, -turning_rate)
        wait(1)

PARK_DISTANCE = 200

def TryPark(parkingWidth: int) -> bool:
    robot.straight(-(parkingWidth / 5))
    robot.turn(-90)
    wait(500)
    robot.stop()
    if sonar.distance() >= 350: # Parking spot Empty
        start = robot.distance()
        while robot.distance() - start < PARK_DISTANCE:
            if IsLineColorDistance(right_light, compare):
                robot.drive(100, -100)
            else:
                robot.drive(100, 0)
            wait(1)
        robot.stop()
        return True
    else: # Parking spot Taken
        for b in range(int(sonar.distance() / 100)):
            ev3.speaker.beep()
        robot.turn(90)
        wait(500)
        robot.stop()
        return False

def Park(parkingWidth: int):
    robot.straight(-(parkingWidth / 5))
    robot.turn(-90)
    wait(500)
    robot.stop()
    start = robot.distance()
    while robot.distance() - start < PARK_DISTANCE:
        if IsLineColorDistance(right_light, compare):
            robot.drive(100, -100)
        else:
            robot.drive(100, 0)
        wait(1)
    robot.stop()

def UnPark():
    start = robot.distance()
    while start - robot.distance() < PARK_DISTANCE + 25:
        robot.drive(-100, 0)
        wait(1)
    robot.stop()
    robot.turn(90)
    wait(500)
    robot.turn(-10)
    robot.stop()

def SingleSensorCount(parkingData: list):
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
        global lastLine
        
        if lastLine is not None and robot.distance() - lastLine > 120 and robot.distance() - lastLine < 250:
            parkingData.append(CheckParkEmpty(robot.distance() - lastLine))
        
        lastLine = robot.distance()
    
    if IsLineColorDistance(right_light, compare):
        while IsLineColorDistance(right_light, compare):
            robot.drive(speed * 0.5, turning_rate + 10)
            wait(1)
    else:
        robot.drive(speed, -turning_rate)
        wait(1)

def CheckParkEmpty(parkingWidth: int) -> bool:
    robot.straight(-(parkingWidth / 5))
    robot.turn(-90)
    wait(500)
    robot.stop()
    isEmpty = sonar.distance() >= 350
    robot.turn(90)
    wait(500)
    robot.stop()
    return isEmpty

global parkIndex
parkIndex = 0

def SingleSensorCountedDrive(parkingData: list):
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
        global lastLine
        
        if lastLine is not None and robot.distance() - lastLine > 120 and robot.distance() - lastLine < 250:
            global parkIndex
            if parkingData[parkIndex]: # If the recorded data says this parking spot is empty
                Park(robot.distance() - lastLine)
                wait(5000)
                UnPark()

            if parkIndex >= len(parkingData) - 1: # Prevent index out of range
                parkIndex = 0
            else:
                parkIndex += 1
        
        lastLine = robot.distance()
    
    if IsLineColorDistance(right_light, compare):
        while IsLineColorDistance(right_light, compare):
            robot.drive(speed * 0.5, turning_rate + 10)
            wait(1)
    else:
        robot.drive(speed, -turning_rate)
        wait(1)

#AlignTowardsLine(floorcolor, linecolor, compare)
Align(floorcolor, linecolor, compare)
#AlignSingleSensor()
#ev3.speaker.play_file(SoundFile.KUNG_FU)

def Start():
    startAngle = robot.angle()
    # List booleans of empty parking spots
    # Where index 0 is the first parking spot
    # and parkingData[index] = is this spot empty?
    parkingData = []
    # Count parking spots while the robot hasnt made a 360 degree turn
    # Not the most robust way since it will not work in some concave tracks
    while GetAngleBetween(startAngle, robot.angle()) < 350:
        SingleSensorCount(parkingData)
    # Reset lastLine because both functions use the same variable which is eeh coding but it works
    global lastLine
    lastLine = None
    while True:
        SingleSensorCountedDrive(parkingData)

Start()
