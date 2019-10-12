#!/usr/bin/env micropython

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, MoveTank, SpeedPercent
from ev3dev2.sensor.lego import GyroSensor
from libs.logToDisplay import logToDisplay
from time import sleep
import math
import sys

def gyroRotateAbsoluteAngle():
    """Test code for rotating using the gyror"""
    tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)
    gyro = GyroSensor()
    logger = logToDisplay()

    for sleepTime in [2, 0.5]:
        gyro.mode = 'GYRO-RATE'
        gyro.mode = 'GYRO-ANG'
        sleep(sleepTime)

    startingAngle = gyro.angle
    endingAngle = 90

    while True:
        currentAngle = gyro.angle
        logger.log("Current angle {}".format(currentAngle))
        if (currentAngle >= endingAngle-2 and currentAngle <= endingAngle+2):
            tank_drive.stop()
            break
        elif (currentAngle > endingAngle):
            leftSpeed = SpeedPercent(-5)
            rightSpeed = SpeedPercent(5)
        else:
            leftSpeed = SpeedPercent(5)
            rightSpeed = SpeedPercent(-5)

        tank_drive.on(leftSpeed, rightSpeed)

        sleep(0.1)

if __name__ == "__main__":
    gyroRotateAbsoluteAngle()