#!/usr/bin/env micropython

from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.led import Leds
from time import sleep
from libs.logToDisplay import logToDisplay

myLogger = logToDisplay()

def calibrateGyro(gyro):
    """
    Calibrates the gyro sensor.

    NOTE: This takes 3sec to run
    """

    myLogger.log("Calibrating gyro...")
    for sleepTime in [2, 0.5]:
        gyro.mode = 'GYRO-RATE'
        gyro.mode = 'GYRO-ANG'
        sleep(sleepTime)
    myLogger.log("DONE")


def doGyro():
    """Test code for using the gyro sensor"""

    leds = Leds()
    gyroSensor = GyroSensor()

    # Reset gyro
    calibrateGyro(gyroSensor)

    while True:
        # Fetch the current angle from the gyro
        angle = gyroSensor.angle

        if angle > 1:
            # veering to the right
            leds.set_color('LEFT', 'AMBER')
            leds.set_color('RIGHT', 'RED')
            message = "Veering RIGHT {} deg"
        elif angle < -1:
            # veering to the left
            leds.set_color('LEFT', 'RED')
            leds.set_color('RIGHT', 'AMBER')
            message = "Veering LEFT {} deg"
        elif angle == 0:
            # straight
            leds.set_color('LEFT', 'GREEN')
            leds.set_color('RIGHT', 'GREEN')
            message = "STRAIGHT"
        else:
            # angle is a little off
            leds.set_color('LEFT', 'AMBER')
            leds.set_color('RIGHT', 'AMBER')
            message = "Off by {} deg"

        # Display results
        myLogger.log(message.format(angle))

        # wait for a bit before sampling next gyro reading
        sleep(0.5)

if __name__ == "__main__":
    doGyro()