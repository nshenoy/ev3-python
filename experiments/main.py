#!/usr/bin/env micropython

from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveSteering, follow_for_ms
from ev3dev2.sensor.lego import ColorSensor, GyroSensor, UltrasonicSensor
from ev3dev2.led import Leds
from sys import stderr
from time import sleep
import os
from libs.moveTankWithGyro import MoveTankWithGyro
from libs.logToDisplay import logToDisplay

def gyroMove():
    """Test code for gyro PID drive"""

    gyro_drive = MoveTankWithGyro(OUTPUT_A, OUTPUT_D)
    gyro_drive.gyro = GyroSensor()
    gyro_drive.calibrate_gyro()
    gyro_drive.follow_gyro(
        kp=11.3, ki=0.05, kd=3.2,
        speed=SpeedPercent(30),
        gyro_angle_tolerance=3,
        follow_for=follow_for_ms,
        ms=10000
    )

def main():
    gyroMove()

if __name__ == "__main__":
    main()
