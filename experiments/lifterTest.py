#!/usr/bin/env micropython

from ev3dev2.motor import MediumMotor, OUTPUT_C
from time import sleep

def doLifter():
    """Test code for operating lifter"""

    lifter = MediumMotor()
    lifter.reset()
    lifter.run_to_rel_pos(position_sp=60, speed_sp=100, stop_action='hold')
    lifter.wait_while('running')
    sleep(1)
    lifter.run_to_rel_pos(position_sp=-60, speed_sp=100, stop_action='hold')
    lifter.wait_while('running')

if __name__ == "__main__":
    doLifter()