from ev3dev2.motor import LargeMotor, MoveTank, SpeedNativeUnits, follow_for_forever
from time import sleep
from logging import getLogger

log = getLogger(__name__)

class GyroFollowErrorLostAngle(Exception):
    """
    Raised when a gyro following robot has lost the angle
    """
    pass


class GyroFollowErrorTooFast(Exception):
    """
    Raised when a gyro following robot has been asked to follow
    an angle at an unrealistic speed
    """
    pass


class MoveTankWithGyro(MoveTank):
    def __init__(self, left_motor_port, right_motor_port, desc=None, motor_class=LargeMotor):
        super(MoveTankWithGyro, self).__init__(
            left_motor_port, right_motor_port, desc, motor_class)
        self.gyro = None

    def calibrate_gyro(self):
        """
        Calibrates the gyro sensor.

        NOTE: This takes 1sec to run
        """
        assert self.gyro, "GyroSensor must be defined"

        for x in range(2):
            self.gyro.mode = 'GYRO-RATE'
            self.gyro.mode = 'GYRO-ANG'
            sleep(0.5)

    def follow_gyro(self,
                    kp, ki, kd,
                    speed,
                    target_angle=0,
                    sleep_time=0.01,
                    follow_for=follow_for_forever,
                    **kwargs
        ):
        """
        PID line follower
        ``kp``, ``ki``, and ``kd`` are the PID constants.
        ``speed`` is the desired speed of the midpoint of the robot
        ``target_angle`` is the angle we want drive
        ``sleep_time`` is how many seconds we sleep on each pass through
            the loop.  This is to give the robot a chance to react
            to the new motor settings. This should be something small such
            as 0.01 (10ms).
        ``follow_for`` is called to determine if we should keep following the
            line or stop.  This function will be passed ``self`` (the current
            ``MoveWithGyroTank`` object). Current supported options are:
            - ``follow_for_forever``
            - ``follow_for_ms``
        ``**kwargs`` will be passed to the ``follow_for`` function
        Example:
        .. code:: python
            from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveTank, SpeedPercent, follow_for_ms
            from ev3dev2.sensor.lego import GyroSensor
            tank = MoveTank(OUTPUT_A, OUTPUT_B)
            tank.gyro = GyroSensor()
            try:
                # Follow the line for 4500ms
                tank.calibrate_gyro()
                tank.follow_gyro(
                    kp=11.3, ki=0.05, kd=3.2,
                    speed=SpeedPercent(30),
                    follow_for=follow_for_ms,
                    ms=4500
                )
            except Exception:
                tank.stop()
                raise
        """
        assert self.gyro, "GyroSensor must be defined"

        integral = 0.0
        last_error = 0.0
        derivative = 0.0
        speed_native_units = speed.to_native_units(self.left_motor)
        MAX_SPEED = SpeedNativeUnits(self.max_speed)

        while follow_for(self, **kwargs):
            current_angle = self.gyro.angle
            error = current_angle - target_angle
            integral = integral + error
            derivative = error - last_error
            last_error = error
            turn_native_units = (kp * error) + (ki * integral) + (kd * derivative)

            left_speed = SpeedNativeUnits(speed_native_units - turn_native_units)
            right_speed = SpeedNativeUnits(speed_native_units + turn_native_units)

            if left_speed > MAX_SPEED:
                log.info("%s: left_speed %s is greater than MAX_SPEED %s" %
                        (self, left_speed, MAX_SPEED))
                self.stop()
                raise GyroFollowErrorTooFast(
                    "The robot is moving too fast to follow the angle")

            if right_speed > MAX_SPEED:
                log.info("%s: right_speed %s is greater than MAX_SPEED %s" %
                        (self, right_speed, MAX_SPEED))
                self.stop()
                raise GyroFollowErrorTooFast(
                    "The robot is moving too fast to follow the angle")

            if sleep_time:
                sleep(sleep_time)

            self.on(left_speed, right_speed)

        self.stop()

    def pivot_gyro(self,
                    speed,
                    target_angle=0,
                    sleep_time=0.01
        ):
        """
        Pivot Turn
        ``speed`` is the desired speed of the midpoint of the robot
        ``target_angle`` is the angle we want drive
        ``sleep_time`` is how many seconds we sleep on each pass through
            the loop.  This is to give the robot a chance to react
            to the new motor settings. This should be something small such
            as 0.01 (10ms).
        Example:
        .. code:: python
            from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveTank, SpeedPercent
            from ev3dev2.sensor.lego import GyroSensor
            tank = MoveTank(OUTPUT_A, OUTPUT_B)
            tank.gyro = GyroSensor()
            try:
                # Reset gyro sensor to zero
                tank.calibrate_gyro()

                # Pivot 30 degrees
                tank.pivot_gyro(
                    speed=SpeedPercent(5),
                    target_angle(30)
                )
            except Exception:
                tank.stop()
                raise
        """
        assert self.gyro, "GyroSensor must be defined"

        speed_native_units = speed.to_native_units(self.left_motor)
        target_reached = False

        while not target_reached:
            current_angle = self.gyro.angle
            if (current_angle >= target_angle-2 and current_angle <= target_angle+2):
                target_reached = True
                self.stop()
            elif (current_angle > target_angle):
                left_speed = SpeedNativeUnits(-1 * speed_native_units)
                right_speed = SpeedNativeUnits(speed_native_units)
            else:
                left_speed = SpeedNativeUnits(speed_native_units)
                right_speed = SpeedNativeUnits(-1 * speed_native_units)

            if sleep_time:
                sleep(sleep_time)

            self.on(left_speed, right_speed)

