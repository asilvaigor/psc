import time


class CrazyflieStateMachine:
    """
    Controls the behavior of the drone, to switch between movement and stopping according to the
    duration of the actions.
    """

    MOVING = 1
    INACTIVE = 2  # Drone is moving its rotors but is idle.
    STOPPED = 3  # Drone's motors are stopped.

    def __init__(self):
        """
        Default constructor. Will start stopped.
        """
        self.__state = CrazyflieStateMachine.STOPPED
        self.__start_time = 0
        self.__duration = 0
        self.__will_land = False
        self.__just_stopped = False
        self.__is_stopped = True

    def is_inactive(self):
        """
        Checks if drone is inactive.
        :return: Boolean
        """
        self.__update()
        return self.__state == CrazyflieStateMachine.INACTIVE or \
               self.__state == CrazyflieStateMachine.STOPPED

    def just_stopped(self):
        """
        Checks if drone has just stopped. This is useful to send a 'stop' command to the crazyflie.
        :return: Boolean
        """
        self.__update()
        if self.__just_stopped:
            self.__just_stopped = False
            return True
        return False

    def is_stopped(self):
        """
        Checks if the drone is stopped.
        :return: Boolean
        """
        self.__update()
        return self.__state == CrazyflieStateMachine.STOPPED

    def start_movement(self, duration, will_land=False):
        """
        Starts a new movement.
        :param duration: Duration of the complete movement.
        :param will_land: If it is a landing movement. This makes the drone stop after finishing
        its movement.
        """
        self.__state = CrazyflieStateMachine.MOVING
        self.__start_time = time.time()
        self.__duration = duration
        self.__will_land = will_land

    def stop(self):
        """
        Stops a drone.
        """
        self.__state = CrazyflieStateMachine.STOPPED
        self.__just_stopped = True
        self.__will_land = False

    def __update(self):
        """
        Analyzes time to switch states.
        """
        if self.__state == CrazyflieStateMachine.MOVING and \
                self.__duration < time.time() - self.__start_time:
            if self.__will_land:
                self.stop()
            else:
                self.__state = CrazyflieStateMachine.INACTIVE
