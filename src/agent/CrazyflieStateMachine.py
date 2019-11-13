import time


class CrazyflieStateMachine:
    MOVING = 1
    INACTIVE = 2
    STOPPED = 3

    def __init__(self):
        self.__state = CrazyflieStateMachine.STOPPED
        self.__start_time = 0
        self.__duration = 0
        self.__will_land = False
        self.__just_stopped = False
        self.__is_stopped = True

    def is_inactive(self):
        self.__update()
        return self.__state == CrazyflieStateMachine.INACTIVE or \
               self.__state == CrazyflieStateMachine.STOPPED

    def just_stopped(self):
        self.__update()
        if self.__just_stopped:
            self.__just_stopped = False
            return True
        return False

    def is_stopped(self):
        self.__update()
        return self.__state == CrazyflieStateMachine.STOPPED

    def start_movement(self, duration, will_land=False):
        self.__state = CrazyflieStateMachine.MOVING
        self.__start_time = time.time()
        self.__duration = duration
        self.__will_land = will_land

    def __update(self):
        if self.__state == CrazyflieStateMachine.MOVING and \
                self.__duration < time.time() - self.__start_time:
            if self.__will_land:
                self.__state = CrazyflieStateMachine.STOPPED
                self.__will_land = False
                self.__just_stopped = True
            else:
                self.__state = CrazyflieStateMachine.INACTIVE
