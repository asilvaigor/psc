class Event:
    def __init__(self, event_type):
        """
            :param event_type: 0 if it's a critical event
                         1 if stopping to avoid a future collision (when a projection of the path reaches a
                           horizontal or vertical minimum of some obstacle around which it may not go
                           without changing the homotopy class of the original path)
                         2 if restarting from case 1 when the obstacle has been reached
                         3 if stopping, slowing or speeding up when the path initiates or breaks contact with
                           some coordination space obstacle
                         4 if stopping each robot that reaches its goal
            :param more characteristics of the event?
        """
        self.__event_type = event_type

        # TODO What each event means and how to describe it
        if self.__event_type == 0:
            pass
        if self.__event_type == 1:
            pass
        if self.__event_type == 2:
            pass
        if self.__event_type == 3:
            pass
        if self.__event_type == 4:
            pass

    def get_type(self):
        return self.__event_type

    def run_event(self):
        # Function to run the event
        # TODO run the current event
        pass