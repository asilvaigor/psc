import ast
from PyQt5 import QtCore
from subprocess import Popen, PIPE

from tools.connection_handler.src.ProcessManager import ProcessManager

class ConnectionThread(QtCore.QThread):
    update_crazyflie_status = QtCore.pyqtSignal(object)
    add_crazyflie_available = QtCore.pyqtSignal(object)
    remove_crazyflie_available = QtCore.pyqtSignal(object)

    def __init__(self, drones_to_connect):
        QtCore.QThread.__init__(self)
        self.drones_to_connect = drones_to_connect
        self.connection_process = None

    def run(self):
        """
        Tries to get each drone id in the droones to connect list
        """

        # Adding initial connection text
        for drone in self.drones_to_connect:
            self.update_crazyflie_status.emit((drone, "connecting...", "blue"))

        # Process to get drones ids
        self.connection_process = Popen(["rosrun", "psc", "generate_launchfile.py"] +
                                        list(map(str, self.drones_to_connect)), stdout=PIPE)

        # Function decode a line in the generate launchfile script
        def decode_line(text):
            if text.find("retry") >= 0:
                # Showing message for retry
                drone_id = int(text[text.find("drone")+6:text.find("retry")-1])
                retry = int(text[text.find("/")-2:text.find("/")])
                self.update_crazyflie_status.emit((drone_id, "connection retry: " + str(retry), "darkorange"))
            elif text.find("Unable") >= 0:
                # Showing message for unable to connect and updating availability
                drone_id = int(text[text.find("drone")+6:text.find("drone")+8])
                self.update_crazyflie_status.emit((drone_id, "unable to connect", "red"))
                self.remove_crazyflie_available.emit(drone_id)
            elif text.find("Drone") >= 0:
                # Showing online drone message and updating availability
                drone_id = int(text[text.find("Drone")+6:text.find("Drone")+8])
                radio_id = text[text.find("address:")+9:-1]
                self.update_crazyflie_status.emit((drone_id, "id found: " + radio_id, "olive"))
                self.add_crazyflie_available.emit((drone_id, radio_id))

        # Getting output from connection process
        while self.connection_process.poll() is None:
            output = self.connection_process.stdout.readline()
            if len(output) > 0:
                decode_line(output)

    def force_stop(self):
        """
        Stops the connection process
        """

        # Stopping thread
        self.quit()

        # Killing connection process children
        ProcessManager(self.connection_process).close_all_child()

        # Killing connection process
        ProcessManager(self.connection_process).sigint()

