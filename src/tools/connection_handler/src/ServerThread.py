import psutil, os, signal
from PyQt5 import QtCore
from subprocess import Popen, PIPE

TIMEOUT = 100


class ServerThread(QtCore.QThread):
    update_crazyflie_status = QtCore.pyqtSignal(object)
    display_messages = QtCore.pyqtSignal(object)

    def __init__(self, available_drones):
        QtCore.QThread.__init__(self)
        self.available_drones = available_drones
        self.server_process = None

    def run(self):
        """
        Launch server and crazyflie instances
        """
        self.display_messages.emit(("Starting connection...", "blue"))

        # Opening server process
        self.server_process = Popen(["roslaunch", "crazyflie_driver", "crazyflie_server.launch"], stdout=PIPE)

        # TODO treat errors and centralize message (in UI)

        self.display_messages.emit(("Server online", "darkgreen"))

        # TODO treat different behaviours of the crazyflies add

        # Update crazyflies status to connecting
        for drone in self.available_drones:
            self.update_crazyflie_status.emit((drone, "connecting...", "blue"))

        # Opening communications with drones
        for drone in self.available_drones:
            cf_process = Popen(["roslaunch", "psc", "run_real.launch",
                                "cf:=" + str(drone), "radio_id:=" + self.available_drones[drone]])
            # TODO use timeout
            while cf_process.poll() is None:
            # (output, err) = process.communicate()
                pass

            # Showing successful connection message
            self.update_crazyflie_status.emit((drone, "online", "green"))

    def stop(self):
        """
        Stop server process
        """

        # Getting child processes
        children_pids = [x.pid for x in psutil.Process(self.server_process.pid).children(recursive=True)]

        # Killing all the child processes
        for pid in children_pids:
            os.killpg(pid, signal.SIGINT)

        self.display_messages.emit(("Idle", "black"))

        # TODO treat possibly closing crazyflie_add end errors
        # Update crazyflies status to connecting
        for drone in self.available_drones:
            self.update_crazyflie_status.emit((drone, "offline id:"+self.available_drones[drone], "olive"))
