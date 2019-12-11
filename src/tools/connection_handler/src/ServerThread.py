import psutil, os, signal, time
from PyQt5 import QtCore
from subprocess import Popen, PIPE

from tools.connection_handler.src.ProcessManager import ProcessManager

TIMEOUT = 10
NO_USB_DELAY = 3

class ServerThread(QtCore.QThread):
    update_crazyflie_status = QtCore.pyqtSignal(object)
    display_messages = QtCore.pyqtSignal(object)
    stop_server = QtCore.pyqtSignal(object)

    def __init__(self, available_drones):
        QtCore.QThread.__init__(self)
        self.available_drones = available_drones
        self.server_process = None
        self.cf_process = None
        self.working = True

    def run(self):
        """
        Launch server and crazyflie instances
        """
        self.display_messages.emit(("Starting connection...", "blue"))

        # Opening server process
        self.server_process = Popen(["roslaunch", "crazyflie_driver", "crazyflie_server.launch"], stdout=PIPE)

        self.display_messages.emit(("Server online", "darkgreen"))

        # Update crazyflies status to connecting
        for drone in self.available_drones:
            self.update_crazyflie_status.emit((drone, "connecting...", "blue"))

        # Opening communications with drones
        for drone in self.available_drones:
            self.cf_process = Popen(["roslaunch", "psc", "run_real.launch",
                                "cf:=" + str(drone), "radio_id:=" + self.available_drones[drone]],
                               stdout=PIPE, stderr=PIPE)

            start_time = time.time()
            while self.cf_process.poll() is None and time.time() < start_time+TIMEOUT:
                output, err = self.cf_process.communicate()
                err = str(err)
                if len(err) > 0:
                    # Treating no USB device error
                    if err.find("No matching USB Device") >= 0:
                        self.display_messages.emit(("USB radio not detected", "red"))
                        time.sleep(NO_USB_DELAY)
                        self.working = False
                        self.stop()
                        return

            # Treat timeout error
            if time.time() > start_time+TIMEOUT:
                self.update_crazyflie_status.emit((drone, "connection timeout", "red"))
                continue

            # Showing successful connection message
            self.update_crazyflie_status.emit((drone, "online", "green"))

        # Updating working status
        self.working = False

    def stop(self):
        """
        Stop server process
        """

        if self.working:
            return

        # Emit state change signal
        self.stop_server.emit(None)

        # Closing child processes
        ProcessManager(self.server_process).close_all_child()

        self.display_messages.emit(("Idle", "black"))

        # Update crazyflies status to connecting
        for drone in self.available_drones:
            self.update_crazyflie_status.emit((drone, "offline id:"+self.available_drones[drone], "olive"))

    def force_stop(self):
        """
        Stops everything including running crazyflie add
        """

        # Stopping thread
        self.quit()

        # Killing all running processes
        ProcessManager(self.cf_process).close_all_child()
        ProcessManager(self.server_process).close_all_child()
