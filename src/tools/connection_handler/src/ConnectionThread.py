import ast
from PyQt5 import QtCore
from subprocess import Popen, PIPE


class ConnectionThread(QtCore.QThread):
    update_crazyflie_status = QtCore.pyqtSignal(object)
    add_crazyflie_available = QtCore.pyqtSignal(object)

    def __init__(self, drones_to_connect):
        QtCore.QThread.__init__(self)
        self.drones_to_connect = drones_to_connect

    def run(self):
        # Adding widget and text
        for drone in self.drones_to_connect:
            self.update_crazyflie_status.emit((drone, "connecting...", "blue"))

        # TODO add retries

        # Getting drones ids
        process = Popen(["rosrun", "psc", "generate_launchfile.py"] + list(map(str, self.drones_to_connect)), stdout=PIPE)
        read = ""
        while process.poll() is None:
            (output, err) = process.communicate()
            read += output
        connected_drones = ast.literal_eval(read[read.find('['):read.find(']') + 1])
        radio_ids = ast.literal_eval("[" + read[read.find('{') + 1:read.find('}')] + "]")

        # Showing results
        for drone_id in self.drones_to_connect:
            if drone_id in connected_drones:
                self.update_crazyflie_status.emit((drone_id, "offline id:" +
                                                   radio_ids[self.drones_to_connect.index(drone_id)], "olive"))
                self.add_crazyflie_available.emit((drone_id, radio_ids[self.drones_to_connect.index(drone_id)]))
            else:
                self.update_crazyflie_status.emit((drone_id, "unable to connect", "red"))
