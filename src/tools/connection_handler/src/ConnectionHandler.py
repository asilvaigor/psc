import os, sys
import signal, psutil
import ast
from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget, QMessageBox, QListWidgetItem, QLabel
from PyQt5 import QtCore
from representations.Constants import N_DRONES
from subprocess import Popen, PIPE

from tools.connection_handler.src.ServerThread import ServerThread


class ConnectionHandler(Plugin):
    """
    Qt Plugin to manage the connection of crazyflie drones
    """

    def __init__(self, context):
        """
        Basic constructor. Has a bunch of Qt Plugin default calls,
        :param context: for Plugin constructor.
        """
        super(ConnectionHandler, self).__init__(context)
        self.__configure_plugin(context)

        # UI constants
        self.connected_drone_line_length = 50

        # UI configurations
        self.__configure_connection_button()
        self.__configure_run_server_button()
        self.__configure_connected_list()

        # State variables for the widget
        self.connection_mode = True
        self.selected_drone = -1
        # TODO : see if makes sente t still have this dictinary
        self.process_dict = {}
        self.server_thread = None
        self.server_running = False

        context.add_widget(self.__widget)

    def change_to_connection_mode(self, connection_mode):
        if connection_mode:
            self.__widget.connection_button.setText("Connect")
            self.connection_mode = True
        else:
            self.__widget.connection_button.setText("Disconnect")
            self.connection_mode = False

    def display_message(self, text, color):
        self.__widget.status_label.setText("<font color=\"%s\">%s</font>" % (color, text))

    def run_server(self, drones_ids, radio_ids):
        # Opening server process
        self.server_process = Popen(["roslaunch", "crazyflie_driver", "crazyflie_server.launch"], stdout=PIPE)

        # Opening communications with drones
        for i in range(len(drones_ids)):
            cf_process = Popen(["roslaunch", "psc", "run_real.launch",
                       "cf:=" + str(drones_ids[i]), "radio_id:=" + radio_ids[i]])
            self.process_dict[drones_ids[i]] = cf_process

    def __configure_connection_button(self):
        """
        Configures connection button
        """

        def extract_drones_to_connect(text):
            drones = text.split()
            for drone in drones:
                if not drone.isdigit():
                    return None
            return [int(drone) for drone in drones]

        def connect_to_drones(drones_ids):
            def formatted_string(drone, text, color):
                black_text = "<font color=\"black\">Drone %s" % str(drone)
                spaces = ""
                for i in range(self.connected_drone_line_length-len(black_text)-len(str(drone))+20):
                    spaces += "&nbsp;"
                color_text = "<font color=\"%s\">%s</font>" % (color, text)
                return black_text+spaces+color_text

            # Adding widget and text
            for drone_id in drones_ids:
                item = QListWidgetItem("", type=drone_id)
                self.__widget.connected_list.addItem(item)
                # TODO: change connect to add and disconnect to remove
                # TODO: change to update in separate thread
                start_message = "Connecting..."
                label = QLabel(formatted_string(drone_id, start_message, "blue"))
                self.__widget.connected_list.setItemWidget(item, label)

            # Getting drones ids
            process = Popen(["rosrun", "psc", "generate_launchfile.py"]+list(map(str, drones_ids)), stdout=PIPE)
            read = ""
            while process.poll() is None:
                (output, err) = process.communicate()
                read += output
            connected_drones = ast.literal_eval(read[read.find('['):read.find(']') + 1])
            radio_ids = ast.literal_eval("[" + read[read.find('{') + 1:read.find('}')] + "]")

            for drone_id in drones_ids:
                # Getting the item with id droone id
                item = next((self.__widget.connected_list.item(i) for i in
                             range(self.__widget.connected_list.count())
                             if self.__widget.connected_list.item(i).type() == drone_id), None)

                if drone_id in connected_drones:
                    message = "Connected"
                    label = QLabel(formatted_string(drone_id, message, "darkgreen"))
                    self.__widget.connected_list.setItemWidget(item, label)
                else:
                    message = "Unable to connect"
                    label = QLabel(formatted_string(drone_id, message, "red"))
                    self.__widget.connected_list.setItemWidget(item, label)
            self.run_server(connected_drones, radio_ids)
            # if radio_id == "":
            #     message = "Unable to connect"
            #     label = QLabel(formatted_string(drone_id, start_message, "blue"))
            #     self.__widget.connected_list.setItemWidget(item, label)
            #     label.setText(formatted_string(drone_id, message, "red"))
            # else:
            #     message = "Connected"
            #     label.setText(formatted_string(drone_id, message, "darkgreen"))
            #     cf_process = Popen(["roslaunch", "psc", "run_real.launch",
            #            "cf:=" + str(drone_id), "radio_id:=" + radio_id])
            #     self.process_dict[drone_id] = cf_process

        def handle_connection_button():
            list_widget = self.__widget.connected_list

            if self.connection_mode:
                # Treating input
                drones = extract_drones_to_connect(self.__widget.drones_line_edit.text())
                self.__widget.drones_line_edit.setText("")
                if drones is None:
                    self.display_message("Input error", "red")
                    return

                # Running checks and starting connection with given drones
                items = [list_widget.item(i) for i in range(list_widget.count())]
                items_ids = [item.type() for item in items]
                drones_to_add = []
                for drone in drones:
                    if drone < 1 or drone > N_DRONES:
                        message = "Drone %d out of bounds" % drone
                        self.display_message(message, "red")
                    elif drone in items_ids:
                        message = "Drone %d already in the list" % drone
                        self.display_message(message, "olive")
                    else:
                        # TODO: change the messages
                        message = "Idle"
                        self.display_message(message, "black")
                        drones_to_add += [drone]
                connect_to_drones(drones_to_add)
            else:
                # Killing process
                try:
                    if self.selected_drone in self.process_dict:
                        pro = self.process_dict[self.selected_drone]
                        children_pids = [x.pid for x in psutil.Process(pro.pid).children(recursive=True)]

                        for pid in children_pids:
                            os.killpg(pid, signal.SIGINT)

                        # pro.call("exit", shell=True)
                        # pro.kill()
                        # sys.stdout.flush()
                        # os
                        # s = ""
                        # children = pro.children(recursive=True)
                        # for child in children:
                        #     s+= str(child.pid) + " "
                        # self.display_message(s+"ola", "red")
                        # os.killpg(os.getpgid(pro.pid), signal.SIGINT)
                        os.killpg(os.getpgid(self.server_process.pid), signal.SIGINT)
                        # sys.stdout.flush()
                        del self.process_dict[self.selected_drone]
                    else:
                        self.display_message("not here", "red")
                except:
                    pass


                # Removing from graphical interface
                items = [list_widget.item(i) for i in range(list_widget.count())]
                item_to_remove = -1
                for item in items:
                    if item.type() == self.selected_drone:
                        item_to_remove = items.index(item)
                        break
                self.__widget.connected_list.takeItem(item_to_remove)
                self.change_to_connection_mode(True)
                self.selected_drone = -1

        self.__widget.connection_button.clicked.connect(handle_connection_button)

    def __configure_run_server_button(self):
        """
        Configures the button to launch the crazyflie server and lauches the crazyflie server thread
        """

        def handle_click():
            self.__widget.run_server_button.setText("ola")
            self.thread = ServerThread(1, "teste")

            def progress(text):
                self.display_message(text, "red")
                pass

            self.thread.data_downloaded.connect(progress)



            self.thread.start()
            # from PyQt5.QtCore import QObject
            # self.__widget.run_server_button.connect(thread, QtCore.SIGNAL("progress(int, int)"), progress)




        self.__widget.run_server_button.clicked.connect(handle_click)

    def __configure_connected_list(self):
        """
        Configures the list which shows the connected drones
        """

        def handle_item_change(item):
            if self.selected_drone == item.type():
                self.__widget.connected_list.clearSelection()
                self.selected_drone = -1
                self.change_to_connection_mode(True)
            else:
                self.selected_drone = item.type()
                self.change_to_connection_mode(False)

        self.__widget.connected_list.itemPressed.connect(handle_item_change)

    def __configure_plugin(self, context):
        """
        Default calls to correctly set the plugin in rqt.
        :param context: for Plugin constructor
        """
        self.setObjectName('ConnectionHandler')

        # Handles arguments
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())

        # Initiates widget
        self.__widget = QWidget()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'ConnectionHandler.ui')
        loadUi(ui_file, self.__widget)
        self.__widget.setObjectName('ConnectionHandlerUi')
        if context.serial_number() > 1:
            self.__widget.setWindowTitle(self.__widget.windowTitle() +
                                         (' (%d)' % context.serial_number()))

    def __shutdown_plugin(self):
        """
        Stops processes.
        """

        for key in self.process_dict:
            self.process_dict[key].kill()
