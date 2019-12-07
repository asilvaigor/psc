import os
import signal, psutil
import ast
from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget, QMessageBox, QListWidgetItem, QLabel
from representations.Constants import N_DRONES
from subprocess import Popen, PIPE

from tools.connection_handler.src.ServerThread import ServerThread
from tools.connection_handler.src.ConnectionThread import ConnectionThread


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
        self.__configure_status_label()

        # State variables for the widget
        self.connection_mode = True
        self.selected_drone = -1
        self.connection_thread = None
        self.server_thread = None
        self.server_running = False
        self.available_drones = {}

        context.add_widget(self.__widget)

    def change_to_connection_mode(self, connection_mode):
        if connection_mode:
            self.__widget.connection_button.setText("Add")
            self.connection_mode = True
        else:
            self.__widget.connection_button.setText("Remove")
            self.connection_mode = False

    def display_message(self, text, color):
        self.__widget.status_label.setText("<font color=\"%s\">%s</font>" % (color, text))

    def update_item_message(self, drone_id, text, color):
        def formatted_string(drone, text, color):
            black_text = "<font color=\"black\">Drone %s" % str(drone)
            spaces = ""
            for i in range(self.connected_drone_line_length - len(black_text) - len(str(drone)) + 20):
                spaces += "&nbsp;"
            color_text = "<font color=\"%s\">%s</font>" % (color, text)
            return black_text + spaces + color_text

        # Getting the item with id drone id
        item = next((self.__widget.connected_list.item(i) for i in range(self.__widget.connected_list.count())
                     if self.__widget.connected_list.item(i).type() == drone_id), None)

        # Setting label text
        label = QLabel(formatted_string(drone_id, text, color))
        self.__widget.connected_list.setItemWidget(item, label)

    def __configure_status_label(self):
        """
        Sets status label initial text
        """

        self.display_message("Idle", "black")

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
                    else:
                        message = "Idle"
                        self.display_message(message, "black")
                        drones_to_add += [drone]
                        if drone not in items_ids:
                            class ListItem(QListWidgetItem):
                                def __lt__(self, other):
                                    return self.type() < other.type()
                            item = ListItem("", type=drone)
                            self.__widget.connected_list.addItem(item)

                # Creating connection thread and connecting signals to thread
                self.connection_thread = ConnectionThread(drones_to_add)
                self.connection_thread.update_crazyflie_status.connect(
                    lambda (d_id, m, c): self.update_item_message(d_id, m, c))

                def add_available((drone_id, radio_id)):
                    self.available_drones[drone_id] = radio_id
                self.connection_thread.add_crazyflie_available.connect(add_available)

                def remove_available(drone_id):
                    if drone_id in self.available_drones:
                        del self.available_drones[drone_id]
                self.connection_thread.remove_crazyflie_available.connect(remove_available)

                self.connection_thread.start()
            else:
                # Removing drone from the available drones list
                if self.selected_drone in self.available_drones:
                    del self.available_drones[self.selected_drone]

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
            if not self.server_running:
                self.__widget.run_server_button.setText("Stop")
                self.server_running = True
                self.__widget.connection_button.setEnabled(False)

                # Starting thread and connecting signals
                self.server_thread = ServerThread(self.available_drones)
                self.server_thread.display_messages.connect(lambda (m, c): self.display_message(m, c))
                self.server_thread.update_crazyflie_status.connect(
                    lambda (i, m, c): self.update_item_message(i, m, c))

                self.server_thread.start()
            else:
                self.__widget.run_server_button.setText("Run")
                self.server_running = False
                self.server_thread.stop()
                self.__widget.connection_button.setEnabled(True)

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
        Stops kills server and open processes
        """

        # TODO stop connection process and sollve bug in stop server thread

        if self.server_thread is not None:
            self.server_thread.stop()

