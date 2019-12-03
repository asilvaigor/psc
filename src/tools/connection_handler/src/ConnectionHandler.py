import os
from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget, QMessageBox, QListWidgetItem, QLabel
from representations.Constants import N_DRONES


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
        self.__configure_connected_list()

        # State variables for the widget
        self.connection_mode = True
        self.selected_drone = -1

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

        def connect_to_drone(drone_id):
            def formatted_string(drone, text, color):
                black_text = "<font color=\"black\">Drone %s" % str(drone)
                spaces = ""
                for i in range(self.connected_drone_line_length-len(black_text)-len(str(drone))+20):
                    spaces += "&nbsp;"
                color_text = "<font color=\"%s\">%s</font>" % (color, text)
                return black_text+spaces+color_text

            # Adding widget and text
            item = QListWidgetItem("", type=drone_id)
            self.__widget.connected_list.addItem(item)
            start_message = "connecting..."
            label = QLabel(formatted_string(drone_id, start_message, "blue"))
            self.__widget.connected_list.setItemWidget(item, label)

            # Getting drone id


        def handle_connection_button():
            list_widget = self.__widget.connected_list

            if self.connection_mode:
                # Treating input
                drones = extract_drones_to_connect(self.__widget.drones_line_edit.text())
                self.__widget.drones_line_edit.setText("")
                if drones is None:
                    self.display_message("Input error", "red")
                    return

                # Running checks and starting connection
                items = [list_widget.item(i) for i in range(list_widget.count())]
                items_ids = [item.type() for item in items]
                for drone in drones:
                    if drone < 1 or drone > N_DRONES:
                        message = "Drone %d out of bounds" % drone
                        self.display_message(message, "red")
                    elif drone in items_ids:
                        message = "Drone %d already connected" % drone
                        self.display_message(message, "olive")
                    else:
                        message = "Idle"
                        self.display_message(message, "black")
                        connect_to_drone(drone)
            else:

                # Removing from graphical interface
                items = [list_widget.item(i) for i in range(list_widget.count())]
                item_to_remove = -1
                for item in items:
                    if item.type == self.selected_drone:
                        item_to_remove = items.index(item)
                        break
                self.__widget.connected_list.takeItem(item_to_remove)
                self.change_to_connection_mode(True)
                self.selected_drone = -1

        self.__widget.connection_button.clicked.connect(handle_connection_button)

    def __configure_connected_list(self):
        """
        Configures the list which shows the connected drones
        """

        def handle_item_change(item):
            if self.selected_drone == item.type:
                self.__widget.connected_list.clearSelection()
                self.selected_drone = -1
                self.change_to_connection_mode(True)
            else:
                self.selected_drone = item.type
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
