import os, time
from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget, QMessageBox, QListWidgetItem, QLabel, QTreeWidgetItem
from representations.Constants import N_DRONES

from tools.connection_handler.src.ServerThread import ServerThread
from tools.connection_handler.src.ConnectionThread import ConnectionThread

SELECTION_DELAY = 0.1


class ConnectionHandler(Plugin):
    """
    Qt Plugin to manage the connection of crazyflie drones
    Color conventions: green->success
                       olive->partial success
                       darkorange->partial error
                       red->error
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
        self.__configure_drones_list()
        self.__configure_status_label()
        self.__configure_drones_list()

        # State variables for the widget
        self.__last_click = 0
        self.__add_mode = True
        self.__selected_drone = -1
        self.__connection_thread = None
        self.__server_thread = None
        self.__server_running = False
        self.__available_drones = {}

        context.add_widget(self.__widget)

    def __change_to_add_mode(self, add_mode):
        """
        Changes the conection button mode from add to remove
        :param add_mode: bool which indicates if the mode is add mode
        """
        if add_mode:
            self.__widget.connection_button.setText("Add")
            self.__add_mode = True
        else:
            self.__widget.connection_button.setText("Remove")
            self.__add_mode = False

    def __display_message(self, text, color):
        """
        Displays a message in the text output label
        :param text: string with the message
        :param color: color of the message
        """
        self.__widget.status_label.setText("<font color=\"%s\">%s</font>" % (color, text))

    def __update_item_message(self, drone_id, text, color):
        # Getting the item with id drone id
        item = next((self.__widget.drones_list.topLevelItem(i)
                     for i in range(self.__widget.drones_list.topLevelItemCount())
                     if self.__widget.drones_list.topLevelItem(i).type() == drone_id), None)

        # Setting item texts
        drone_name = "<font color=\"black\">Drone %s" % str(drone_id)
        status_text = "<font color=\"%s\">%s</font>" % (color, text)
        drone_label, status_label = QLabel(drone_name), QLabel(status_text)
        self.__widget.drones_list.setItemWidget(item, 0, drone_label)
        self.__widget.drones_list.setItemWidget(item, 1, status_label)

        # Sorting items (columns 0 in ascending order)
        self.__widget.drones_list.sortItems(0, 0)

    def __configure_status_label(self):
        """
        Sets status label initial text
        """

        self.__display_message("Idle", "black")

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

        def handle_click():
            # Getting the widget with the list
            tree_widget = self.__widget.drones_list

            if self.__add_mode:
                # Display general connection message
                message = "Connecting to drones..."
                self.__display_message(message, "blue")

                # Treating input
                drones = extract_drones_to_connect(self.__widget.drones_line_edit.text())
                self.__widget.drones_line_edit.setText("")
                if drones is None:
                    self.__display_message("Input error", "red")
                    return

                # Checking for ongoing connection
                if self.__connection_thread is not None and self.__connection_thread.isRunning():
                    message = "Radio busy, please wait..."
                    self.__display_message(message, "olive")
                    return

                # Running checks and starting connection with given drones
                items = [tree_widget.topLevelItem(i) for i in range(tree_widget.topLevelItemCount())]
                items_ids = [item.type() for item in items]
                drones_to_add = []

                for drone in drones:
                    if drone < 1 or drone > N_DRONES:
                        message = "Drone %d out of bounds" % drone
                        self.__display_message(message, "red")
                    else:
                        drones_to_add += [drone]
                        if drone not in items_ids:
                            class ListItem(QTreeWidgetItem):
                                def __lt__(self, other):
                                    return self.type() < other.type()
                            item = ListItem(drone)
                            tree_widget.addTopLevelItem(item)

                # Creating connection thread and connecting signals to thread
                self.__connection_thread = ConnectionThread(drones_to_add)
                self.__connection_thread.update_crazyflie_status.connect(
                    lambda (d_id, m, c): self.__update_item_message(d_id, m, c))

                def add_available((drone_id, radio_id)):
                    self.__available_drones[drone_id] = radio_id
                self.__connection_thread.add_crazyflie_available.connect(add_available)

                def remove_available(drone_id):
                    if drone_id in self.__available_drones:
                        del self.__available_drones[drone_id]
                self.__connection_thread.remove_crazyflie_available.connect(remove_available)

                def handle_idle_signal():
                    if not self.__server_running:
                        message = "Idle"
                        self.__display_message(message, "black")
                self.__connection_thread.idle_signal.connect(handle_idle_signal)

                self.__connection_thread.start()
            else:
                # Checking if the connection server is running
                if self.__connection_thread is not None and self.__connection_thread.isRunning():
                    message = "Can't remove drones while attempting to connect"
                    self.__display_message(message, "darkorange")
                    return

                # Removing drone from the available drones list
                if self.__selected_drone in self.__available_drones:
                    del self.__available_drones[self.__selected_drone]

                # Removing from graphical interface
                items = [tree_widget.topLevelItem(i) for i in range(tree_widget.topLevelItemCount())]
                item_to_remove = -1
                for item in items:
                    if item.type() == self.__selected_drone:
                        item_to_remove = items.index(item)
                        break
                self.__widget.drones_list.takeTopLevelItem(item_to_remove)
                self.__change_to_add_mode(True)
                self.__selected_drone = -1

        self.__widget.connection_button.clicked.connect(handle_click)

    def __configure_run_server_button(self):
        """
        Configures the button to launch the crazyflie server and lauches the crazyflie server thread
        """

        def handle_click():
            if not self.__server_running:
                # Check if there's a connection attempt
                if self.__connection_thread is not None and self.__connection_thread.isRunning():
                    message = "Can't run server while attempting to connect"
                    self.__display_message(message, "darkorange")
                    return

                # Changing connection state
                self.__widget.run_server_button.setText("Stop")
                self.__server_running = True
                self.__widget.connection_button.setEnabled(False)

                # Starting thread and connecting signals
                self.__server_thread = ServerThread(self.__available_drones)
                self.__server_thread.display_messages.connect(lambda (m, c): self.__display_message(m, c))
                self.__server_thread.update_crazyflie_status.connect(
                    lambda (i, m, c): self.__update_item_message(i, m, c))
                self.__server_thread.stop_server.connect(stop_server)

                self.__server_thread.start()
            else:
                self.__server_thread.stop()

        def stop_server():
            # Actions to be taken when stopping the server
            self.__widget.run_server_button.setText("Run")
            self.__server_running = False
            self.__widget.connection_button.setEnabled(True)

        self.__widget.run_server_button.clicked.connect(handle_click)

    def __configure_drones_list(self):
        """
        Configures the list which shows the connected drones
        """

        def handle_item_change(item, col):
            if time.time() > self.__last_click+SELECTION_DELAY:
                if self.__selected_drone == item.type():
                    self.__selected_drone = -1
                    item.setSelected(False)
                    self.__change_to_add_mode(True)
                else:
                    self.__selected_drone = item.type()
                    self.__change_to_add_mode(False)
                self.__last_click = time.time()

        self.__widget.drones_list.itemPressed.connect(handle_item_change)

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
        Kills server and open processes
        """

        if self.__server_thread is not None:
            self.__server_thread.force_stop()

        if self.__connection_thread is not None:
            self.__connection_thread.force_stop()

