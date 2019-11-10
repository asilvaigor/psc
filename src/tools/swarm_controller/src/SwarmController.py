import os
from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget, QMessageBox
from PyQt5.QtGui import QIcon
from geometry_msgs.msg import Pose

from agent.Swarm import Swarm
from representations.Constants import PAUSED, RUNNING


class SwarmController(Plugin):
    """
    Qt Plugin to control the swarm of drones, simulated or not. It allows moving connected drones
    individually or in group to a defined goal position, pause them and shut them off.
    """

    def __init__(self, context):
        """
        Basic constructor. Has a bunch of Qt Plugin default calls,
        :param context: for Plugin constructor.
        """
        super(SwarmController, self).__init__(context)
        self.__configure_plugin(context)

        self.__swarm = Swarm()
        self.__is_simulated = False
        self.__selected_drone_id = 0
        self.__goal_pose = Pose()
        self.__state = PAUSED

        self.__configure_drones_combo_box()
        self.__configure_run_pause_buttons()
        self.__configure_shutdown_buttons()
        self.__configure_connected_button()

        context.add_widget(self.__widget)

    def __configure_drones_combo_box(self):
        """
        Configures the combo box at the top of the plugin, which selects if real drones or simulated
        will be used.
        """

        def handle_index_change(idx):
            if idx == 0:
                self.__is_simulated = False
                self.__widget.connected_push_button.setText("Update")
                icon = QIcon()
                self.__widget.connected_push_button.setIcon(icon.fromTheme("view-refresh"))
                self.__widget.connected_push_button.setToolTip(
                    "Update the status of connection of the drones.")
                self.__widget.connected_push_button.animateClick()
            else:
                self.__is_simulated = True
                self.__widget.connected_push_button.setText("Add")
                icon = QIcon()
                self.__widget.connected_push_button.setIcon(icon.fromTheme("list-add"))
                self.__widget.connected_push_button.setToolTip(
                    "Adds a new drone to be simulated.")
                self.__widget.connected_push_button.animateClick()

        self.__widget.drones_combo_box.currentIndexChanged.connect(handle_index_change)

    def __configure_run_pause_buttons(self):
        """
        Sets the Run and Pause push button behaviors like radio buttons, and sets their handlers.
        """
        self.__widget.pause_push_button.setCheckable(True)
        self.__widget.pause_push_button.setAutoExclusive(True)
        self.__widget.run_push_button.setCheckable(True)
        self.__widget.run_push_button.setAutoExclusive(True)
        self.__widget.pause_push_button.animateClick()

        def handle_pause_button():
            if self.__state == RUNNING:
                self.__swarm.pause()
                self.__state = PAUSED

        def handle_run_button():
            if self.__state == PAUSED:
                self.__swarm.unpause(self.__goal_pose)
                self.__state = RUNNING

        self.__widget.pause_push_button.clicked.connect(handle_pause_button)
        self.__widget.run_push_button.clicked.connect(handle_run_button)

    def __configure_shutdown_buttons(self):
        """
        Configures Shutdown All and Shutdown push buttons, which include a confirmation window.
        """
        def handle_shutdown_button(is_all=False):
            confirmation = QMessageBox()
            reply = confirmation.question(confirmation, "Confirmation", "Are you sure?",
                                          QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
            if reply == QMessageBox.Yes:
                if is_all:
                    self.__swarm.shutdown_drone()
                else:
                    self.__swarm.shutdown_drone(self.__selected_drone_id)
                self.__widget.pause_push_button.animateClick()

            confirmation.show()

        self.__widget.shutdown_push_button.clicked.connect(handle_shutdown_button)
        self.__widget.shutdown_all_push_button.clicked.connect(lambda: handle_shutdown_button(True))

    def __configure_connected_button(self):
        """
        Configures Update/Add push button according if its in simulation or not.
        """
        def handle_connected_button():
            # TODO
            if self.__is_simulated:
                self.__widget.connected_list.addItem()
            else:
                pass

        self.__widget.connected_push_button.clicked.connect(handle_connected_button)

    def __configure_plugin(self, context):
        """
        Default calls to correctly set the plugin in rqt.
        :param context: for Plugin constructor
        """
        self.setObjectName('SwarmController')

        # Handles arguments
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())

        # Initiates widget
        self.__widget = QWidget()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'SwarmController.ui')
        loadUi(ui_file, self.__widget)
        self.__widget.setObjectName('SwarmControllerUi')
        if context.serial_number() > 1:
            self.__widget.setWindowTitle(self.__widget.windowTitle() +
                                         (' (%d)' % context.serial_number()))
