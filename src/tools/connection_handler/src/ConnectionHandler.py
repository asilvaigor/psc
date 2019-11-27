import os
from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget, QMessageBox
from PyQt5.QtGui import QIcon
from geometry_msgs.msg import Pose

from agent.Swarm import Swarm
from representations.Constants import MIN_X, MAX_X, MIN_Y, MAX_Y, MIN_Z, MAX_Z
from representations.StablePose import StablePose


class ConnectionHandler(Plugin):
    """
    Qt Plugin to control the swarm of drones, simulated or not. It allows moving connected drones
    individually or in group to a defined goal position, pause them and shut them off.
    """

    def __init__(self, context):
        """
        Basic constructor. Has a bunch of Qt Plugin default calls,
        :param context: for Plugin constructor.
        """
        super(ConnectionHandler, self).__init__(context)
        self.__configure_plugin(context)

        self.__swarm = Swarm()
        self.__goal_pose = Pose()
        self.__is_simulated = False
        self.__simulated_drones_count = 0
        self.__is_running = False

        self.__configure_drones_combo_box()
        self.__configure_run_pause_buttons()
        self.__configure_shutdown_buttons()
        self.__configure_connected_button()
        self.__configure_lists()
        self.__configure_swap_button()
        self.__configure_line_edits()
        self.__configure_goto_button()

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

            while self.__widget.inuse_list.count() > 0:
                self.__widget.inuse_list.takeItem(0)
            while self.__widget.connected_list.count() > 0:
                self.__widget.connected_list.takeItem(0)
            self.__simulated_drones_count = 0

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
            if self.__is_running:
                self.__swarm.pause()
                self.__is_running = False

        def handle_run_button():
            if not self.__is_running:
                self.__swarm.unpause(self.__goal_pose)
                self.__is_running = True

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
                    # Find the selected drone in the lists
                    if len(self.__widget.inuse_list.selectedItems()) == 1:
                        idx = int(self.__widget.inuse_list.selectedItems()[0].text())
                        self.__swarm.shutdown_drone(idx)
                    elif len(self.__widget.connected_list.selectedItems()) == 1:
                        idx = int(self.__widget.connected_list.selectedItems()[0].text())
                        self.__swarm.shutdown_drone(idx)

                self.__widget.pause_push_button.animateClick()

            confirmation.show()

        self.__widget.shutdown_push_button.clicked.connect(handle_shutdown_button)
        self.__widget.shutdown_all_push_button.clicked.connect(lambda: handle_shutdown_button(True))

    def __configure_connected_button(self):
        """
        Configures Update/Add push button according if its in simulation or not.
        """
        def handle_connected_button():
            if self.__is_simulated:
                self.__simulated_drones_count += 1
                self.__widget.connected_list.addItem(str(self.__simulated_drones_count))
            else:
                # First read from txt
                path = os.path.realpath(
                    os.path.join(os.getcwd(), os.path.dirname(__file__)))
                path += "/../../../../launch/connected_drones.txt"
                f = open(path, "r")
                connected_drones = set()
                for s in f.read().split(' '):
                    connected_drones.add(int(s))

                # Update both lists with this function
                def update_list_widget(list_widget):
                    for row in range(list_widget.count()):
                        idx = int(list_widget.item(row).text())
                        if idx not in connected_drones:
                            list_widget.takeItem(row)
                            row -= 1
                        else:
                            connected_drones.remove(idx)

                update_list_widget(self.__widget.inuse_list)
                update_list_widget(self.__widget.connected_list)

                for idx in connected_drones:
                    self.__widget.connected_list.addItem(str(idx))

        self.__widget.connected_push_button.clicked.connect(handle_connected_button)

    def __configure_lists(self):
        """
        Configure the lists so that only one drone is selected at a time.
        """

        def handle_item_change(is_inuse_list=True):
            if is_inuse_list:
                self.__widget.connected_list.clearSelection()
            else:
                self.__widget.inuse_list.clearSelection()

        self.__widget.inuse_list.itemClicked.connect(handle_item_change)
        self.__widget.connected_list.itemClicked.connect(lambda: handle_item_change(False))

    def __configure_swap_button(self):
        """
        Configures swap button.
        """
        def handle_swap_button():
            def swap(list1, list2):
                # TODO: change behavior for simulation, make run gazebo
                if len(list1.selectedIndexes()) == 1:
                    row = int(list1.selectedIndexes()[0].row())
                    idx = int(list1.selectedItems()[0].text())
                    list1.takeItem(row)
                    list2.addItem(str(idx))
                    list1.clearSelection()
                    if list1 == self.__widget.inuse_list:
                        self.__swarm.remove_drone(idx)
                    else:
                        self.__swarm.add_drone(idx)
                    return True
                return False

            swap(self.__widget.connected_list, self.__widget.inuse_list)
            swap(self.__widget.inuse_list, self.__widget.connected_list)

        self.__widget.swap_push_button.clicked.connect(handle_swap_button)

    def __configure_line_edits(self):
        """
        Synchronizes the LineEdits with the label on the bottom.
        """

        def update_coordinates_label():
            s = "Goal: ("
            s += str(self.__goal_pose.position.x) + ", "
            s += str(self.__goal_pose.position.y) + ", "
            s += str(self.__goal_pose.position.z) + ", "
            s += str(StablePose.from_ros(self.__goal_pose).yaw) + ")"
            self.__widget.coordinates_label.setText(s)

        def change_text(coordinate, min_v, max_v):
            """
            Analyzes if the string is double, its size and range.
            :param coordinate: 0:x, 1:y, 2:z, 3:yaw.
            :param min_v: Minimum value for the range.
            :param max_v: Maximum value for the range.
            """
            if coordinate == 0:
                s = self.__widget.x_line_edit.text()
            elif coordinate == 1:
                s = self.__widget.y_line_edit.text()
            elif coordinate == 2:
                s = self.__widget.z_line_edit.text()
            else:
                s = self.__widget.yaw_line_edit.text()

            try:
                if s == '' or s == '-':
                    return
                if len(s) > 5:
                    raise TypeError

                val = float(s)
                raise_error = False
                if val < min_v:
                    val = min_v
                    raise_error = True
                elif val > max_v:
                    val = max_v
                    raise_error = True

                if coordinate == 0:
                    self.__goal_pose.position.x = val
                elif coordinate == 1:
                    self.__goal_pose.position.y = val
                elif coordinate == 2:
                    self.__goal_pose.position.z = val
                else:
                    self.__goal_pose = StablePose.to_ros(
                        StablePose(self.__goal_pose.position, val))

                if raise_error:
                    raise ValueError

            except (TypeError, ValueError):
                self.__widget.x_line_edit.setText(str(self.__goal_pose.position.x))

            update_coordinates_label()

        self.__widget.x_line_edit.textChanged.connect(lambda: change_text(0, MIN_X, MAX_X))
        self.__widget.y_line_edit.textChanged.connect(lambda: change_text(1, MIN_Y, MAX_Y))
        self.__widget.z_line_edit.textChanged.connect(lambda: change_text(2, MIN_Z, MAX_Z))
        self.__widget.yaw_line_edit.textChanged.connect(lambda: change_text(3, -180, 180))

    def __configure_goto_button(self):
        """
        Sets goto button functionality.
        """

        def handle_goto_button():
            if not self.__is_running:
                if len(self.__widget.inuse_list.selectedItems()) == 1:
                    idx = int(self.__widget.inuse_list.selectedItems()[0].text())
                    self.__swarm.goto_drone(idx, self.__goal_pose)

        self.__widget.goto_push_button.clicked.connect(handle_goto_button)

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
