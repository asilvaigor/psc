import math
import os
import signal
import subprocess
import threading
from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget, QMessageBox
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt

from agent.Swarm import Swarm
from representations.Constants import OBSTACLE_MARGIN
from representations.Constants import MIN_X, MAX_X, MIN_Y, MAX_Y, MIN_Z, MAX_Z
from representations.StablePose import StablePose


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
        self.__goal_poses = {0: StablePose()}
        self.__swarm.set_goal_poses(self.__goal_poses)
        self.__is_simulated = False
        self.__simulated_drones_count = 0
        self.__is_running = False

        self.__simulated_server_processes = []
        self.__gazebo_process = None
        self.__kill_simulation()

        self.__configure_drones_combo_box()
        self.__configure_run_pause_buttons()
        self.__configure_shutdown_buttons()
        self.__configure_connected_button()
        self.__configure_run_gazebo_button()
        self.__configure_lists()
        self.__configure_swap_button()
        self.__configure_edits()
        self.__configure_goto_button()

        self.__swarm.run_thread()
        context.add_widget(self.__widget)

    def shutdown_plugin(self):
        """
        Stops processes.
        """
        self.__kill_simulation()
        self.__swarm.stop_thread()

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
                self.__widget.swap_push_button.setEnabled(True)
                self.__widget.run_gazebo_push_button.setEnabled(False)
                self.__widget.run_gazebo_push_button.setText("Run Gazebo")

                self.__kill_simulation()
            else:
                self.__is_simulated = True
                self.__widget.connected_push_button.setText("Add")
                icon = QIcon()
                self.__widget.connected_push_button.setIcon(icon.fromTheme("list-add"))
                self.__widget.connected_push_button.setToolTip(
                    "Adds a new drone to be simulated.")

                self.__widget.swap_push_button.setEnabled(False)
                self.__widget.run_gazebo_push_button.setEnabled(True)

            self.__widget.connected_push_button.setEnabled(True)
            self.__widget.connected_push_button.animateClick()
            self.__clear_drones()
            self.__widget.pause_push_button.animateClick()
            # Restarting swarm to make sure
            self.__swarm.stop_thread()
            self.__swarm = Swarm()
            self.__goal_poses = {0: StablePose()}
            self.__swarm.set_goal_poses(self.__goal_poses)
            self.__swarm.run_thread()

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
                t = threading.Thread(target=self.__swarm.unpause())
                t.start()
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
                if self.__simulated_drones_count == 5:
                    return
                self.__simulated_drones_count += 1
                self.__widget.connected_list.addItem(str(self.__simulated_drones_count))

                # Opens cf2 file
                file_path = os.path.realpath(
                    os.path.join(os.getcwd(), os.path.dirname(__file__)))
                path = file_path + "/../../../../../sim_cf/crazyflie-firmware/sitl_make/build/cf2"
                self.__simulated_server_processes.append(subprocess.Popen(
                    [path, str(self.__simulated_drones_count)]))
            else:
                # First read from txt
                file_path = os.path.realpath(
                    os.path.join(os.getcwd(), os.path.dirname(__file__)))
                path = file_path + "/../../../../launch/connected_drones.txt"
                f = open(path, "r")
                connected_drones = set()
                for s in f.read().split(' '):
                    if len(s) > 0:
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

    def __configure_run_gazebo_button(self):
        """
        Runs the simulated crazyflie in cf2 file, and runs gazebo according to the number of drones
        used.
        """
        def handle_run_gazebo_button():
            if self.__gazebo_process:
                self.__kill_simulation()
                self.__widget.connected_push_button.setEnabled(True)
                self.__widget.swap_push_button.setEnabled(False)
                self.__widget.run_gazebo_push_button.setText("Run Gazebo")
                self.__clear_drones()
            else:
                file_path = os.path.realpath(
                    os.path.join(os.getcwd(), os.path.dirname(__file__)))
                path = file_path + "/../../../../launch/run_simulated.launch"

                self.__gazebo_process = subprocess.Popen(
                    ["roslaunch", path, "nbQuads:=" + str(self.__simulated_drones_count)])

                self.__widget.connected_push_button.setEnabled(False)
                self.__widget.swap_push_button.setEnabled(True)
                self.__widget.run_gazebo_push_button.setText("Kill Gazebo")

        self.__widget.run_gazebo_push_button.clicked.connect(handle_run_gazebo_button)
        self.__widget.run_gazebo_push_button.setEnabled(False)

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
                if len(list1.selectedIndexes()) == 1:
                    row = int(list1.selectedIndexes()[0].row())
                    idx = int(list1.selectedItems()[0].text())

                    if list1 == self.__widget.inuse_list:
                        self.__swarm.remove_drone(idx)
                    else:
                        # Using a timeout to try to add drone
                        def handler(signum, frame):
                            raise Exception("timeout")

                        try:
                            signal.signal(signal.SIGALRM, handler)
                            signal.alarm(1)
                            self.__widget.setCursor(Qt.WaitCursor)
                            self.__swarm.add_drone(idx)
                            self.__widget.setCursor(Qt.ArrowCursor)
                            signal.alarm(0)
                        except Exception:
                            self.__widget.setCursor(Qt.ArrowCursor)
                            return False

                    list1.takeItem(row)
                    list2.addItem(str(idx))
                    list1.clearSelection()
                    self.__configure_goal_poses()

                    return True
                return False

            swap(self.__widget.connected_list, self.__widget.inuse_list)
            swap(self.__widget.inuse_list, self.__widget.connected_list)

        self.__widget.swap_push_button.clicked.connect(handle_swap_button)

    def __configure_goal_poses(self):
        """
        Sets goal poses for each drone, placing them in a line centered on the pose given and
        separating drones with a fixed distance.
        """

        n_drones = len(self.__swarm.drones)
        fixed_dist = 2 * OBSTACLE_MARGIN
        center = self.__goal_poses[0]

        # Finding points for poses
        poses = []
        is_odd = n_drones % 2 == 1
        k = n_drones // 2
        for i in range(k):
            x = center.x - (i + 0.5 * (1 + is_odd)) * fixed_dist * math.sin(center.yaw)
            y = center.y + (i + 0.5 * (1 + is_odd)) * fixed_dist * math.cos(center.yaw)
            poses.append(StablePose(x, y, center.z, center.yaw))
        if is_odd:
            poses.append(center)
        for i in range(k):
            x = center.x + (i + 0.5 * (1 + is_odd)) * fixed_dist * math.sin(center.yaw)
            y = center.y - (i + 0.5 * (1 + is_odd)) * fixed_dist * math.cos(center.yaw)
            poses.append(StablePose(x, y, center.z, center.yaw))

        # Associating each point with a drone
        # TODO: Maximum matching problem, maybe pair drones closest to points to optimize it
        i = 0
        for drone_id in self.__swarm.drones:
            self.__goal_poses[drone_id] = poses[i]
            i += 1

        self.__swarm.set_goal_poses(self.__goal_poses)

    def __configure_edits(self):
        """
        Synchronizes the SpinBoxes for x, y, z and yaw with the label on the bottom.
        """

        def synchronize_labels():
            """
            Changes variables and synchronizes spinboxes with label.
            """
            self.__goal_poses[0].x = self.__widget.x_edit.value()
            self.__goal_poses[0].y = self.__widget.y_edit.value()
            self.__goal_poses[0].z = self.__widget.z_edit.value()
            self.__goal_poses[0].yaw = math.radians(self.__widget.yaw_edit.value())

            s = "Goal: ("
            s += str(self.__goal_poses[0].x) + ", "
            s += str(self.__goal_poses[0].y) + ", "
            s += str(self.__goal_poses[0].z) + ", "
            # Putting center pose in index 0
            s += str(int(math.degrees(self.__goal_poses[0].yaw))) + ")"
            self.__widget.coordinates_label.setText(s)

            self.__configure_goal_poses()

        self.__widget.x_edit.setSingleStep(0.2)
        self.__widget.x_edit.setMinimum(MIN_X)
        self.__widget.x_edit.setMaximum(MAX_X)
        self.__widget.x_edit.valueChanged.connect(synchronize_labels)

        self.__widget.y_edit.setSingleStep(0.2)
        self.__widget.y_edit.setMinimum(MIN_Y)
        self.__widget.y_edit.setMaximum(MAX_Y)
        self.__widget.y_edit.valueChanged.connect(synchronize_labels)

        self.__widget.z_edit.setSingleStep(0.2)
        self.__widget.z_edit.setMinimum(MIN_Z)
        self.__widget.z_edit.setMaximum(MAX_Z)
        self.__widget.z_edit.valueChanged.connect(synchronize_labels)

        self.__widget.yaw_edit.setSingleStep(30)
        self.__widget.yaw_edit.setMinimum(-180)
        self.__widget.yaw_edit.setMaximum(180)
        self.__widget.yaw_edit.valueChanged.connect(synchronize_labels)

    def __configure_goto_button(self):
        """
        Sets goto button functionality.
        """
        def handle_goto_button():
            if not self.__is_running:
                if len(self.__widget.inuse_list.selectedItems()) == 1:
                    idx = int(self.__widget.inuse_list.selectedItems()[0].text())
                    self.__swarm.goto_drone(idx, self.__goal_poses[0])

        self.__widget.goto_push_button.clicked.connect(handle_goto_button)

    def __kill_simulation(self):
        """
        Kills gazebo and cf2 processes if they exist.
        """
        while self.__simulated_server_processes:
            p = self.__simulated_server_processes.pop()
            p.kill()

        subprocess.call(['killall', '-9', 'gzclient'])
        subprocess.call(['killall', '-9', 'gzserver'])
        self.__gazebo_process = None
        subprocess.call(['killall', '-9', 'cf2'])

    def __clear_drones(self):
        while self.__widget.inuse_list.count() > 0:
            self.__widget.inuse_list.takeItem(0)
        while self.__widget.connected_list.count() > 0:
            self.__widget.connected_list.takeItem(0)
        self.__simulated_drones_count = 0

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
