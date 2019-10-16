from PySide2.QtWidgets import QApplication, QWidget, QLabel


class SwarmController:
    def __init__(self, drones):
        self.__drones = drones
        self.__app = QApplication([])
        self.__widget = QWidget()
        self.__is_paused = True
        self.__widget.setWindowTitle("Swarm Controller")
        self.__widget.show()

    def execute(self):
        self.__app.exec_()

    def is_paused(self):
        return self.__is_paused

    def get_goal(self):
        pass
