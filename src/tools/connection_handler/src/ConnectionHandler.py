import os
from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget, QMessageBox


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

        self.__configure_connection_button()

        context.add_widget(self.__widget)

    def __configure_connection_button(self):
        """
        Configures connection button
        """
        def handle_connection_button():
            self.__widget.connection_button.setText("<b>ola</b>")

        self.__widget.connection_button.clicked.connect(handle_connection_button)

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
