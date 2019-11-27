#!/usr/bin/env python

import sys

from tools.connection_handler.src.ConnectionHandler import ConnectionHandler
from rqt_gui.main import Main


main = Main(filename='ConnectionHandler')
sys.exit(main.main(standalone='ConnectionHandler'))
