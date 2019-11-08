#!/usr/bin/env python

import sys

from tools.swarm_controller.src.SwarmController import SwarmController
from rqt_gui.main import Main


main = Main(filename='SwarmController')
sys.exit(main.main(standalone='SwarmController'))
