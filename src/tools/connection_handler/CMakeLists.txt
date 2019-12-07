find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_cpp
)

install(PROGRAMS
  scripts/connection_handler
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
