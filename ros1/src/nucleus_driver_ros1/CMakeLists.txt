cmake_minimum_required(VERSION 3.0.2)

project(nucleus_driver_ros1)
# find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
)
find_package(genmsg REQUIRED)

file(GLOB MSG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/**/*.msg)
file(GLOB SRV_FILES ${PROJECT_SOURCE_DIR}/**/*.srv)

foreach(I IN ITEMS ${MSG_FILES})
  get_filename_component(ITEM_NAME ${I} NAME)
  add_message_files(FILES ${ITEM_NAME})
endforeach()

foreach(I IN ITEMS ${SRV_FILES})
  get_filename_component(ITEM_NAME ${I} NAME)
  add_service_files(FILES ${ITEM_NAME})
endforeach()

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    # geometry_msgs
    # sensor_msgs
    # std_msgs
    # nav_msgs
)


catkin_package(
  CATKIN_DEPENDS
    # nav_msgs
    # sensor_msgs
    # std_msgs
    # geometry_msgs
)

catkin_install_python(PROGRAMS nucleus_node/nucleus_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)