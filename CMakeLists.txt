cmake_minimum_required(VERSION 3.0.2)

project(handsfree_rtk)

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  install(PROGRAMS
    scripts/ros_driver_um98x.py
    scripts/ros_driver_um98x_ntrip.py
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY
    launch
    usb_rules
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()
else()
  find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    std_msgs
  )

  catkin_package(
  )

  include_directories(${catkin_INCLUDE_DIRS})
endif()
