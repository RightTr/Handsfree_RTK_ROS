cmake_minimum_required(VERSION 3.16)

project(handsfree_rtk)

option(BUILD_CPP_DEMOS "Build the C++ demo executables" ON)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
  set(USE_ROS2 ON)
elseif(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
  set(USE_ROS1 ON)
elseif(ament_cmake_FOUND)
  message(STATUS "ROS2 detected")
  set(USE_ROS2 ON)
elseif(catkin_FOUND)
  message(STATUS "ROS1 detected")
  set(USE_ROS1 ON)
else()
  set(USE_ROS1 ON)
endif()

if(USE_ROS1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    nav_msgs
    sensor_msgs
    std_msgs
  )

  catkin_package()
endif()

if(USE_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
endif()

function(configure_cpp_demo target source_file)
  add_executable(${target} ${source_file})
  target_compile_features(${target} PRIVATE cxx_std_11)
  target_link_libraries(${target} Boost::system Threads::Threads)

  if(USE_ROS1)
    target_compile_definitions(${target} PRIVATE USE_ROS1 ROS_EDITION=\"ROS1\")
    target_include_directories(${target} PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(${target} ${catkin_LIBRARIES})
  else()
    target_compile_definitions(${target} PRIVATE USE_ROS2 ROS_EDITION=\"ROS2\")
    ament_target_dependencies(${target}
      rclcpp
      nav_msgs
      sensor_msgs
      std_msgs
    )
  endif()
endfunction()

if(BUILD_CPP_DEMOS)
  configure_cpp_demo(cpp_driver_um98x demo/ros/ros_driver_um98x.cpp)
  configure_cpp_demo(cpp_driver_um98x_ntrip demo/ros/ros_driver_um98x_ntrip.cpp)
endif()

if(USE_ROS2)
  install(PROGRAMS
    scripts/ros_driver_um98x.py
    scripts/ros_driver_um98x_ntrip.py
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY
    launch/
    usb_rules/
    DESTINATION share/${PROJECT_NAME}
  )

  if(TARGET cpp_driver_um98x AND TARGET cpp_driver_um98x_ntrip)
    install(TARGETS
      cpp_driver_um98x
      cpp_driver_um98x_ntrip
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()

  ament_package()
endif()
