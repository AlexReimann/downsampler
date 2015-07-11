cmake_minimum_required(VERSION 2.8.3)
project(downsampler)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/downsampler.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
