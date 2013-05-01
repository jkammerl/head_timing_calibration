cmake_minimum_required(VERSION 2.8.3)
project(head_time_calibration_tools)

find_package(catkin REQUIRED COMPONENTS geometry_msgs
                                        roscpp
                                        tf
                                        tf2_ros
                                        tf_conversions
                                        sensor_msgs
                                        image_transport
                                        pointcloud_floor_filter
                                        
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS geometry_msgs
                   roscpp
                   tf
                   tf2_ros
                   tf_conversions
                   sensor_msgs
                   image_transport
                   pointcloud_floor_filter
                   
  )
                                        

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIRS} 
                    ${Eigen_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS}
                 ${Boost_LIBRARY_DIRS}
                 ${Eigen_LIBRARY_DIRS})

add_library(head_time_calibration_lib src/head_tilting.cpp
                                      src/head_time_calibration_lib.cpp
)
target_link_libraries(head_time_calibration_lib ${catkin_LIBRARIES}
                                                ${Boost_LIBRARIES}
                                                ${Eigen_LIBRARIES})

add_executable(head_time_calibration ros/head_time_calibration.cpp)
target_link_libraries(head_time_calibration head_time_calibration_lib ${Boost_LIBRARY_DIRS} ${catkin_LIBRARIES})

install(TARGETS head_time_calibration_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

