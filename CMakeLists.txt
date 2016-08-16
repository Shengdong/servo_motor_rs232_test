cmake_minimum_required (VERSION 2.6)
project(kinco_motor_rs232)


include_directories(${catkin_INCLUDE_DIRS}
                    /usr/local/include
                    )

find_package(catkin REQUIRED COMPONENTS
             cmake_modules
             roscpp
             std_msgs
             geometry_msgs
            )


catkin_package(
  LIBRARIES kincomotor_rs232
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(kinco_motor_rs232_node src/kinco_motor_rs232.cpp)
target_link_libraries(kinco_motor_rs232_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS kinco_motor_rs232_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/kinco_motor_rs232
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


