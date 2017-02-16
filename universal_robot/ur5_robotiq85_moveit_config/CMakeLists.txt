cmake_minimum_required(VERSION 2.8.3)
project(ur5_robotiq85_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# catkin_python_setup()

## Specify libraries to link a library or executable target against
# target_link_libraries(ur_gazebo_node
#   ${catkin_LIBRARIES}
# )

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_gazebo
#  CATKIN_DEPENDS ur_description
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(ur_gazebo
#   src/${PROJECT_NAME}/ur_gazebo.cpp
# )

## Declare a cpp executable
# add_executable(ur_gazebo_node src/ur_gazebo_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ur_gazebo_node ur_gazebo_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ur_gazebo_node
#   ${catkin_LIBRARIES}
# )
