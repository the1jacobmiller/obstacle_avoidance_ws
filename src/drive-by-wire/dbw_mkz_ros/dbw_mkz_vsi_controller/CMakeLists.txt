cmake_minimum_required(VERSION 2.8.3)
project(dbw_mkz_vsi_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  dbw_mkz_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_vsi_controller src/controller_main.cpp src/Controller.cpp)
add_dependencies(${PROJECT_NAME}_vsi_controller dbw_mkz_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}_vsi_controller
  ${catkin_LIBRARIES}
)

add_executable(hmi_controller 
	src/hmi_controller.cpp 
	include/cf_hmi_include/show_packet.c
	include/cf_hmi_include/cf_packet.c
	include/cf_hmi_include/serial.c
)
add_dependencies(hmi_controller dbw_mkz_msgs_gencpp)
target_link_libraries(hmi_controller ${catkin_LIBRARIES} )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set_target_properties(${PROJECT_NAME}_vsi_controller PROPERTIES OUTPUT_NAME vsi_controller PREFIX "")

install(TARGETS ${PROJECT_NAME}_vsi_controller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
