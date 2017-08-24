cmake_minimum_required(VERSION 2.8.3)
project(lms1xx)

add_compile_options(-std=c++11)

# Build ROS-independent library.
find_package(console_bridge REQUIRED)
include_directories(include ${console_bridge_INCLUDE_DIRS})

# CoLaA Library abstracting protocol and implementing LMS1xx communication
add_library(CoLaA src/colaa.cpp src/parse_helpers.cpp)
target_link_libraries(CoLaA ${console_bridge_LIBRARIES})

# Specialisations for LMS5xx series scanners
add_library(LMS5xx src/lms5xx.cpp)
target_link_libraries(LMS5xx CoLaA ${console_bridge_LIBRARIES})

add_library(MRS1000 src/mrs1000.cpp)
target_link_libraries(MRS1000 CoLaA ${console_bridge_LIBRARIES})


# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(LMS1xx_node src/lms1xx_node.cpp)
target_link_libraries(LMS1xx_node CoLaA ${catkin_LIBRARIES})
add_dependencies(LMS1xx_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(MRS1000_node src/mrs1000_node.cpp src/colaa_conversion.cpp)
target_link_libraries(MRS1000_node MRS1000 ${catkin_LIBRARIES})
add_dependencies(MRS1000_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(LMS5xx_node src/lms5xx_node.cpp src/colaa_conversion.cpp)
target_link_libraries(LMS5xx_node LMS5xx ${catkin_LIBRARIES})
add_dependencies(LMS5xx_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS CoLaA LMS5xx MRS1000 LMS1xx_node LMS5xx_node MRS1000_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY meshes launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/find_sick scripts/set_sick_ip
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_buffer test/test_buffer.cpp)
  target_link_libraries(test_buffer ${catkin_LIBRARIES})

  catkin_add_gtest(test_colaa test/test_colaa.cpp)
  target_link_libraries(test_colaa CoLaA ${catkin_LIBRARIES})
  add_dependencies(test_colaa CoLaA)

  # Copy test data
  file(COPY test/mrs1000.txt DESTINATION ${PROJECT_BINARY_DIR}/test)
  catkin_add_gtest(test_mrs1000_scandata test/mrs1000_scandata_test.cpp)
  target_link_libraries(test_mrs1000_scandata CoLaA ${catkin_LIBRARIES})
  add_dependencies(test_mrs1000_scandata CoLaA)

  catkin_add_gtest(parse_helper_test test/parse_helper_test.cpp src/parse_helpers.cpp)

  find_package(roslint REQUIRED)
  roslint_cpp()
  #roslint_add_test()

  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/LMS1xx.launch)
  roslaunch_add_file_check(launch/LMS5xx.launch)
  roslaunch_add_file_check(launch/MRS1000.launch)
endif()

