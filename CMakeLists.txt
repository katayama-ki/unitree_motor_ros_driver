cmake_minimum_required(VERSION 3.10)
project(unitree_motor_ros_driver)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)
find_package(Threads REQUIRED)

add_message_files(FILES MotorCommand.msg MotorState.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(INCLUDE_DIRS include LIBRARIES unitree_driver_core unitree_driver_sdk
               CATKIN_DEPENDS roscpp std_msgs message_runtime)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(SDK_LIBRARY_NAME libUnitreeMotorSDK_Linux64.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(SDK_LIBRARY_NAME libUnitreeMotorSDK_Arm64.so)
else()
  message(FATAL_ERROR "Bundled Unitree SDK supports only Linux x86_64 and aarch64")
endif()
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "Bundled Unitree SDK requires Linux")
endif()
set(SDK_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/unitree_actuator_sdk/lib/${SDK_LIBRARY_NAME}")
if(NOT EXISTS "${SDK_LIBRARY}")
  message(FATAL_ERROR "Missing bundled SDK: ${SDK_LIBRARY}")
endif()
add_library(unitree_vendor_sdk SHARED IMPORTED)
set_target_properties(unitree_vendor_sdk PROPERTIES
  IMPORTED_LOCATION "${SDK_LIBRARY}" IMPORTED_NO_SONAME TRUE)

add_library(unitree_driver_core src/model.cpp src/bus_driver.cpp)
target_include_directories(unitree_driver_core PUBLIC include)
target_link_libraries(unitree_driver_core PUBLIC Threads::Threads)

add_library(unitree_driver_sdk src/sdk_transport.cpp)
target_include_directories(unitree_driver_sdk PUBLIC include)
target_include_directories(unitree_driver_sdk SYSTEM PRIVATE unitree_actuator_sdk/include)
target_link_libraries(unitree_driver_sdk PUBLIC unitree_driver_core PRIVATE unitree_vendor_sdk)

add_executable(unitree_motor_driver_node src/ros_node.cpp)
target_include_directories(unitree_motor_driver_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(unitree_motor_driver_node unitree_driver_sdk ${catkin_LIBRARIES})
add_dependencies(unitree_motor_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Preserve a relocatable install tree and let catkin/devel resolve the bundled SDK.
set_target_properties(unitree_driver_sdk unitree_motor_driver_node PROPERTIES
  INSTALL_RPATH "\$ORIGIN;\$ORIGIN/..")
install(TARGETS unitree_driver_core unitree_driver_sdk unitree_motor_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES "${SDK_LIBRARY}" DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES unitree_actuator_sdk/LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/unitree_actuator_sdk)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS test/ros_pty_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_library(pty_serial_shim SHARED test/pty_serial_shim.c)
  add_rostest(test/ros_pty.test DEPENDENCIES unitree_motor_driver_node pty_serial_shim)
  catkin_add_gtest(driver_core_test test/driver_core_test.cpp test/test_main.cpp)
  if(TARGET driver_core_test)
    target_link_libraries(driver_core_test unitree_driver_core)
  endif()
  catkin_add_gtest(sdk_protocol_test test/sdk_protocol_test.cpp test/test_main.cpp)
  if(TARGET sdk_protocol_test)
    target_include_directories(sdk_protocol_test SYSTEM PRIVATE unitree_actuator_sdk/include)
    target_link_libraries(sdk_protocol_test unitree_driver_core unitree_vendor_sdk)
  endif()
endif()
