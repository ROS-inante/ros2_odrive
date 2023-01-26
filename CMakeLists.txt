cmake_minimum_required(VERSION 3.8)
project(ros2_odrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ros2_odrive_interfaces REQUIRED)

find_path(LIBUSB_INCLUDE_DIR
  NAMES libusb.h
  PATH_SUFFIXES "include" "libusb" "libusb-1.0")
find_library(LIBUSB_LIBRARY
  NAMES usb
  PATH_SUFFIXES "lib" "lib32" "lib64")





#include_directories(include ${LIBUSB_INCLUDE_DIR})
include_directories(include )

add_executable(odrive src/odrive_utils.cpp src/odrive_endpoint.cpp src/odrive.cpp)
ament_target_dependencies(odrive rclcpp ros2_odrive_interfaces)
target_link_libraries(odrive usb-1.0 jsoncpp)

add_library(
  libodrive 
  SHARED
  src/odrive_endpoint.cpp src/odrive_utils.cpp 
)


ament_target_dependencies(libodrive rclcpp)
ament_export_targets(libodrive HAS_LIBRARY_TARGET)
target_link_libraries(libodrive usb-1.0 jsoncpp)

ament_export_include_directories(include)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  TARGETS odrive
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS libodrive
  EXPORT libodrive
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)




ament_package()
