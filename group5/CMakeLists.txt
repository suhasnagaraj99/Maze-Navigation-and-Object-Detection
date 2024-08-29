cmake_minimum_required(VERSION 3.8)
project(group5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mage_msgs REQUIRED)


include_directories(include/group5)

add_executable(tbot src/tbot.cpp src/utils.cpp)
ament_target_dependencies(tbot rclcpp rclcpp ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs mage_msgs)

add_executable(aruco_broadcaster src/aruco_broadcaster.cpp)
ament_target_dependencies(aruco_broadcaster rclcpp ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs)

add_executable(battery_broadcaster src/battery_broadcaster.cpp)
ament_target_dependencies(battery_broadcaster rclcpp ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs mage_msgs)

add_executable(battery_listner src/battery_listner.cpp src/utils.cpp) 
ament_target_dependencies(battery_listner rclcpp ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs mage_msgs)


install(TARGETS 
  aruco_broadcaster
  battery_broadcaster
  battery_listner  
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  tbot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include config launch DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
