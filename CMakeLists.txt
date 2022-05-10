cmake_minimum_required(VERSION 3.5)
project(gazebo_plugins_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(BASE_LIBS  
  ament_cmake 
  gazebo_dev
  gazebo_ros
  geometry_msgs 
  rclcpp
  std_msgs
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)
link_directories(${GAZEBO_LIBRARY_DIRS})

set(ROS_PLUGINS_LIST "")

add_library(example_model SHARED src/example_model.cc)
ament_target_dependencies(
  example_model
  ${BASE_LIBS}
)
list(APPEND ROS_PLUGINS_LIST example_model)

add_library(example_world SHARED src/example_world.cc)
ament_target_dependencies(
  example_world
  ${BASE_LIBS}
)
list(APPEND ROS_PLUGINS_LIST example_world)


set(EXPORT_MY_LIBS "")
foreach(VAL ${ROS_PLUGINS_LIST})
  list(APPEND EXPORT_MY_LIBS "export_${VAL}")
endforeach()

ament_export_targets(${EXPORT_MY_LIBS})#${EXPORT_MY_LIBS} HAS_LIBRARY_TARGET)
ament_export_dependencies(${BASE_LIBS})
ament_export_include_directories(include)
ament_export_libraries(${ROS_PLUGINS_LIST})

foreach(VAL ${ROS_PLUGINS_LIST})
  install(TARGETS ${VAL}
    EXPORT export_${VAL}#${EXPORT_MY_LIBS}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    INCLUDES DESTINATION include
  )

  install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*~"
  EXCLUDE)
endforeach()

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

ament_package()
