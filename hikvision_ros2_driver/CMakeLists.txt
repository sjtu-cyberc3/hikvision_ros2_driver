cmake_minimum_required(VERSION 3.8)
project(hikvision_ros2_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall)

# find ros dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# find hikvision sdk
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(MVS REQUIRED)

# build components
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hikvision_ros2_driver.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PRIVATE ${MVS_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${MVS_LIBRARIES})

# register components
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "hikvision_ros2_driver::HikvisionDriver"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)