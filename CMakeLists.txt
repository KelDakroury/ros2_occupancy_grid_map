cmake_minimum_required(VERSION 3.14)
project(occupancy_grid_map)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


find_package(ament_cmake_auto REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${GRID_MAP_INCLUDE_DIR}
)


ament_auto_add_executable(${PROJECT_NAME}_exe src/occupancygrid.cpp)

target_link_libraries(${PROJECT_NAME}_exe ${rclcpp_LIBRARIES} ${PCL_LIBRARIES})


install(TARGETS
${PROJECT_NAME}_exe
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
