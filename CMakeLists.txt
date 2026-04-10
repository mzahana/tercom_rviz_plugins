cmake_minimum_required(VERSION 3.8)
project(tercom_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Charts)

set(SOURCES
  src/sparkline_widget.cpp
  src/filter_status_panel.cpp
  src/tercom_quality_panel.cpp
  src/error_history_panel.cpp
  src/profiling_panel.cpp
)

set(HEADERS
  include/tercom_rviz_plugins/sparkline_widget.hpp
  include/tercom_rviz_plugins/filter_status_panel.hpp
  include/tercom_rviz_plugins/tercom_quality_panel.hpp
  include/tercom_rviz_plugins/error_history_panel.hpp
  include/tercom_rviz_plugins/profiling_panel.hpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Charts
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rviz_common std_msgs std_srvs geometry_msgs sensor_msgs pluginlib Qt5)

ament_package()
