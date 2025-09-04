cmake_minimum_required(VERSION 3.8)
project(gz-to-ros-perfect-localization)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find Gazebo Transport
find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

# perfect localization library
ament_auto_add_library(gz_to_ros_perfect_localization ./src/perfect_localization.cpp)
target_compile_features(gz_to_ros_perfect_localization PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(gz_to_ros_perfect_localization PUBLIC include)

# perfect localization node
ament_auto_add_executable(perfect_localization ./src/perfect_localization_node.cpp)
target_include_directories(perfect_localization PUBLIC include)
target_link_libraries(perfect_localization gz_to_ros_perfect_localization)

# perfect localization rviz panel
set(RVIZ_LIBRARY_NAME perfect_localization_rviz_panel)

# Generate header for ui
qt5_wrap_ui(${RVIZ_LIBRARY_NAME}_UI_HEADER ./resource/panel.ui)

ament_auto_add_library(${RVIZ_LIBRARY_NAME} SHARED
  ${${RVIZ_LIBRARY_NAME}_UI_HEADER}
  ./src/tf_calibrator.cpp
  ./include/gz-to-ros-perfect-localization/tf_calibrator.hpp
)
target_compile_definitions(${RVIZ_LIBRARY_NAME} PRIVATE QT_NO_KEYWORDS)
target_link_libraries(${RVIZ_LIBRARY_NAME} gz_to_ros_perfect_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(gz_to_ros_perfect_localization PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(perfect_localization PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(${RVIZ_LIBRARY_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Export the plugin to be imported by rviz2¶
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install targets
install(TARGETS perfect_localization
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${RVIZ_LIBRARY_NAME}
  EXPORT ${RVIZ_LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)
