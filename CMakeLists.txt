cmake_minimum_required(VERSION 3.8)
project(gz-to-ros-perfect-localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find Gazebo Transport
find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

# Add executable
ament_auto_add_executable(perfect_localization src/perfect_localization.cpp)
target_compile_features(perfect_localization PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(perfect_localization
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

# Install targets
install(TARGETS perfect_localization
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
