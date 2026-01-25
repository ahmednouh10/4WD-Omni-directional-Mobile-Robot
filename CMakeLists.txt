cmake_minimum_required(VERSION 3.8)
project(mecanum_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# --- NEW: Install the Python script as an executable ---
install(PROGRAMS
  move_to_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_to_pose  # Allows you to run it without the .py extension
)
# -------------------------------------------------------

# Install directories
install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()