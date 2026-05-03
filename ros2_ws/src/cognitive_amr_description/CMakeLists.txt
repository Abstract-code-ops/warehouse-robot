cmake_minimum_required(VERSION 3.8)
project(cognitive_amr_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

# ── Install all resource directories ──────────────────────────
install(
  DIRECTORY urdf meshes launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()