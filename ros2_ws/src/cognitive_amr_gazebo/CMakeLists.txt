cmake_minimum_required(VERSION 3.8)
project(cognitive_amr_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  cognitive_amr_gazebo/laser_merger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME laser_merger
)

install(PROGRAMS
  cognitive_amr_gazebo/task_dispatcher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME task_dispatcher
)

install(DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
