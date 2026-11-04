cmake_minimum_required(VERSION 3.8)
project(vacuum_cleaner_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate StartStop.srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartStop.srv"
)

ament_package()
