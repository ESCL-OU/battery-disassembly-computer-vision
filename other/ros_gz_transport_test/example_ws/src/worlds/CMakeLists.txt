cmake_minimum_required(VERSION 3.8)
project(worlds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(PROGRAMS 
  setup/environment.bash
  DESTINATION share/${PROJECT_NAME}/setup
)

install(DIRECTORY
  assets
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
