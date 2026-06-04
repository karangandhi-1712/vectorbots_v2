cmake_minimum_required(VERSION 3.8)
project(vector_agv_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install resource directories
install(DIRECTORY 
  launch 
  urdf 
  worlds 
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS 
  scripts/fleet_brain.py 
  scripts/single_brain.py 
  scripts/swarm_supervisor.py 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()