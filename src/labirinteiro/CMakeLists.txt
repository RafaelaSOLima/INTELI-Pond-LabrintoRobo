cmake_minimum_required(VERSION 3.8)
project(labirinteiro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Library with common code (STATIC para evitar problemas de linking)
add_library(labirinteiro_lib STATIC
  src/graph.cpp
  src/pathfinder.cpp
  src/mapper.cpp
)

ament_target_dependencies(labirinteiro_lib
  rclcpp
  cg_interfaces
  std_msgs
)

# Importante: definir como position independent code
set_target_properties(labirinteiro_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# Parte 1 - Navegação com mapa
add_executable(parte1_navegacao src/parte1_navegacao.cpp)
target_link_libraries(parte1_navegacao labirinteiro_lib)
ament_target_dependencies(parte1_navegacao
  rclcpp
  cg_interfaces
  std_msgs
)

# Parte 2 - Mapeamento
add_executable(parte2_mapeamento src/parte2_mapeamento.cpp)
target_link_libraries(parte2_mapeamento labirinteiro_lib)
ament_target_dependencies(parte2_mapeamento
  rclcpp
  cg_interfaces
  std_msgs
)

# Install executáveis
install(TARGETS
  parte1_navegacao
  parte2_mapeamento
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (opcional, mas boa prática)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()