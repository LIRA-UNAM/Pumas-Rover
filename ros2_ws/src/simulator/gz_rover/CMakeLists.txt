cmake_minimum_required(VERSION 3.8)
project(gz_rover)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ament_cmake - NO buscar tu propio paquete
find_package(ament_cmake REQUIRED)

# Instalar archivos de lanzamiento
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Instalar archivos URDF
install(DIRECTORY urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()