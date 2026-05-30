cmake_minimum_required(VERSION 3.8)
project(cargo_bot_navigation)

find_package(ament_cmake REQUIRED)

# Instalar configs y maps para que sean accesibles via $(find-pkg-share)
install(DIRECTORY config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()