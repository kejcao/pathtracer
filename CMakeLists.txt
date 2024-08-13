cmake_minimum_required(VERSION 3.28)

project(raytracer LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_library(math)
target_sources(math
  PUBLIC
    FILE_SET CXX_MODULES FILES
      math.cpp
)

add_library(object)
target_sources(object
  PUBLIC
    FILE_SET CXX_MODULES FILES
      object.cpp
)
target_link_libraries(object math)

add_library(scene)
target_sources(scene
  PUBLIC
    FILE_SET CXX_MODULES FILES
      scene.cpp
)
target_link_libraries(scene math object)

add_library(camera)
target_sources(camera
  PUBLIC
    FILE_SET CXX_MODULES FILES
      camera.cpp
)
target_link_libraries(camera math scene object)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} math scene camera object)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
