cmake_minimum_required(VERSION 3.20)
project(masssprings)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Amend custom CMake modules path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

### libIGL options: choose between header only and compiled static library
option(LIBIGL_USE_STATIC_LIBRARY    "Use libigl as static library" OFF)
option(LIBIGL_OPENGL                "Use OpenGL"         ON)
option(LIBIGL_STB                   "Use PNG"            ON)
option(LIBIGL_GLFW                  "Use GLFW"           ON)
include(libigl)

# Use solution files
option(USE_SOLUTION "Use solution files" OFF)
set(SRCDIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
if (USE_SOLUTION)
  set(SRCDIR "${CMAKE_CURRENT_SOURCE_DIR}/solution")
endif()
file(GLOB SRCFILES CONFIGURE_DEPENDS "${SRCDIR}/*.cpp")

# Create library from source files
add_library(lib ${SRCFILES})
target_include_directories(lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories(lib SYSTEM PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/json/")

# Link libIGL components
target_link_libraries(lib PUBLIC igl::core)
if(LIBIGL_OPENGL)
  target_link_libraries(lib PUBLIC igl::opengl)
endif()
if(LIBIGL_STB)
  target_link_libraries(lib PUBLIC igl::stb)
endif()
if(LIBIGL_GLFW)
  target_link_libraries(lib PUBLIC igl::glfw)
endif()

set(DENSE "${PROJECT_NAME}_dense")
add_executable(${DENSE} "main.cpp")
target_link_libraries(${PROJECT_NAME}_dense PRIVATE lib)

set(SPARSE "${PROJECT_NAME}_sparse")
add_executable(${SPARSE} "main.cpp")
target_compile_definitions(${SPARSE} PRIVATE SPARSE)
target_link_libraries(${SPARSE} PRIVATE lib)

set(SHOWCASE "${PROJECT_NAME}_showcase")
add_executable(${SHOWCASE} "main_showcase.cpp" "src/JelloMesh.cpp")
target_link_libraries(${SHOWCASE} PRIVATE lib)

foreach(tgt IN ITEMS lib ${DENSE} ${SPARSE} ${SHOWCASE})
  if (MSVC)
    target_compile_options(${tgt} PRIVATE /W4 /permissive-)
  else()
    target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endforeach()


