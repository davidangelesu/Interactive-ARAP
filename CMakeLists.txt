cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(interactive_ARAP)

#Set cmake Folder
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#add libIGL
### libIGL options: choose between header only and compiled static library
option(LIBIGL_USE_STATIC_LIBRARY     "Use libigl as static library" OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)

find_package(LIBIGL REQUIRED)


# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs CACHE PATH "Path to lib folder")
set(Eigen3_INCLUDE_DIR ${LIBRARY_DIR}/Eigen/ CACHE PATH "Path to Eigen source folder")


# Add your project files
include_directories("include/")
file(GLOB SRCFILES src/*.cpp)



add_executable(interactive_ARAP ${SRCFILES} include/init_system_matrix.h src/init_system_matrix.cpp)

target_link_libraries(interactive_ARAP igl::core igl::opengl igl::opengl_glfw)
target_include_directories(interactive_ARAP PUBLIC ${Eigen3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})


# # Visual Studio properties
# set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT interactive_ARAP)
# set_property(TARGET interactive_ARAP PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)


