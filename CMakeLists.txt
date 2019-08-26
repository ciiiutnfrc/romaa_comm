# CMakeLists files in this project can refer to the root source directory 
# of the project as ${ROMAA_SOURCE_DIR} and to the root binary directory 
# of the project as ${ROMAA_BINARY_DIR}.
cmake_minimum_required(VERSION 2.8.11)
project(romaa_comm)

set(CMAKE_CXX_STANDARD 11)

# Create a library called "romaa_comm" which includes the source file "romaa_comm.cpp".
# The extension is already found. Any number of sources could be listed here.
add_library(romaa_comm SHARED romaa_comm.cpp)

# Make sure the compiler can find include files for our comm library
# when other libraries or executables link to romaa_comm
target_include_directories(romaa_comm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Install library
install(TARGETS romaa_comm DESTINATION lib/${PROJECT_NAME})

# Install library headers
file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME})

