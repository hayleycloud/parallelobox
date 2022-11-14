# Meta
cmake_minimum_required(VERSION 3.23)
project(parallelobox)

# C++ version control
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(OpenMP)

add_executable(parallelobox src/main.cpp src/config.cpp)
target_link_libraries(parallelobox CGAL::CGAL CGAL::CGAL_Core OpenMP::OpenMP_CXX)

