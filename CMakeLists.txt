# Meta
cmake_minimum_required(VERSION 3.23)
project(parallelobox)

# C++ version control
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Make Fast
#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
#set(CMAKE_CXX_FLAGS_DEBUG "-g -O3")

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(OpenMP)

set(USE_DEBUG OFF CACHE BOOL "Use debug mode")

if(USE_DEBUG)
	message("Using debug mode")
	add_definitions(-DDEBUG)
endif()

add_executable(parallelobox 
	src/main.cpp 
	src/args.cpp 
	src/config.cpp 
	src/random.cpp
	src/vec.cpp
	src/clustering.cpp
	src/grid_cut.cpp 
	src/align.cpp 
	src/objective.cpp
	src/symmetry.cpp 
	src/meshbox.cpp
	src/regiongrowth.cpp
	src/resolve.cpp)
#   src/blockmerge.cpp)
target_link_libraries(parallelobox Eigen3::Eigen CGAL::CGAL CGAL::CGAL_Core OpenMP::OpenMP_CXX)

