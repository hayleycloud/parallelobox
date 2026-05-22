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
find_package(Eigen3 5.0 REQUIRED NO_MODULE)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(OpenMP)

set(REPORTING 
	"standard" 
	CACHE STRING 
	"Set output reporting level (none, bare, standard, extra, verbose, all)")

if(REPORTING STREQUAL "none")
	message("Suppressing output reporting.")
	add_compile_definitions(VERBOSITY=0)
elseif(REPORTING STREQUAL "bare")
	message("Reporting only essential outputs.")
	add_compile_definitions(VERBOSITY=1)
elseif(REPORTING STREQUAL "standard")
	message("Reporting normal outputs.")
	add_compile_definitions(VERBOSITY=2)
elseif(REPORTING STREQUAL "extra")
	message("Reporting normal outputs and extra information.")
	add_compile_definitions(VERBOSITY=3)
elseif(REPORTING STREQUAL "verbose")
	message("Reporting with extensive detail.")
	add_compile_definitions(VERBOSITY=4)
elseif(REPORTING STREQUAL "all")
	message("Reporting absolutely everything.")
	add_compile_definitions(VERBOSITY=5)
else()
	message("Reporting normal outputs.")
	add_compile_definitions(VERBOSITY=2)
endif()

set(USE_DEBUG OFF CACHE BOOL "Use debug mode")

if(USE_DEBUG)
	message("Using debug mode")
	add_definitions(-DDEBUG)
endif()

set(OVERHANG_LOGIC 
	"balance" CACHE STRING 
	"Specify accuracy of overhang determination logic (accurate, balance, fast)")

if(OVERHANG_LOGIC STREQUAL "accurate")
	message("Using slow, high accuracy overhang determination logic.")
	add_compile_definitions(OVERHANG_LOGIC=1)
elseif(OVERHANG_LOGIC STREQUAL "balance")
	message("Using reasonably accurate overhang determination logic.")
	add_compile_definitions(OVERHANG_LOGIC=2)
elseif(OVERHANG_LOGIC STREQUAL "fast")
	message("Using fast, low accuracy accurate overhang determination logic.")
	add_compile_definitions(OVERHANG_LOGIC=3)
else()
	message("Using reasonably accurate overhang determination logic.")
	add_compile_definitions(OVERHANG_LOGIC=2)
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
	src/fillempty.cpp
	src/resolve.cpp)
target_link_libraries(parallelobox Eigen3::Eigen CGAL::CGAL CGAL::CGAL_Core OpenMP::OpenMP_CXX)

 
