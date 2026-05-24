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
	message(STATUS "Suppressing output reporting.")
	add_compile_definitions(VERBOSITY=0)
elseif(REPORTING STREQUAL "bare")
	message(STATUS "Reporting only essential outputs.")
	add_compile_definitions(VERBOSITY=1)
elseif(REPORTING STREQUAL "standard")
	message(STATUS "Reporting normal outputs.")
	add_compile_definitions(VERBOSITY=2)
elseif(REPORTING STREQUAL "extra")
	message(STATUS "Reporting normal outputs and extra information.")
	add_compile_definitions(VERBOSITY=3)
elseif(REPORTING STREQUAL "verbose")
	message(STATUS "Reporting with extensive detail.")
	add_compile_definitions(VERBOSITY=4)
elseif(REPORTING STREQUAL "all")
	message(STATUS "Reporting absolutely everything.")
	add_compile_definitions(VERBOSITY=5)
else()
	message(STATUS "Reporting normal outputs.")
	add_compile_definitions(VERBOSITY=2)
endif()

set(USE_DEBUG OFF CACHE BOOL "Use debug mode")

if(USE_DEBUG)
	message("Using debug mode")
	add_definitions(-DDEBUG)
endif()

set(OVERHANG_LOGIC 
	"balance" CACHE STRING 
	"Specify accuracy of overhang determination logic \
	(accurate, balance [default], fast)")

if(OVERHANG_LOGIC STREQUAL "accurate")
	message(STATUS "Using slow, high accuracy overhang determination logic.")
	add_compile_definitions(OVERHANG_LOGIC=1)
elseif(OVERHANG_LOGIC STREQUAL "balance")
	message(STATUS "Using reasonably accurate overhang determination logic.")
	add_compile_definitions(OVERHANG_LOGIC=2)
elseif(OVERHANG_LOGIC STREQUAL "fast")
	message(STATUS "Using fast, low accuracy overhang determination logic.")
	add_compile_definitions(OVERHANG_LOGIC=3)
else()
	message(WARNING "Unknown overhang determination logic specified!")
	message(STATUS "Using default reasonably accurate overhang determination logic.")
	add_compile_definitions(OVERHANG_LOGIC=2)
endif()

set(RESOLVE_PARALLELIZE
	"narrow" CACHE STRING 
	"Computational parallelization of the conflict resolution algorithm \
	(broad, narrow [default], off)")

if(RESOLVE_PARALLELIZE STREQUAL "off")
	message(STATUS "Conflict resolution algorithm will run in serial.")
	add_compile_definitions(PARALLEL_RESOLVE=0)
elseif(RESOLVE_PARALLELIZE STREQUAL "narrow")
	message(STATUS "Conflict resolution algorithm will parallelize local search.")
	add_compile_definitions(PARALLEL_RESOLVE=1)
elseif(RESOLVE_PARALLELIZE STREQUAL "broad")
	message(STATUS "Conflict resolution algorithm will parallelize broad search.")
	message(WARNING 
"CGAL methods that are supposed to be thread-safe but are not may result in \
race conditions and indeterminate behavior when using broad parallelization \
of the conflict resolution algorithm.\
\
Narrow parallelization is less fast but guarantees stability and determinacy.")
	add_compile_definitions(PARALLEL_RESOLVE=2)
else()
	message(WARNING "Unknown conflict resolution parallelization specified!")
	message(STATUS "Conflict resolution algorithm will parallelize narrow search.")
	add_compile_definitions(PARALLEL_RESOLVE=1)
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

 
