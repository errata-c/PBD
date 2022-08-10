cmake_minimum_required(VERSION 3.20)

project(PBD
VERSION 0.1.0
LANGUAGES CXX)

option(PBD_BUILD_TESTS "Build the test executables" OFF)
set(PBD_CONFIG_DIR "share/pbd" CACHE STRING "The relative directory to install package config files.")

include(GNUInstallDirs)
include(FetchContent)

FetchContent_Declare(
	ez-cmake
	GIT_REPOSITORY "https://github.com/errata-c/ez-cmake.git"
	GIT_TAG "main"
)
FetchContent_MakeAvailable(ez-cmake)

find_package(glm CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

find_path(PHMAP_HEADERS "parallel_hashmap/phmap.h")
if("${PHMAP_HEADERS}" STREQUAL "PHMAP_HEADERS-NOTFOUND")
	message(FATAL_ERROR "Failed to find the parallel_hashmap headers!")
endif()

add_library(pbd STATIC
	"src/Engine.cpp"
	
	"src/ExtractedRotation.cpp"
	"src/hash/Table.cpp"

	"src/constraint/Distance.cpp"
	"src/constraint/TetraVolume.cpp"

	"src/collide/Particle.cpp"
	"src/collide/Plane.cpp"
)
target_include_directories(pbd PUBLIC
	$<INSTALL_INTERFACE:include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${PHMAP_HEADERS}>
)
target_compile_features(pbd PUBLIC cxx_std_17)
target_link_libraries(pbd PUBLIC 
	glm::glm
	nlohmann_json::nlohmann_json
)

add_library(pbd::pbd ALIAS pbd)

if(PBD_BUILD_TESTS)
	add_subdirectory("test")
endif()