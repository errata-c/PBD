cmake_minimum_required(VERSION 3.21)

project(PBD
VERSION 0.2.0
LANGUAGES CXX)

include(GNUInstallDirs)
include(FetchContent)

set(PBD_CONFIG_DIR "${CMAKE_INSTALL_DATADIR}/pbd" CACHE STRING "The relative directory to install package config files.")

FetchContent_Declare(
	ez-cmake
	GIT_REPOSITORY "https://github.com/errata-c/ez-cmake.git"
	GIT_TAG "main"
)
FetchContent_MakeAvailable(ez-cmake)

find_package(glm CONFIG REQUIRED)
find_package(ez-kvstore CONFIG REQUIRED)
find_package(ez-serialize CONFIG REQUIRED)
find_package(ez-math CONFIG REQUIRED)
find_package(cppitertools CONFIG REQUIRED)
find_package(xxHash CONFIG REQUIRED)
find_package(pbd-hashing CONFIG REQUIRED)

find_path(PHMAP_HEADERS "parallel_hashmap/phmap.h")
if("${PHMAP_HEADERS}" STREQUAL "PHMAP_HEADERS-NOTFOUND")
	message(FATAL_ERROR "Failed to find the parallel_hashmap headers!")
endif()

find_path(PCG_HEADERS "pcg_random.hpp")
if("${PCG_HEADERS}" STREQUAL "PCG_HEADERS-NOTFOUND")
	message(FATAL_ERROR "Failed to find the pcg_random headers!")
endif()

add_library(pbd STATIC
	"src/common/Utils.cpp"
	"src/common/TransformTracker.cpp"
	"src/common/RotationExtractor.cpp"

	"src/engine/Engine.cpp"
	"src/engine/ManagedEngine.cpp"
	
	"src/engine/ObjectMap.cpp"
	"src/engine/ObjectQueue.cpp"
	"src/engine/ConstraintRef.cpp"

	"src/engine/ParticleList.cpp"
	"src/engine/ConstraintList.cpp"
	"src/engine/TrackerList.cpp"

	"src/engine/constraint/Distance.cpp"
	"src/engine/constraint/TetraVolume.cpp"
	"src/engine/constraint/NHTetraVolume.cpp"

	"src/engine/collide/Particle.cpp"
	"src/engine/collide/Plane.cpp"
	"src/engine/collide/ParticleCompliant.cpp"
	"src/engine/collide/PlaneCompliant.cpp"

	"src/prefab/Prefab.cpp"
	"src/prefab/PrefabDatabase.cpp"
	"src/prefab/PrefabParticle.cpp"
	"src/prefab/PrefabTracker.cpp"
)
target_include_directories(pbd PUBLIC
	$<INSTALL_INTERFACE:include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${PHMAP_HEADERS}>
	$<BUILD_INTERFACE:${PCG_HEADERS}>
)
target_compile_features(pbd PUBLIC cxx_std_17)
target_link_libraries(pbd PUBLIC
	glm::glm
	cppitertools::cppitertools

	ez::kvstore
	ez::serialize
	ez::math

	xxHash::xxhash

	pbd::hashing
)

add_library(pbd::pbd ALIAS pbd)

if(PROJECT_IS_TOP_LEVEL)
	include(CTest)
	if(BUILD_TESTING)
		add_subdirectory("test")
	endif()

	install(
		DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/"
		TYPE INCLUDE
		FILES_MATCHING
		PATTERN "*.h" PATTERN "*.hpp"
	)

	install(TARGETS pbd
		EXPORT pbd-targets
		RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/$<CONFIG>"
		ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}/$<CONFIG>"
		LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}/$<CONFIG>"
	)

	install_package(
		NAME "pbd"
		EXPORT "pbd-targets"
		VERSION "${CMAKE_PROJECT_VERSION}"

		NAMESPACE "pbd::"

		# Version compatiblity
		COMPATIBILITY "SameMajorVersion"

		# Relative install location of the package config files.
		DESTINATION "${PBD_CONFIG_DIR}"

		# Pre and Post configuration files for the packages. Files are run as scripts when the package is found.
		# Most dependencies must be found in the preconfig file, except non-target dependencies
		# Non-target dependencies should probably be found in the post config, and added onto the imported target manually.
		# Non-target dependencies kinda suck, avoid them if you can.

		PRECONFIG "cmake/preconfig.cmake"
		POSTCONFIG "cmake/postconfig.cmake"
	)
endif()
