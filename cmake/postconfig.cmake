
find_path(PBD_PHMAP_HEADERS "parallel_hashmap/phmap.h")
if("${PBD_PHMAP_HEADERS}" STREQUAL "PBD_PHMAP_HEADERS-NOTFOUND")
	if(pbd_FIND_REQUIRED)
		message(FATAL_ERROR "Failed to find the parallel_hashmap headers!")
	else()
		set(pbd_FOUND FALSE)
		return()
	endif()
endif()

find_path(PBD_PCG_HEADERS "pcg_random.hpp")
if("${PBD_PCG_HEADERS}" STREQUAL "PBD_PCG_HEADERS-NOTFOUND")
	if(pbd_FIND_REQUIRED)
		message(FATAL_ERROR "Failed to find the pcg_random headers!")
	else()
		set(pbd_FOUND FALSE)
		return()
	endif()
endif()

set_property(TARGET pbd::pbd
	APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${PBD_PHMAP_HEADERS} ${PBD_PCG_HEADERS}
)