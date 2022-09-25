
if(NOT TARGET glm::glm)
	find_dependency(glm CONFIG)
endif()

if(NOT TARGET ez::kvstore)
	find_dependency(ez-kvstore CONFIG)
endif()

if(NOT TARGET ez::serialize)
	find_dependency(ez-serialize CONFIG)
endif()

if(NOT TARGET ez::math)
	find_dependency(ez-math CONFIG)
endif()

if(NOT TARGET cppitertools::cppitertools)
	find_dependency(cppitertools CONFIG)
endif()

if(NOT TARGET pbd::hashing)
	find_dependency(pbd-hashing CONFIG)
endif()