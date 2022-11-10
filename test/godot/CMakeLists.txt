
find_package(godot-cpp 3.4 CONFIG REQUIRED)

add_library(pbd_godot_test SHARED
	"src/gdnativelib.cpp"

	"src/EngineNode.cpp"
	"src/ManagedEngineNode.cpp"
	"src/PrefabRef.cpp"
)
target_include_directories(pbd_godot_test PRIVATE
	"include"
)

target_link_libraries(pbd_godot_test PRIVATE 
	godot::cpp 
	pbd::pbd
)

# Test installation
install_gdnlib(
	TARGET pbd_godot_test

	DESTINATION "${CMAKE_CURRENT_LIST_DIR}/godot_pbd_project/bin"

	CLASSES 
		"EngineNode"
		"PrefabRef"

	EXCLUDE_FROM_ALL
	COMPONENT "TEST_INSTALL"
)