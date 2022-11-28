find_package(Catch2 CONFIG REQUIRED)

add_executable(pbd_basic_test 
	"span.cpp"
	"tet_volume.cpp"
	"perpendicular.cpp"
	"sdf.cpp"
	"rigid_body_transform.cpp"
	"rigid_body_corrections.cpp"

	"collisions/obb_obb.cpp"
)
target_link_libraries(pbd_basic_test
PRIVATE 
	Catch2::Catch2WithMain
	pbd::pbd
)