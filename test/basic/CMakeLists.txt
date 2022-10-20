find_package(Catch2 CONFIG REQUIRED)

add_executable(pbd_basic_test 
	"span.cpp"
	"tet_volume.cpp"
	"perpendicular.cpp"
)
target_link_libraries(pbd_basic_test
PRIVATE 
	Catch2::Catch2WithMain
	pbd::pbd
)