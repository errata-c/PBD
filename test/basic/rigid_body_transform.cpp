#include <vector>
#include <deque>
#include <memory>
#include <pbd/common/Span.hpp>
#include <pbd/engine/RigidBody.hpp>

#include <fmt/format.h>

#include <cppitertools/itertools.hpp>

#include <catch2/catch_all.hpp>

using namespace pbd;
using Catch::Approx;

TEST_CASE("rigid body transforms") {
	RigidBody body;
	body.orientation = glm::quat(std::sqrt(0.5), 0.f, std::sqrt(0.5), 0.f);

	glm::vec3 test(1.f, 0.f, 0.f);

	glm::vec3 result = body.to_world_vector(test);
	REQUIRE(result.x == Approx(0.0));
	REQUIRE(result.y == Approx(0.0));
	REQUIRE(result.z == Approx(-1.0));

	result = body.to_local_vector(result);
	REQUIRE(result.x == Approx(test.x));
	REQUIRE(result.y == Approx(test.y));
	REQUIRE(result.z == Approx(test.z));
}