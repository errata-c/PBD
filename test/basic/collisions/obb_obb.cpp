#include <vector>
#include <deque>
#include <memory>
#include <pbd/common/Span.hpp>
#include <pbd/engine/RigidBody.hpp>
#include <pbd/engine/collider/AllColliders.hpp>

#include <fmt/format.h>

#include <cppitertools/itertools.hpp>

#include <catch2/catch_all.hpp>

using namespace pbd;
using Catch::Approx;

TEST_CASE("OBB OBB No Collision") {
	// Try each possible collision configuration:
	// The 3 axes of r0
	// The 3 axes of r1
	// The 9 axes for edges between them

	// First 3 axes
	{
		RigidBody r0, r1;
	
		r0.position = glm::vec3{0,0,0};
		r0.orientation = glm::quat(1,0,0,0);
		r0.dims = glm::vec3(1);
		//r0.orientation = glm::angleAxis(glm::radians(45.f), glm::vec3(0,0,0));
	
		r1.position = glm::vec3{3,0,0};
		r1.orientation = glm::quat(1,0,0,0);
		r1.dims = glm::vec3(1);

		std::optional<Collision> result;

		// r0 x-axis
		r1.position = glm::vec3{ 3,0,0 };

		result = pbd::obb_obb_collide(r0, r1);
		REQUIRE(!result.has_value());

		// r0 y-axis
		r1.position = glm::vec3{ 0,3,0 };

		result = pbd::obb_obb_collide(r0, r1);
		REQUIRE(!result.has_value());

		// r0 z-axis
		r1.position = glm::vec3{ 0,0,3 };

		result = pbd::obb_obb_collide(r0, r1);
		REQUIRE(!result.has_value());
	}

	// Second 3 axes
	{
		RigidBody r0, r1;

		r0.position = glm::vec3{ 0,0,0 };
		r0.dims = glm::vec3(1);


		r1.orientation = glm::quat(1, 0, 0, 0);
		r1.dims = glm::vec3(1);

		std::optional<Collision> result;

		// r1 x-axis
		r0.orientation = glm::angleAxis(glm::radians(45.f), glm::vec3(0, 1, 0));
		r1.position = glm::vec3{ std::sqrt(2.0) + 1.01, 0,0 };

		result = pbd::obb_obb_collide(r0, r1);
		REQUIRE(!result.has_value());

		// r1 y-axis
		r0.orientation = glm::angleAxis(glm::radians(45.f), glm::vec3(1, 0, 0));
		r1.position = glm::vec3{ 0, std::sqrt(2.0) + 1.01, 0 };

		result = pbd::obb_obb_collide(r0, r1);
		REQUIRE(!result.has_value());

		// r1 z-axis
		r0.orientation = glm::angleAxis(glm::radians(45.f), glm::vec3(1, 0, 0));
		r1.position = glm::vec3{ 0,0, std::sqrt(2.0) + 1.01 };

		result = pbd::obb_obb_collide(r0, r1);
		REQUIRE(!result.has_value());
	}

	// 9 edge axes
	{
		RigidBody r0, r1;

		r0.position = glm::vec3{ 0,0,0 };
		r0.dims = glm::vec3(1);

		r1.dims = glm::vec3(1);

		std::optional<Collision> result;

		// r0 x to r1 x
		r0.orientation = glm::angleAxis(glm::radians(45.f), glm::vec3(0, 0, 1));
		r1.orientation = glm::angleAxis(glm::radians(45.f), glm::vec3(1, 0, 0));
		r1.position = glm::vec3{ 0, 2.0 * std::sqrt(2.0) + 0.01, 0 };

		result = pbd::obb_obb_collide(r0, r1);
		REQUIRE(!result.has_value());

		// r0 x to r1 y
		r0.orientation = glm::angleAxis(glm::radians(45.f), glm::vec3(1, 0, 0));
		r1.orientation = glm::angleAxis(glm::radians(45.f), glm::vec3(0, 0, 1));
		r1.position = glm::vec3{ 0, 2.0 * std::sqrt(2.0) + 0.01, 0 };

		result = pbd::obb_obb_collide(r0, r1);
		REQUIRE(!result.has_value());
	}
}

