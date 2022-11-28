#include <vector>
#include <deque>
#include <memory>
#include <pbd/common/Span.hpp>
#include <pbd/engine/RigidBody.hpp>
#include <pbd/engine/Engine.hpp>
#include <pbd/engine/constraint/body/Utils.hpp>

#include <cppitertools/itertools.hpp>

#include <catch2/catch_all.hpp>

#include "formatting.hpp"

using namespace pbd;
using Catch::Approx;

TEST_CASE("positional correction") {
	Engine engine;
	engine.gravity = vec3_t(0);
	int i0 = engine.bodies.list.add_box(
		Transform3{}, 
		1.0,
		vec3_t(1,1,1)
	);
	int i1 = engine.bodies.list.add_box(
		Transform3{vec3_t(0,0,-3)},
		1.0,
		vec3_t(1, 1, 1)
	);

	// Apply a positional correction such that the bodies are pulled together
	vec3_t norm = vec3_t(0,0,-3);
	std::array<RigidBody*, 2> bodies{ &engine.bodies.list[i0], &engine.bodies.list[i1] };
	std::array<vec3_t, 2> attach{vec3_t(0), vec3_t(0)};
	std::array<const vec3_t*, 2> r{&attach[0], &attach[1]};

	// 60 times a second
	real_t delta = 1.0 / 60.0;
	// 8 substeps
	real_t sdt = delta / 8.0;
	real_t sdt2 = sdt * sdt;

	// Infinitely stiff
	real_t compliance = 0.0;

	// Save state for comparison
	std::array<RigidBody, 2> prev{*bodies[0], *bodies[1]};

	SECTION("equal masses") {
		apply_positional_correction(bodies, r, norm, compliance * sdt2);

		std::array<vec3_t, 2> pdelta;
		for (int i : iter::range(2)) {
			pdelta[i] = bodies[i]->position - prev[i].position;
		}

		CAPTURE(pdelta[0]);
		CAPTURE(pdelta[1]);

		REQUIRE(pdelta[0].z == Approx(-1.5));
		REQUIRE(pdelta[1].z == Approx(1.5));

		REQUIRE(pdelta[0].x == Approx(0));
		REQUIRE(pdelta[1].x == Approx(0));

		REQUIRE(pdelta[0].y == Approx(0));
		REQUIRE(pdelta[1].y == Approx(0));
	}
	
	SECTION("unequal masses") {
		bodies[0]->set_mass(1.0);
		bodies[1]->set_mass(2.0);

		apply_positional_correction(bodies, r, norm, compliance * sdt2);

		std::array<vec3_t, 2> pdelta;
		for (int i : iter::range(2)) {
			pdelta[i] = bodies[i]->position - prev[i].position;
		}

		real_t distance = 3.0;
		std::array<real_t, 2> factors{
			bodies[0]->imass / (bodies[0]->imass + bodies[1]->imass),
			bodies[1]->imass / (bodies[0]->imass + bodies[1]->imass)
		};

		CAPTURE(pdelta[0]);
		CAPTURE(pdelta[1]);

		REQUIRE(pdelta[0].z == Approx(-distance * factors[0]));
		REQUIRE(pdelta[1].z == Approx(distance * factors[1]));

		REQUIRE(pdelta[0].x == Approx(0));
		REQUIRE(pdelta[1].x == Approx(0));

		REQUIRE(pdelta[0].y == Approx(0));
		REQUIRE(pdelta[1].y == Approx(0));
	}
}


TEST_CASE("angular correction") {
	Engine engine;
	engine.gravity = vec3_t(0);
	int i0 = engine.bodies.list.add_box(
		Transform3{ vec3_t(0,0,0), glm::angleAxis(glm::radians(real_t(30)), vec3_t(0,1,0)) },
		1.0,
		vec3_t(1, 1, 1)
	);
	int i1 = engine.bodies.list.add_box(
		Transform3{ vec3_t(0,0,0), glm::angleAxis(glm::radians(real_t(-30)), vec3_t(0,1,0)) },
		1.0,
		vec3_t(1, 1, 1)
	);

	// Apply an angular correction, such that the bodies' orientations are aligned
	std::array<RigidBody*, 2> bodies{ &engine.bodies.list[i0], &engine.bodies.list[i1] };

	quat_t q = bodies[0]->orientation * glm::inverse(bodies[1]->orientation);
	vec3_t norm = vec3_t(q.x, q.y, q.z) * real_t(2) * glm::sign(q.w);

	// 60 times a second
	real_t delta = 1.0 / 60.0;
	// 8 substeps
	real_t sdt = delta / 8.0;
	real_t sdt2 = sdt * sdt;

	// Infinitely stiff
	real_t compliance = 0.0;

	// Save state for comparison
	std::array<RigidBody, 2> prev{ *bodies[0], *bodies[1] };

	struct AngleAxis{
		real_t angle;
		vec3_t axis;
	};
	auto convert_angle_axis = [](const quat_t & rot) -> AngleAxis {
		vec3_t delta = vec3_t(rot.x, rot.y, rot.z) * real_t(2) * glm::sign(rot.w);
		real_t len = glm::length(delta);
		if (len < 1e-5) {
			delta = vec3_t(1,0,0);
			len = 0.0;
		}
		else {
			delta /= len;
		}

		return AngleAxis{ len, delta };
	};

	SECTION("equal masses") {
		bodies[0]->set_mass(1.0);
		bodies[1]->set_mass(1.0);

		// Due to linearization of the angular update, it only converges onto the right solution after multiple iterations.
		for (int i = 0; i < 8; ++i) {
			quat_t q = bodies[0]->orientation * glm::inverse(bodies[1]->orientation);
			vec3_t norm = vec3_t(q.x, q.y, q.z) * real_t(2);

			apply_angular_correction(bodies, norm, compliance * sdt2);

			// Renormalize
			//bodies[0]->orientation = glm::normalize(bodies[0]->orientation);
			//bodies[1]->orientation = glm::normalize(bodies[1]->orientation);
		}
		//bodies[0]->orientation = glm::normalize(bodies[0]->orientation);
		//bodies[1]->orientation = glm::normalize(bodies[1]->orientation);

		CAPTURE(bodies[0]->orientation);
		CAPTURE(bodies[1]->orientation);

		REQUIRE(std::abs(bodies[0]->orientation.x) < 1e-5);
		REQUIRE(std::abs(bodies[0]->orientation.y) < 1e-5);
		REQUIRE(std::abs(bodies[0]->orientation.z) < 1e-5);
		REQUIRE(std::abs(bodies[0]->orientation.w - 1.0) < 1e-5);

		REQUIRE(std::abs(bodies[1]->orientation.x) < 1e-5);
		REQUIRE(std::abs(bodies[1]->orientation.y) < 1e-5);
		REQUIRE(std::abs(bodies[1]->orientation.z) < 1e-5);
		REQUIRE(std::abs(bodies[1]->orientation.w - 1.0) < 1e-5);
	}

	SECTION("unequal masses") {
		bodies[0]->set_mass(1.0);
		bodies[1]->set_mass(2.0);

	}
}