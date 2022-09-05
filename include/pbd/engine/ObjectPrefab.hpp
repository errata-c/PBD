#pragma once
#include <cinttypes>
#include <glm/vec3.hpp>

#include <pbd/common/ConstraintList.hpp>


namespace pbd {
	struct ObjectParticle {
		glm::vec3 position, velocity;
		float imass, radius;
		uint32_t flags;
	};

	/*
	This class stores a number of particles and constraints for them.
	At runtime the object can then be instanced into the scene as needed.

	Additionally, can store a list of bones to extract rotations from
	*/
	class ObjectPrefab {
	public:
		std::vector<ObjectParticle> particles;
		ConstraintList constraints;
	};
}