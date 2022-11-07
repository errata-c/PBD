#include <pbd/engine/body/constraint/HingeJoint.hpp>
#include <pbd/engine/Engine.hpp>

#include <pbd/engine/body/Utils.hpp>

namespace pbd {
	void CHingeJoint::eval(Engine& engine, float rdt2) const {
		float alpha = (compliance * rdt2);

		// First calculate the positional correction.
		std::array<RigidBody*, 2> b;
		b[0] = &engine.bodies.list[info[0].id];
		b[1] = &engine.bodies.list[info[1].id];

		apply_positional_correction(b, { &info[0].r, &info[1].r }, b[1]->to_world(info[1].r) - b[0]->to_world(info[0].r), alpha);

		glm::vec3 n = glm::cross(b[0]->to_world_vector(info[0].a), b[1]->to_world_vector(info[1].a));
		apply_angular_correction(b, n, alpha);
	}

	void CHingeJoint::remap(int32_t offset) {
		info[0].id += offset;
		info[1].id += offset;
	}
	void CHingeJoint::transform(const Transform3& form) {
		
	}
}