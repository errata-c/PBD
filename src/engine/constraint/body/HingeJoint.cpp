#include <pbd/engine/constraint/body/HingeJoint.hpp>
#include <pbd/engine/constraint/body/Utils.hpp>

#include <pbd/engine/Engine.hpp>

#include <cppitertools/itertools.hpp>

namespace pbd {
	static constexpr uint32_t 
		TargetBit = 1,
		LimitBit = 2;

	void CHingeJoint::eval(Engine& engine, float rdt2) const {
		// First calculate the positional correction.
		std::array<RigidBody*, 2> bodies;
		bodies[0] = &engine.bodies.list[info[0].id];
		bodies[1] = &engine.bodies.list[info[1].id];

		apply_positional_correction(
			bodies,
			{ &info[0].r, &info[1].r }, 
			bodies[1]->to_world(info[1].r) - bodies[0]->to_world(info[0].r),
			positional_compliance * rdt2
		);

		std::array<glm::vec3, 2> a;
		for (int i: iter::range(2)) {
			a[i] = bodies[i]->to_local_vector(info[i].a);
		}

		glm::vec3 n = glm::cross(a[0], a[1]);
		apply_angular_correction(bodies, n, angular_compliance * rdt2);

		if (components) {
			std::array<glm::vec3, 2> b;
			for (int i: iter::range(2)) {
				b[i] = bodies[i]->to_local_vector(info[i].b);
			}

			// Always apply the target before the limit, so the limit can work properly.
			if (components & TargetBit) {
				glm::vec3 btarget = glm::rotate(glm::angleAxis(target.angle, a[0]), b[0]);
				apply_angular_correction(bodies, glm::cross(btarget, b[1]), target.compliance * rdt2);
			}
			if (components & LimitBit) {
				apply_angular_limit(bodies, a[0], b[0], b[1], limit.min_angle, limit.max_angle, limit.compliance * rdt2);
			}
		}
	}

	void CHingeJoint::remap(int32_t offset) {
		info[0].id += offset;
		info[1].id += offset;
	}
	void CHingeJoint::transform(const Transform3& form) {
		
	}


	bool CHingeJoint::has_limit() const noexcept {
		return bool(components & LimitBit);
	}
	bool CHingeJoint::has_target() const noexcept {
		return bool(components & TargetBit);
	}

	void CHingeJoint::enable_limit(bool val) {
		components = (components & ~LimitBit) | uint32_t(val);
	}
	void CHingeJoint::enable_target(bool val) {
		components = (components & ~TargetBit) | uint32_t(val);
	}
}