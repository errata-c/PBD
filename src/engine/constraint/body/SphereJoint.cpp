#include <pbd/engine/constraint/body/SphereJoint.hpp>
#include <pbd/engine/Engine.hpp>

#include <pbd/engine/constraint/body/Utils.hpp>

#include <cppitertools/itertools.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	void CSphereJoint::serialize(const CSphereJoint& in, std::string& output) {

	}
	const char* CSphereJoint::deserialize(const char* first, const char* last, CSphereJoint& out) {
		return first;
	}

	void CSphereJoint::eval(Engine& engine, float rdt2) const {
		// First calculate the positional correction.
		std::array<RigidBody*, 2> bodies;
		bodies[0] = &engine.bodies.list[info[0].id];
		bodies[1] = &engine.bodies.list[info[1].id];

		apply_positional_correction(
			bodies,
			{ &info[0].r, &info[1].r },
			bodies[1]->to_world(info[1].r) - bodies[0]->to_world(info[0].r),
			compliance * rdt2
		);

		if (components) {
			std::array<glm::vec3, 2> a, b;
			for (int i : iter::range(2)) {
				a[i] = bodies[i]->to_world_vector(info[i].a);
				b[i] = bodies[i]->to_world_vector(info[i].b);
			}

			if (components & TargetBit) {
				glm::quat tmp = bodies[0]->orientation * glm::conjugate(bodies[1]->orientation * glm::conjugate(target.orientation));
				apply_angular_correction(bodies, glm::vec3(tmp.x, tmp.y, tmp.z) * 2.f, target.compliance * rdt2);
			}
			if (components & TwistLimitBit) {
				glm::vec3 n = glm::normalize(a[0] + a[1]);
				glm::vec3 n0 = glm::normalize(b[0] - glm::dot(n, b[0]) * n);
				glm::vec3 n1 = glm::normalize(b[1] - glm::dot(n, b[1]) * n);
				apply_angular_limit(bodies, n, n0, n1, twist_limit.min_angle, twist_limit.max_angle, twist_limit.compliance * rdt2);
			}
			if (components & SwingLimitBit) {
				apply_angular_limit(bodies, glm::cross(a[0], a[1]), a[0], a[1], swing_limit.min_angle, swing_limit.max_angle, swing_limit.compliance * rdt2);
			}
		}
	}

	void CSphereJoint::remap(int32_t offset) {

	}
	void CSphereJoint::transform(const Transform3& form) {

	}
}