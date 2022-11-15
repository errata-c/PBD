#include <pbd/engine/constraint/body/SphereJoint.hpp>
#include <pbd/engine/Engine.hpp>

#include <pbd/engine/constraint/body/Utils.hpp>

#include <cppitertools/itertools.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	void CSphereJoint::serialize(const CSphereJoint& in, std::string& output) {
		for (int i: iter::range(2)) {
			ez::serialize::i32(in.info[i].id, output);
			ez::serialize::f32(in.info[i].r[0], output);
			ez::serialize::f32(in.info[i].r[1], output);
			ez::serialize::f32(in.info[i].r[2], output);

			ez::serialize::f32(in.info[i].a[0], output);
			ez::serialize::f32(in.info[i].a[1], output);
			ez::serialize::f32(in.info[i].a[2], output);

			ez::serialize::f32(in.info[i].b[0], output);
			ez::serialize::f32(in.info[i].b[1], output);
			ez::serialize::f32(in.info[i].b[2], output);
		}

		ez::serialize::f32(in.compliance, output);
		ez::serialize::u8(static_cast<uint8_t>(in.components), output);
		if (in.components & TargetBit) {
			ez::serialize::f32(in.target.orientation[0], output);
			ez::serialize::f32(in.target.orientation[1], output);
			ez::serialize::f32(in.target.orientation[2], output);
			ez::serialize::f32(in.target.orientation[3], output);

			ez::serialize::f32(in.target.compliance, output);
		}
		if (in.components & TwistLimitBit) {
			ez::serialize::f32(in.twist_limit.min_angle, output);
			ez::serialize::f32(in.twist_limit.max_angle, output);
			ez::serialize::f32(in.twist_limit.compliance, output);
		}
		if (in.components & SwingLimitBit) {
			ez::serialize::f32(in.swing_limit.min_angle, output);
			ez::serialize::f32(in.swing_limit.max_angle, output);
			ez::serialize::f32(in.swing_limit.compliance, output);
		}
	}
	const char* CSphereJoint::deserialize(const char* first, const char* last, CSphereJoint& out) {
		for (int i : iter::range(2)) {
			first = ez::deserialize::i32(first, last, out.info[i].id);
			first = ez::deserialize::f32(first, last, out.info[i].r[0]);
			first = ez::deserialize::f32(first, last, out.info[i].r[1]);
			first = ez::deserialize::f32(first, last, out.info[i].r[2]);

			first = ez::deserialize::f32(first, last, out.info[i].a[0]);
			first = ez::deserialize::f32(first, last, out.info[i].a[1]);
			first = ez::deserialize::f32(first, last, out.info[i].a[2]);

			first = ez::deserialize::f32(first, last, out.info[i].b[0]);
			first = ez::deserialize::f32(first, last, out.info[i].b[1]);
			first = ez::deserialize::f32(first, last, out.info[i].b[2]);
		}

		first = ez::deserialize::f32(first, last, out.compliance);
		uint8_t bits = 0;
		first = ez::deserialize::u8(first, last, bits);
		out.components = bits;
		if (bits & TargetBit) {
			first = ez::deserialize::f32(first, last, out.target.orientation[0]);
			first = ez::deserialize::f32(first, last, out.target.orientation[1]);
			first = ez::deserialize::f32(first, last, out.target.orientation[2]);
			first = ez::deserialize::f32(first, last, out.target.orientation[3]);

			first = ez::deserialize::f32(first, last, out.target.compliance);
		}
		if (bits & TwistLimitBit) {
			first = ez::deserialize::f32(first, last, out.twist_limit.min_angle);
			first = ez::deserialize::f32(first, last, out.twist_limit.max_angle);
			first = ez::deserialize::f32(first, last, out.twist_limit.compliance);
		}
		if (bits & SwingLimitBit) {
			first = ez::deserialize::f32(first, last, out.swing_limit.min_angle);
			first = ez::deserialize::f32(first, last, out.swing_limit.max_angle);
			first = ez::deserialize::f32(first, last, out.swing_limit.compliance);
		}

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

	void CSphereJoint::remap(int32_t particle_offset, int32_t body_offset) {
		info[0].id += body_offset;
		info[1].id += body_offset;
	}
	void CSphereJoint::transform(const Transform3& form) {

	}
}