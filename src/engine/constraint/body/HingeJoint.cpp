#include <pbd/engine/constraint/body/HingeJoint.hpp>
#include <pbd/engine/constraint/body/Utils.hpp>

#include <pbd/engine/Engine.hpp>

#include <cppitertools/itertools.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	void CHingeJoint::serialize(const CHingeJoint& in, std::string& output) {
		for (int i: iter::range(2)) {
			ez::serialize::i32(in.info[i].id, output);

			
			ez::serialize::f32(in.info[i].a[0], output);
			ez::serialize::f32(in.info[i].a[1], output);
			ez::serialize::f32(in.info[i].a[2], output);

			ez::serialize::f32(in.info[i].b[0], output);
			ez::serialize::f32(in.info[i].b[1], output);
			ez::serialize::f32(in.info[i].b[2], output);
		}

		ez::serialize::f32(in.positional_compliance, output);
		ez::serialize::f32(in.angular_compliance, output);

		ez::serialize::u8(static_cast<uint8_t>(in.components), output);
		if (in.has_target()) {
			ez::serialize::f32(in.target.angle, output);
			ez::serialize::f32(in.target.compliance, output);
		}
		if (in.has_limit()) {
			ez::serialize::f32(in.limit.min_angle, output);
			ez::serialize::f32(in.limit.max_angle, output);
			ez::serialize::f32(in.limit.compliance, output);
		}
	}
	const char* CHingeJoint::deserialize(const char* first, const char* last, CHingeJoint& out) {
		for (int i : iter::range(2)) {
			first = ez::deserialize::i32(first, last, out.info[i].id);

			first = ez::deserialize::f32(first, last, out.info[i].a[0]);
			first = ez::deserialize::f32(first, last, out.info[i].a[1]);
			first = ez::deserialize::f32(first, last, out.info[i].a[2]);

			first = ez::deserialize::f32(first, last, out.info[i].b[0]);
			first = ez::deserialize::f32(first, last, out.info[i].b[1]);
			first = ez::deserialize::f32(first, last, out.info[i].b[2]);
		}

		first = ez::deserialize::f32(first, last, out.positional_compliance);
		first = ez::deserialize::f32(first, last, out.angular_compliance);

		uint8_t bits = 0;
		first = ez::deserialize::u8(first, last, bits);
		if (bits & TargetBit) {
			first = ez::deserialize::f32(first, last, out.target.angle);
			first = ez::deserialize::f32(first, last, out.target.compliance);
		}
		if (bits & LimitBit) {
			first = ez::deserialize::f32(first, last, out.limit.min_angle);
			first = ez::deserialize::f32(first, last, out.limit.max_angle);
			first = ez::deserialize::f32(first, last, out.limit.compliance);
		}

		return first;
	}

	void CHingeJoint::eval(Engine& engine, real_t rdt2) const {
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

		std::array<vec3_t, 2> a;
		for (int i: iter::range(2)) {
			a[i] = bodies[i]->to_world_vector(info[i].a);
		}

		vec3_t n = glm::cross(a[0], a[1]);
		apply_angular_correction(bodies, n, angular_compliance * rdt2);

		if (components) {
			std::array<vec3_t, 2> b;
			for (int i: iter::range(2)) {
				b[i] = bodies[i]->to_world_vector(info[i].b);
			}

			// Always apply the target before the limit, so the limit can work properly.
			if (components & TargetBit) {
				vec3_t btarget = glm::rotate(glm::angleAxis(target.angle, a[0]), b[0]);
				apply_angular_correction(bodies, glm::cross(btarget, b[1]), target.compliance * rdt2);
			}
			if (components & LimitBit) {
				apply_angular_limit(bodies, a[0], b[0], b[1], limit.min_angle, limit.max_angle, limit.compliance * rdt2);
			}
		}
	}

	void CHingeJoint::remap(int32_t particle_offset, int32_t body_offset) {
		info[0].id += body_offset;
		info[1].id += body_offset;
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