#include <pbd/engine/constraint/body/Fixed.hpp>
#include <pbd/engine/Engine.hpp>

#include <pbd/engine/constraint/body/Utils.hpp>
#include <cppitertools/itertools.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	void CFixed::serialize(const CFixed& in, std::string& output) {
		for (int i : iter::range(2)) {
			ez::serialize::i32(in.info[i].id, output);
			ez::serialize::f32(in.info[i].r[0], output);
			ez::serialize::f32(in.info[i].r[1], output);
			ez::serialize::f32(in.info[i].r[2], output);
		}

		ez::serialize::f32(in.alignment[0], output);
		ez::serialize::f32(in.alignment[1], output);
		ez::serialize::f32(in.alignment[2], output);
		ez::serialize::f32(in.alignment[3], output);

		ez::serialize::f32(in.positional_compliance, output);
		ez::serialize::f32(in.angular_compliance, output);
	}
	const char* CFixed::deserialize(const char* first, const char* last, CFixed& out) {
		for (int i : iter::range(2)) {
			first = ez::deserialize::i32(first, last, out.info[i].id);
			first = ez::deserialize::f32(first, last, out.info[i].r[0]);
			first = ez::deserialize::f32(first, last, out.info[i].r[1]);
			first = ez::deserialize::f32(first, last, out.info[i].r[2]);
		}

		first = ez::deserialize::f32(first, last, out.alignment[0]);
		first = ez::deserialize::f32(first, last, out.alignment[1]);
		first = ez::deserialize::f32(first, last, out.alignment[2]);
		first = ez::deserialize::f32(first, last, out.alignment[3]);

		first = ez::deserialize::f32(first, last, out.positional_compliance);
		first = ez::deserialize::f32(first, last, out.angular_compliance);

		return first;
	}

	void CFixed::eval(Engine& engine, float rdt2) const {
		// First calculate the positional correction.
		std::array<RigidBody*, 2> b;
		b[0] = &engine.bodies.list[info[0].id];
		b[1] = &engine.bodies.list[info[1].id];

		apply_positional_correction(b, {&info[0].r, &info[1].r}, b[1]->to_world(info[1].r) - b[0]->to_world(info[0].r), positional_compliance * rdt2);

		// Calculate the angular correction
		glm::quat tmp = (b[0]->orientation * alignment) * glm::conjugate(b[1]->orientation);
		apply_angular_correction(b, 2.f * glm::vec3(tmp.x, tmp.y, tmp.z), angular_compliance * rdt2);
	}

	void CFixed::remap(int32_t particle_offset, int32_t body_offset) {
		info[0].id += body_offset;
		info[1].id += body_offset;
	}
	void CFixed::transform(const Transform3& form) {

	}
}