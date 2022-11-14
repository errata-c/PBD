#include <pbd/engine/constraint/body/Align.hpp>
#include <pbd/engine/Engine.hpp>

#include <pbd/engine/constraint/body/Utils.hpp>
#include <cppitertools/itertools.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	void CAlign::serialize(const CAlign& in, std::string& output) {
		for (int i: iter::range(2)) {
			ez::serialize::i32(in.info[i].id, output);
			ez::serialize::f32(in.info[i].r[0], output);
			ez::serialize::f32(in.info[i].r[1], output);
			ez::serialize::f32(in.info[i].r[2], output);
		}
		
	}
	const char* CAlign::deserialize(const char* first, const char* last, CAlign& out) {
		return first;
	}

	void CAlign::eval(Engine& engine, float rdt2) const {
		float alpha = (compliance * rdt2);

		// First calculate the positional correction.
		std::array<RigidBody*, 2> b;
		b[0] = &engine.bodies.list[info[0].id];
		b[1] = &engine.bodies.list[info[1].id];

		apply_positional_correction(b, {&info[0].r, &info[1].r}, b[1]->to_world(info[1].r) - b[0]->to_world(info[0].r), alpha);

		// Calculate the angular correction
		glm::quat tmp = (b[0]->orientation * alignment) * glm::conjugate(b[1]->orientation);
		apply_angular_correction(b, 2.f * glm::vec3(tmp.x, tmp.y, tmp.z), alpha);
	}

	void CAlign::remap(int32_t offset) {
		info[0].id += offset;
		info[1].id += offset;
	}
	void CAlign::transform(const Transform3& form) {
		
	}
}