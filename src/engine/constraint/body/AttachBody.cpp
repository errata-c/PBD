#include <pbd/engine/constraint/body/AttachBody.hpp>
#include <pbd/engine/Engine.hpp>

#include <pbd/engine/constraint/body/Utils.hpp>
#include <cppitertools/itertools.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	void CAttachBody::eval(Engine& engine, float rdt2) const {
		float alpha = (compliance * rdt2);

		// First calculate the positional correction.
		std::array<RigidBody*, 2> b;
		b[0] = &engine.bodies.list[info[0].id];
		b[1] = &engine.bodies.list[info[1].id];

		apply_positional_correction(b, { &info[0].r, &info[1].r }, b[1]->to_world(info[1].r) - b[0]->to_world(info[0].r), alpha);
	}

	void CAttachBody::remap(int32_t offset) {
		
	}
	void CAttachBody::transform(const Transform3& form) {

	}
}