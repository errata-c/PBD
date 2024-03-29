#include <pbd/engine/constraint/body/PrismaticJoint.hpp>
#include <pbd/engine/constraint/body/Utils.hpp>

#include <pbd/engine/Engine.hpp>

#include <cppitertools/itertools.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>


namespace pbd {
	void CPrismaticJoint::serialize(const CPrismaticJoint& in, std::string& output) {
		
	}
	const char* CPrismaticJoint::deserialize(const char* first, const char* last, CPrismaticJoint& out) {
		return first;
	}

	void CPrismaticJoint::eval(Engine& engine, real_t rdt2) const {
		// First calculate the positional correction.
		std::array<RigidBody*, 2> bodies;
		bodies[0] = &engine.bodies.list[info[0].id];
		bodies[1] = &engine.bodies.list[info[1].id];

		std::array<vec3_t, 2> r;
		for (int i: iter::range(2)) {
			r[i] = bodies[i]->to_world(info[i].r);
		}
		vec3_t a = bodies[0]->to_world_vector(axis);


	}

	void CPrismaticJoint::remap(int32_t particle_offset, int32_t body_offset) {
		info[0].id += body_offset;
		info[1].id += body_offset;
	}
	void CPrismaticJoint::transform(const Transform3& form) {

	}

	bool CPrismaticJoint::has_limit() const noexcept {
		return true;
	}
	bool CPrismaticJoint::has_target() const noexcept {
		return components & TargetBit;
	}

	void CPrismaticJoint::enable_limit(bool val) {
		
	}
	void CPrismaticJoint::enable_target(bool val) {
		components = (components & ~TargetBit);
		if (val) {
			components |= TargetBit;
		}
	}
}