#pragma once
#include <cinttypes>
#include <string>
#include <array>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

#include <pbd/engine/constraint/ConstraintType.hpp>

namespace pbd {
	class Engine;

	/*
	Connect a rigid body and a particle.
	*/
	struct CAttachParticle {
		static void serialize(const CAttachParticle& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CAttachParticle& out);

		static constexpr ConstraintType Kind = ConstraintType::AttachParticle;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t particle_offset, int32_t body_offset);
		void transform(const Transform3& form);

		int32_t body_id, particle_id;

		// Body relative attachment point
		glm::vec3 r;

		float compliance;
	};
}