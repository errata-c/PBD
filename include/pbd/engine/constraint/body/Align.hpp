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
	Connect two bodies together, force them to be perfectly aligned.
	This essentially causes them to act as a unit.
	*/
	struct CAlign {
		static void serialize(const CAlign& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CAlign& out);

		static constexpr ConstraintType Kind = ConstraintType::Align;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t particle_offset, int32_t body_offset);
		void transform(const Transform3& form);

		struct BodyInfo {
			int32_t id;
			
			// Relative attachment point
			glm::vec3 r;
		};

		std::array<BodyInfo, 2> info;

		// The relative rotation from the first body to the second body's desired orientation.
		glm::quat alignment;

		float compliance;
	};
}