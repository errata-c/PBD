#pragma once
#include <cinttypes>
#include <string>
#include <array>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

namespace pbd {
	class Engine;

	/*
	Connect two bodies together, force them to be perfectly aligned.
	This essentially causes them to act as a unit.
	*/
	struct CAlign {
		static void serialize(const CAlign& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CAlign& out);

		static constexpr Constraint Kind = Constraint::Align;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t offset);
		void transform(const Transform3& form);

		struct BodyInfo {
			int32_t id;
			
			// Relative attachment point
			glm::vec3 r;
		};

		// The relative rotation from the first body to the second body.
		glm::quat alignment;

		std::array<BodyInfo, 2> info;
		float compliance;
	};
}