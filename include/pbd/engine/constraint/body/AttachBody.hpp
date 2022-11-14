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
	Connect a rigid body to another rigid body.
	*/
	struct CAttachBody {
		static void serialize(const CAttachBody& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CAttachBody& out);

		static constexpr ConstraintType Kind = ConstraintType::AttachBody;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t offset);
		void transform(const Transform3& form);

		struct BodyInfo {
			int32_t id;

			// Relative attachment point
			glm::vec3 r;
		};

		std::array<BodyInfo, 2> info;
		float compliance;
	};
}