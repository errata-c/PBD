#pragma once
#include <cinttypes>
#include <string>
#include <array>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

namespace pbd {
	class Engine;

	struct CHingeJoint {
		static void serialize(const CHingeJoint& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CHingeJoint& out);

		static constexpr Constraint Kind = Constraint::HingeJoint;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t offset);
		void transform(const Transform3& form);

		struct BodyInfo {
			int32_t id;
			// 'r' is relative attach point, 'a' is the hinge axis, 'b' is the up axis, must be perpendicular to 'a'
			glm::vec3 r, a, b;
		};

		std::array<BodyInfo, 2> info;
		// Possibly store different kinds of compliance, such as positional compliance, angular compliance
		float compliance;
	};
}