#pragma once
#include <cinttypes>
#include <string>
#include <array>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

namespace pbd {
	class Engine;

	struct CTwistLimit {
		float min_angle, max_angle;
		float compliance;
	};
	struct CSwingLimit {
		float min_angle, max_angle;
		float compliance;
	};
	struct CSphereTarget {
		glm::quat orientation;
		float compliance;
	};

	// Swing and twist are defined in terms of the second body relative to the first body.
	struct CSphereJoint {
		static void serialize(const CSphereJoint& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CSphereJoint& out);

		static constexpr Constraint Kind = Constraint::SphereJoint;

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

		uint32_t components;

		CSphereTarget target;
		CTwistLimit twist_limit;
		CSwingLimit swing_limit;
	};
}