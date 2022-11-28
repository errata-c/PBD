#pragma once
#include <cinttypes>
#include <string>
#include <array>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

#include <pbd/engine/constraint/ConstraintType.hpp>

namespace pbd {
	class Engine;

	// Swing and twist are defined in terms of the second body relative to the first body.
	struct CSphereJoint {
		static void serialize(const CSphereJoint& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CSphereJoint& out);

		static constexpr ConstraintType Kind = ConstraintType::SphereJoint;

		void eval(Engine& engine, real_t rdt2) const;

		void remap(int32_t particle_offset, int32_t body_offset);
		void transform(const Transform3& form);


		struct TwistLimit {
			real_t min_angle, max_angle;
			real_t compliance;
		};
		struct SwingLimit {
			real_t min_angle, max_angle;
			real_t compliance;
		};
		struct Target {
			quat_t orientation;
			real_t compliance;
		};
		static constexpr uint32_t
			TargetBit = 4,
			TwistLimitBit = 2,
			SwingLimitBit = 1;

		struct BodyInfo {
			int32_t id;
			// 'r' is relative attach point, 'a' is the hinge axis, 'b' is the up axis, must be perpendicular to 'a'
			vec3_t r, a, b;
		};

		std::array<BodyInfo, 2> info;
		// Possibly store different kinds of compliance, such as positional compliance, angular compliance
		real_t compliance;

		uint32_t components;
		
		TwistLimit twist_limit;
		SwingLimit swing_limit;
		Target target;
	};
}