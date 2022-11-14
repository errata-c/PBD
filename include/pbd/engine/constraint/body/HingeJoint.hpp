#pragma once
#include <cinttypes>
#include <string>
#include <array>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

#include <pbd/engine/constraint/ConstraintType.hpp>

namespace pbd {
	class Engine;
	class RigidBody;

	struct CHingeJoint {
		static void serialize(const CHingeJoint& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CHingeJoint& out);

		static constexpr ConstraintType Kind = ConstraintType::HingeJoint;
		
		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t offset);
		void transform(const Transform3& form);

		bool has_limit() const noexcept;
		bool has_target() const noexcept;

		void enable_limit(bool val);
		void enable_target(bool val);

		struct BodyInfo {
			int32_t id;
			// 'r' is relative attach point, 'a' is the hinge axis, 'b' is the up axis, must be perpendicular to 'a'
			glm::vec3 r, a, b;
		};

		std::array<BodyInfo, 2> info;
		// Possibly store different kinds of compliance, such as positional compliance, angular compliance
		float positional_compliance, angular_compliance;

		static constexpr uint32_t
			TargetBit = 1,
			LimitBit = 2;
		uint32_t components;

		struct Target {
			float angle;
			float compliance;
		} target;
		struct Limit {
			float min_angle, max_angle;
			float compliance;
		} limit;
	};

	
}