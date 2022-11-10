#pragma once
#include <cinttypes>
#include <string>
#include <array>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

namespace pbd {
	class Engine;
	class RigidBody;

	struct CHingeTarget {
		float angle;
		float compliance;
	};
	struct CHingeLimit {
		float min_angle, max_angle;
		float compliance;
	};
	

	struct CHingeJoint {
		static void serialize(const CHingeJoint& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CHingeJoint& out);

		static constexpr Constraint Kind = Constraint::HingeJoint;

		template<typename ...Ts>
		static CHingeJoint From(
			int32_t id0,
			const Transform3& rb0,
			int32_t id1,
			const Transform3& rb1,
			const glm::vec3& attach,
			const glm::vec3& hinge_axis, Ts&&... args) {

		}
	private:
		template<typename ...Ts>
		static void Init(CHingeJoint & joint, const CHingeLimit& limit, Ts&&... args) {
			joint.limit = limit;
			joint.enable_limit(true);
			Init(joint, std::forward<Ts>(args)...);
		}
		template<typename ...Ts>
		static void Init(CHingeJoint& joint, const CHingeTarget& target, Ts&&... args) {
			joint.target = target;
			joint.enable_target(true);
			Init(joint, std::forward<Ts>(args)...);
		}
		static void Init(CHingeJoint& joint) {}
	public:
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

		uint32_t components;

		CHingeTarget target;
		CHingeLimit limit;
	};

	
}