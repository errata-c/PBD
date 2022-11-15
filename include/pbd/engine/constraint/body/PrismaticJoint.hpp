#pragma once
#include <cinttypes>
#include <string>
#include <array>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

#include <pbd/engine/constraint/ConstraintType.hpp>

namespace pbd {
	class Engine;

	struct CPrismaticJoint {
		static void serialize(const CPrismaticJoint& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CPrismaticJoint& out);

		static constexpr ConstraintType Kind = ConstraintType::PrismaticJoint;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t particle_offset, int32_t body_offset);
		void transform(const Transform3& form);

		bool has_limit() const noexcept;
		bool has_target() const noexcept;

		void enable_limit(bool val);
		void enable_target(bool val);

		struct BodyInfo {
			int32_t id;
			glm::vec3 r;
		};

		// Id and body relative attachment point
		std::array<BodyInfo, 2> info;

		// The axis of linear motion
		glm::vec3 axis;

		// The relative rotation from the first body to the second body.		
		glm::quat alignment;

		// Inverse stiffness
		float positional_compliance, angular_compliance;
		
		static constexpr uint32_t
			TargetBit = 1;
		uint32_t components;

		struct Target {
			float position;
			float compliance;
		} target;

		struct Limit {
			float min, max;
			float compliance;
		} limit;
	};
}