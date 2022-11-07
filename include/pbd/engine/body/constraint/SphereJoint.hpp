#pragma once
#include <cinttypes>
#include <string>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

namespace pbd {
	class Engine;

	struct CSphereJoint {
		static void serialize(const CSphereJoint& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CSphereJoint& out);

		static constexpr Constraint Kind = Constraint::SphereJoint;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t offset);
		void transform(const Transform3& form);

		int32_t p0, p1;
		float length;
		float compliance;
	};
}