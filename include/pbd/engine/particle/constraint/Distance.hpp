#pragma once
#include <cinttypes>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

namespace pbd {
	class Engine;

	struct CDistance {
		static void serialize(const CDistance& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CDistance& out);

		static constexpr Constraint Kind = Constraint::Distance;

		void eval(Engine& engine, float rdt2) const;
		
		void remap(int32_t offset);
		void transform(const Transform3& form);

		int32_t p0, p1;
		float length;
		float compliance;
	};

	static_assert(alignof(CDistance) == alignof(int32_t));
	static_assert(SizeOf(Constraint::Distance) == sizeof(CDistance));
}