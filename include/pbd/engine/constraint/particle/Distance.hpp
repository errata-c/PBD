#pragma once
#include <cinttypes>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

#include <pbd/engine/constraint/ConstraintType.hpp>

namespace pbd {
	class Engine;

	struct CDistance {
		static void serialize(const CDistance& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CDistance& out);

		static constexpr ConstraintType Kind = ConstraintType::Distance;

		void eval(Engine& engine, real_t rdt2) const;
		
		void remap(int32_t particle_offset, int32_t body_offset);
		void transform(const Transform3& form);

		int32_t p0, p1;
		real_t length;
		real_t compliance;
	};

	static_assert(alignof(CDistance) == alignof(int32_t));
}