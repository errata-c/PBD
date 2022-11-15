#pragma once
#include <cinttypes>
#include <array>
#include <string>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

#include <pbd/engine/constraint/ConstraintType.hpp>

namespace pbd {
	class Engine;

	struct CTetra {
		static void serialize(const CTetra& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CTetra& out);

		static constexpr ConstraintType Kind = ConstraintType::Tetra;
		static constexpr std::array<std::array<int32_t, 3>, 4> faceOrder{ {
			{0,1,3},
			{0,2,3},
			{0,3,1},
			{0,1,2}
		}};

		std::array<int32_t, 4> ids;
		float volume;
		float compliance;

		void eval(Engine & engine, float rdt2) const;

		void remap(int32_t particle_offset, int32_t body_offset);
		void transform(const Transform3& form);
	};

	static_assert(alignof(CTetra) == alignof(int32_t));
}