#pragma once
#include <cinttypes>
#include <array>
#include <string>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

#include <pbd/engine/constraint/ConstraintType.hpp>

#include <glm/mat3x3.hpp>

namespace pbd {
	class Engine;

	// More costly than the other Tetra constraint, but does not require distance constraints.
	// Hydrostatic constraint is volume, Deviatoric constraint is essentially shear.
	struct CNHTetra {
		static void serialize(const CNHTetra& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CNHTetra& out);

		static constexpr ConstraintType Kind = ConstraintType::NHTetra;

		CNHTetra();
		CNHTetra(const std::array<int32_t, 4> _ids, const glm::mat3& _irest, real_t hydro, real_t devia);
		CNHTetra(
			const std::array<int32_t, 4> _ids, 
			const vec3_t& p0,
			const vec3_t& p1,
			const vec3_t& p2,
			const vec3_t& p3,
			real_t hydro, 
			real_t devia);
		
		std::array<int32_t, 4> ids;
		glm::mat3 inv_rest;
		real_t hydrostatic_compliance, deviatoric_compliance;

		void eval(Engine& engine, real_t rdt2) const;

		void remap(int32_t particle_offset, int32_t body_offset);
		void transform(const Transform3& form);
	};

	static_assert(alignof(CNHTetra) == alignof(int32_t));
}