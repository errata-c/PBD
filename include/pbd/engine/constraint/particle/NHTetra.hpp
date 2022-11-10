#pragma once
#include <cinttypes>
#include <array>
#include <string>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

#include <glm/mat3x3.hpp>

namespace pbd {
	class Engine;

	// More costly than the other Tetra constraint, but does not require distance constraints.
	// Hydrostatic constraint is volume, Deviatoric constraint is essentially shear.
	struct CNHTetra {
		static void serialize(const CNHTetra& in, std::string& output);
		static const char* deserialize(const char* first, const char* last, CNHTetra& out);

		static constexpr Constraint Kind = Constraint::NHTetra;

		CNHTetra();
		CNHTetra(const std::array<int32_t, 4> _ids, const glm::mat3& _irest, float hydro, float devia);
		CNHTetra(
			const std::array<int32_t, 4> _ids, 
			const glm::vec3& p0,
			const glm::vec3& p1,
			const glm::vec3& p2,
			const glm::vec3& p3,
			float hydro, 
			float devia);
		
		std::array<int32_t, 4> ids;
		glm::mat3 inv_rest;
		float hydrostatic_compliance, deviatoric_compliance;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t offset);
		void transform(const Transform3& form);
	};

	static_assert(alignof(CNHTetra) == alignof(int32_t));
}