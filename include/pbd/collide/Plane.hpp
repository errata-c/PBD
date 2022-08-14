#pragma once
#include <cinttypes>
#include <glm/vec3.hpp>
#include <pbd/common/Types.hpp>

namespace pbd {
	class Engine;
	struct CollidePlane {
		static constexpr Constraint Kind = Constraint::CollidePlane;

		int32_t id;
		glm::vec3 origin, normal;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t offset);
	};

	static_assert(alignof(CollidePlane) == alignof(int32_t));
}