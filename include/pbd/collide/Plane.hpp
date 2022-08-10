#pragma once
#include <cinttypes>
#include <glm/vec3.hpp>
#include <pbd/Types.hpp>

namespace pbd {
	class Engine;
	struct CollidePlane {
		static constexpr Constraint Kind = Constraint::CollidePlane;

		CollidePlane();
		CollidePlane(int32_t id, const glm::vec3 & o, const glm::vec3& n);

		void reset(int32_t id, const glm::vec3& o, const glm::vec3& n);

		int32_t id;
		glm::vec3 origin, normal;

		void eval(Engine& engine) const;
	};

	static_assert(alignof(CollidePlane) == alignof(int32_t));
}