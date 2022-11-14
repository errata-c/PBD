#pragma once
#include <pbd/engine/collider/ColliderType.hpp>
#include <array>

namespace pbd {
	class RigidBody;
	class Particle;
	class Engine;

	class Collider {
	public:
		Collider(const Collider&) noexcept = default;
		Collider& operator=(const Collider&) noexcept = default;
		Collider(Collider&&) noexcept = default;
		Collider& operator=(Collider&&) noexcept = default;
		~Collider() = default;

		Collider(int32_t i0, int32_t i1, CollideType _type);
		
		void position_solve(Engine& engine, float dt);
		void velocity_solve(Engine& engine, float dt);
		
		CollideType type;
		std::array<int32_t, 2> ids;
	};
}