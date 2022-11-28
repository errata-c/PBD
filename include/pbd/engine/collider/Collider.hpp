#pragma once
#include <pbd/engine/collider/ColliderType.hpp>
#include <pbd/engine/collider/Collision.hpp>
#include <array>
#include <optional>

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

		Collider(int32_t i0, int32_t i1, ColliderType _type);
		
		void position_solve(Engine& engine, real_t dt);
		void velocity_solve(Engine& engine, real_t dt);
		
		ColliderType type;
		std::array<int32_t, 2> ids;
	private:
		std::optional<Collision> collide(Engine& engine) const;
	};
}