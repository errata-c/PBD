#include <pbd/engine/collider/ColliderType.hpp>
#include <cmath>

namespace pbd {
	ColliderType particle_particle_collider() noexcept {
		return ColliderType::ParticleParticle;
	}
	ColliderType shape_particle_collider(Shape lh) noexcept {
		return static_cast<ColliderType>(
			static_cast<uint32_t>(lh) + static_cast<uint32_t>(ColliderType::CapsuleParticle)
		);
	}
	ColliderType shape_shape_collider(Shape lh, Shape rh) noexcept {
		static constexpr uint32_t offsets[4] = {
			static_cast<uint32_t>(ColliderType::CapsuleCapsule),
			static_cast<uint32_t>(ColliderType::CylinderCylinder),
			static_cast<uint32_t>(ColliderType::OBBOBB),
			static_cast<uint32_t>(ColliderType::SphereSphere),
		};

		uint32_t first = std::min(static_cast<uint32_t>(lh), static_cast<uint32_t>(rh));
		uint32_t second = std::max(static_cast<uint32_t>(lh), static_cast<uint32_t>(rh));

		return static_cast<ColliderType>(
			offsets[first] + second
		);
	}
}