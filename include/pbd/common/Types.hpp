#pragma once
#include <cinttypes>

namespace pbd {
	enum class Constraint: uint32_t {
		Distance = 0 | (16 << 8) | (2 << 16),
		TetraVolume = 1 | (24 << 8) | (4 << 16),
		NHTetraVolume = 2 | (56 << 8) | (4 << 16),

		CollideParticle = 3 | (8 << 8) | (2 << 16),
		CollidePlane = 4 | (28 << 8) | (1 << 16),

		CollideParticleCompliant = 5 | (12 << 8) | (2 << 16),
		CollidePlaneCompliant = 6 | (32 << 8) | (1 << 16),
	};

	constexpr size_t SizeOf(Constraint type) noexcept {
		return (static_cast<uint32_t>(type) >> 8) & 0xFF;
	}
	constexpr size_t NumIds(Constraint type) noexcept {
		return (static_cast<uint32_t>(type) >> 16) & 0xFF;
	}
}