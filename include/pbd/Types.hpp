#pragma once

namespace pbd {
	enum class Constraint {
		Distance,
		TetraVolume,

		CollideParticle,
		CollidePlane,
	};
}