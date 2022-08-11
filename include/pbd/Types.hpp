#pragma once

namespace pbd {
	enum class Constraint {
		Distance,
		TetraVolume,
		NHTetraVolume,

		CollideParticle,
		CollidePlane,

		CollideParticleCompliant,
		CollidePlaneCompliant,
	};
}