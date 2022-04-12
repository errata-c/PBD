#pragma once

namespace pbd {
	class Engine3D;
	
	// Very similar to distance constraint, but only active when >= 0
	struct CollideParticle3D {
		int p0, p1;
		float distance;

		void eval(Engine3D& engine) const;
	};
}