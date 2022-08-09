#pragma once

namespace pbd {
	class Engine;
	
	// Very similar to distance constraint, but only active when >= 0
	struct CollideParticle {
		int p0, p1;
		float distance;

		void eval(Engine& engine) const;
	};
}