#pragma once

namespace pbd {
	class Engine;

	struct Distance {
		int p0, p1;
		float initialLength;

		void eval(Engine& engine) const;
	};
}