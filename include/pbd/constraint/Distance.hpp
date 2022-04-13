#pragma once

namespace pbd {
	class Engine3D;

	struct Distance3D {
		int p0, p1;
		float initialLength;

		void eval(Engine3D& engine) const;
	};

	struct Distance2D {
		int p0, p1;
		float length;

		void eval(Engine3D& engine) const;
	};
}