#pragma once

namespace pbd {
	class Engine3D;

	struct Distance3D {
		float initialLength;
		int p0, p1;

		void eval(Engine3D& engine) const;
	};

	struct Distance2D {
		float length;
		int p0, p1;

		void eval(Engine3D& engine) const;
	};
}