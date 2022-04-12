#pragma once
#include <array>

namespace pbd {
	class Engine3D;

	struct TetraVolume3D {
		static constexpr std::array<std::array<int, 3>, 4> faceOrder{ {
			{0,1,3},
			{0,2,3},
			{0,3,1},
			{0,1,2}
		}};
		std::array<int, 4> ids;
		float initialVolume;

		void eval(Engine3D & engine) const;
	};
}