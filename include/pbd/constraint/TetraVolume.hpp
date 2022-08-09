#pragma once
#include <array>

namespace pbd {
	class Engine;

	struct TetraVolume {
		static constexpr std::array<std::array<int, 3>, 4> faceOrder{ {
			{0,1,3},
			{0,2,3},
			{0,3,1},
			{0,1,2}
		}};
		std::array<int, 4> ids;
		float initialVolume;

		void eval(Engine & engine) const;
	};
}