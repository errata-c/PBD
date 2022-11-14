#include <pbd/engine/collider/Collider.hpp>

namespace pbd {
	void Collider::position_solve(Engine& engine, float dt) {
		// This is the constraint formulation of the collider, to be run during the positional constraint phase.


	}
	void Collider::velocity_solve(Engine& engine, float dt) {
		// This is secondary velocity pass, to be run after solving the initial constraints.

		// Its not entirely clear whether or not we need to recompute contacts, or if we are supposed to reuse the previous ones.
	}
}