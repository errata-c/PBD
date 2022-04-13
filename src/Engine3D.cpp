#include <pbd/Engine3D.hpp>
#include <algorithm>

namespace pbd {
	Engine3D::Engine3D()
		: friction(1)
		, gravity(0,-9.8,0)
		, numSubsteps(4)
		, dt(1.0 / 60.0)
	{}

	void Engine3D::resize(int count) {
		pos.resize(count);
		prevPos.resize(count);
		velocity.resize(count);
		invMass.resize(count);
	}
	int Engine3D::size() const {
		return static_cast<int>(pos.size());
	}

	void Engine3D::solve() {
		float sdt = dt / float(numSubsteps);
		for (int i = 0; i < numSubsteps; ++i) {
			preSolve(sdt);
			midSolve(sdt);
			postSolve(sdt);
		}
	}

	void Engine3D::preSolve(float sdt) {
		prevPos.assign(pos.begin(), pos.end());

		for (int i = 0; i < pos.size(); ++i) {
			if (invMass[i] < 1e-5f) {
				continue;
			}

			velocity[i] += gravity * sdt;
			pos[i] += velocity[i] * sdt;

			if (pos[i].y < 0.f) {
				pos[i] = prevPos[i];
				pos[i].y = 0.f;
			}
		}
	}
	void Engine3D::midSolve(float sdt) {
		// Run all the constraints, we need to have the constraints added to this
		for (const Distance3D & constraint : distances) {
			constraint.eval(*this);
		}
		for (const TetraVolume3D& constraint : tetras) {
			constraint.eval(*this);
		}
		// Universal colliders
		/*
		for (const CollidePlane& plane : planes) {
			for (int i = 0; i < size(); ++i) {
				plane.eval(*this, i);
			}
		}
		*/
	}
	void Engine3D::postSolve(float sdt) {
		sdt = 1.f / sdt;

		for (int i = 0; i < pos.size(); ++i) {
			if (invMass[i] < 1e-5f) {
				continue;
			}

			velocity[i] = (pos[i] - prevPos[i]) * sdt;
		}
	}
}