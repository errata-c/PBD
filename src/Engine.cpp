#include <pbd/Engine.hpp>
#include <algorithm>

namespace pbd {
	Engine::Engine()
		: friction(1)
		, gravity(0,-9.8,0)
		, numSubsteps(4)
		, dt(1.0 / 60.0)
	{}

	void Engine::resize(int64_t count) {
		particle.pos.resize(count);
		particle.prevPos.resize(count);
		particle.velocity.resize(count);
		particle.invMass.resize(count);
	}
	int64_t Engine::size() const {
		return static_cast<int64_t>(particle.pos.size());
	}

	void Engine::solve() {
		float sdt = dt / float(numSubsteps);
		for (int i = 0; i < numSubsteps; ++i) {
			preSolve(sdt);
			midSolve(sdt);
			postSolve(sdt);
		}
	}

	void Engine::preSolve(float sdt) {
		particle.prevPos.assign(particle.pos.begin(), particle.pos.end());

		for (int64_t i = 0, count = size(); i < count; ++i) {
			if (particle.invMass[i] < 1e-5f) {
				continue;
			}
			glm::vec3 & velocity = particle.velocity[i];
			glm::vec3 & position = particle.pos[i];
			const glm::vec3 & prevPosition = particle.prevPos[i];

			velocity += gravity * sdt;
			position += velocity * sdt;

			if (position.y < 0.f) {
				position = prevPosition;
				position.y = 0.f;
			}
		}
	}
	void Engine::midSolve(float sdt) {
		// Run all the constraints, we need to have the constraints added to this

		for (CVariant & cvar: constraints) {
			switch (cvar.kind) {
			case Constraint::Distance:
				((ConstraintDistance*)(&cdata[cvar.index]))->eval(*this);
				break;
			case Constraint::TetraVolume:
				((ConstraintTetraVolume*)(&cdata[cvar.index]))->eval(*this);
				break;
			case Constraint::CollideParticle:
				((CollideParticle*)(&cdata[cvar.index]))->eval(*this);
				break;
			case Constraint::CollidePlane:
				((CollidePlane*)(&cdata[cvar.index]))->eval(*this);
				break;
			}
		}
	}
	void Engine::postSolve(float sdt) {
		sdt = 1.f / sdt;

		for (int64_t i = 0, count = size(); i < count; ++i) {
			if (particle.invMass[i] < 1e-5f) {
				continue;
			}

			particle.velocity[i] = (particle.pos[i] - particle.prevPos[i]) * sdt;
		}
	}
}