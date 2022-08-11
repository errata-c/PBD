#include <pbd/Engine.hpp>
#include <algorithm>
#include <limits>

namespace pbd {
	static constexpr size_t ParticleLimit = std::numeric_limits<int32_t>::max();

	Engine::Engine()
		: staticFriction(1.f)
		, kineticFriction(1.f)
		, gravity(0,-9.8,0)
		, numSubsteps(4)
		, dt(1.0 / 60.0)
	{}

	void Engine::resize(int64_t count) {
		assert(count < ParticleLimit);
		particle.pos.resize(count);
		particle.prevPos.resize(count);
		particle.velocity.resize(count);
		particle.invMass.resize(count);
	}
	int64_t Engine::size() const {
		return static_cast<int64_t>(particle.pos.size());
	}

	int32_t Engine::addParticle(const glm::vec3& position, const glm::vec3& velocity, float invMass, float radius) {
		assert(size() < ParticleLimit);
		int32_t id = static_cast<int32_t>(size());
		particle.pos.push_back(position);
		particle.velocity.push_back(velocity);
		particle.invMass.push_back(invMass);
		particle.radius.push_back(radius);

		return id;
	}
	int32_t Engine::addParticle(const glm::vec3& position, float invMass, float radius) {
		assert(size() < ParticleLimit);
		int32_t id = static_cast<int32_t>(size());

		particle.pos.push_back(position);
		particle.velocity.push_back(glm::vec3(0.f));
		particle.invMass.push_back(invMass);
		particle.radius.push_back(radius);

		return id;
	}

	// Run one iteration of the solver
	void Engine::solve() {
		// Collision constraints would be generated here!
		// If needed we can predict where the positions will be roughly, then find the collisions.

		
		// Run the iteration substeps.
		float sdt = dt / float(numSubsteps);
		for (int i = 0; i < numSubsteps; ++i) {
			predictPositions(sdt);
			applyConstraints(sdt);
			updateParticles(sdt);
		}
	}

	void Engine::predictPositions(float sdt) {
		// Store these starting positions for comparison in the constraints.
		particle.prevPos.assign(particle.pos.begin(), particle.pos.end());

		for (int64_t i = 0, count = size(); i < count; ++i) {
			float imass = particle.invMass[i];
			if (imass < 1e-5f) {
				continue;
			}
			// The original XPBD paper does not update the velocity here, but a later review paper does.
			// I'm going to trust the later paper, as its more comprehensive.

			glm::vec3 & velocity = particle.velocity[i];
			glm::vec3 & position = particle.pos[i];

			// Right now gravity is the only external force, I'll add more later.
			velocity = velocity + sdt * gravity * imass; // Mass?
			position = position + sdt * velocity;
		}
	}
	void Engine::applyConstraints(float sdt) {
		// Run all the constraints, we need to have the constraints added to this

		float rdt2 = 1.f / (sdt * sdt);

		for (CVariant & cvar: constraints) {
			switch (cvar.kind) {
			case Constraint::Distance:
				((ConstraintDistance*)(&cdata[cvar.index]))->eval(*this, rdt2);
				break;
			case Constraint::TetraVolume:
				((ConstraintTetraVolume*)(&cdata[cvar.index]))->eval(*this, rdt2);
				break;
			case Constraint::NHTetraVolume:
				((ConstraintNHTetraVolume*)(&cdata[cvar.index]))->eval(*this, rdt2);
				break;
			case Constraint::CollideParticle:
				((CollideParticle*)(&cdata[cvar.index]))->eval(*this, rdt2);
				break;
			case Constraint::CollidePlane:
				((CollidePlane*)(&cdata[cvar.index]))->eval(*this, rdt2);
				break;
			}
		}
	}
	void Engine::updateParticles(float sdt) {
		sdt = 1.f / sdt;

		for (int64_t i = 0, count = size(); i < count; ++i) {
			/// We don't have to check the mass here, if they have infinite mass the delta will be zero anyways.
			//if (particle.invMass[i] < 1e-5f) {
			//	continue;
			//}

			particle.velocity[i] = (particle.pos[i] - particle.prevPos[i]) * sdt;
		}
	}
}