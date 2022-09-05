#pragma once
#include <vector>
#include <cinttypes>
#include <glm/vec3.hpp>

#include <pbd/engine/ExtractedRotation.hpp>

#include <pbd/common/Types.hpp>
#include <pbd/common/ConstraintList.hpp>

namespace pbd {
	class Engine {
	public:
		Engine();

		glm::vec3 gravity;
		float dt;
		int numSubsteps;
		float kineticFriction;
		float staticFriction;


		// prevPos is entirely for internal use.
		// Structure of arrays or array of structures?
		struct Particles {
			std::vector<glm::vec3> pos, prevPos, velocity, force;
			std::vector<float> invMass, radius;
			std::vector<int32_t> flags;
		} particle;

		ConstraintList constraints;
		
		void reserve(int64_t count);
		int64_t size() const noexcept;
		int64_t numParticles() const noexcept;
		int64_t numConstraints() const noexcept;

		void solve();

		int32_t addParticle(const glm::vec3& position, const glm::vec3& velocity, float invMass, float radius);
		int32_t addParticle(const glm::vec3& position, float invMass, float radius);
		
		template<typename T>
		int64_t addConstraint(const T& cval) {
			return constraints.add(cval);
		}
	private:
		void predictPositions(float sdt);
		void applyConstraints(float sdt);
		void updateParticles(float sdt);
	};
}