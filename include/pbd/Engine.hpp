#pragma once
#include <vector>
#include <cinttypes>
#include <glm/vec3.hpp>

#include <pbd/collide/Particle.hpp>
#include <pbd/collide/Plane.hpp>

#include <pbd/constraint/Distance.hpp>
#include <pbd/constraint/TetraVolume.hpp>
#include <pbd/constraint/NHTetraVolume.hpp>

#include <pbd/ExtractedRotation.hpp>
#include <pbd/Types.hpp>

namespace pbd {
	class Engine {
	public:
		Engine();

		glm::vec3 gravity;
		float dt;
		int numSubsteps;
		float kineticFriction;
		float staticFriction;

		// Map for objects?

		// prevPos is entirely for internal use.
		
		// Structure of arrays or array of structures?
		struct Particles {
			std::vector<glm::vec3> pos, prevPos, velocity, force;
			std::vector<float> invMass, radius;
			std::vector<int32_t> flags;
		} particle;

		struct CVariant {
			Constraint kind;
			int64_t index;
		};
		std::vector<CVariant> constraints;
		std::vector<int32_t> cdata;
		
		void reserve(int64_t count);
		int64_t size() const;

		void solve();

		int32_t addParticle(const glm::vec3& position, const glm::vec3& velocity, float invMass, float radius);
		int32_t addParticle(const glm::vec3& position, float invMass, float radius);
		
		template<typename T>
		int64_t addConstraint(const T& cval) {
			int64_t index = cdata.size();
			Constraint kind = T::Kind;

			const int32_t* ival = (const int32_t*)&cval;
			for (size_t i = 0; i < (sizeof(T) / sizeof(int32_t)); ++i) {
				cdata.push_back(ival[i]);
			}
			constraints.push_back(CVariant{kind, index});

			return constraints.size()-1;
		}
	private:
		void predictPositions(float sdt);
		void applyConstraints(float sdt);
		void updateParticles(float sdt);
	};
}