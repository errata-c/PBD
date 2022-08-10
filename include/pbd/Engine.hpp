#pragma once
#include <vector>
#include <cinttypes>
#include <glm/vec3.hpp>

#include <pbd/collide/Particle.hpp>
#include <pbd/collide/Plane.hpp>

#include <pbd/constraint/Distance.hpp>
#include <pbd/constraint/TetraVolume.hpp>

#include <pbd/ExtractedRotation.hpp>
#include <pbd/Types.hpp>

namespace pbd {
	class Engine {
	public:
		Engine();

		glm::vec3 gravity;
		float dt;
		int numSubsteps;
		float friction;

		// Map for objects?

		// We should allow for setting the position and the velocity in between steps.
		// We should allow the mass to change as well.
		// prevPos is entirely for internal use.
		
		// Structure of arrays or array of structures?
		struct Particles {
			std::vector<glm::vec3> pos, prevPos, velocity;
			std::vector<float> invMass;
			std::vector<int32_t> flags;
		} particle;

		struct CVariant {
			Constraint kind;
			int64_t index;
		};
		std::vector<CVariant> constraints;
		std::vector<int32_t> cdata;

		// Lets start with the bare minimum
		void resize(int64_t count);
		int64_t size() const;

		void solve();

		
		template<typename T>
		int64_t add(const T& cval) {
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
		void midSolve(float sdt);
		void preSolve(float sdt);
		void postSolve(float sdt);
	};
}