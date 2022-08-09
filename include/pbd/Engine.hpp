#pragma once
#include <vector>
#include <glm/vec3.hpp>

#include <pbd/constraint/CollideParticle.hpp>
#include <pbd/constraint/CollidePlane.hpp>
#include <pbd/constraint/Distance.hpp>
#include <pbd/constraint/TetraVolume.hpp>

namespace pbd {
	class Engine {
	public:
		Engine();

		glm::vec3 gravity;
		float dt;
		int numSubsteps;
		float friction;

		// We should allow for setting the position and the velocity in between steps.
		// We should allow the mass to change as well.
		// prevPos is entirely for internal use.
		
		// Structure of arrays or array of structures?
		std::vector<glm::vec3> pos, prevPos, velocity;
		std::vector<float> invMass;

		std::vector<CollidePlane> planes;
		std::vector<Distance> distances;
		std::vector<TetraVolume> tetras;

		// Lets start with the bare minimum
		void resize(int count);
		int size() const;

		void solve();

	private:
		void midSolve(float sdt);
		void preSolve(float sdt);
		void postSolve(float sdt);
	};
}