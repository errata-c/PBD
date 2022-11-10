#include <pbd/engine/Engine.hpp>
#include <algorithm>
#include <limits>

namespace pbd {
	Engine::Engine()
		: static_friction(1.f)
		, kinetic_friction(1.f)
		, gravity(0,-9.8,0)
		, substeps(4)
		, dt(1.0 / 60.0)
	{}

	size_t Engine::num_particles() const noexcept {
		return particles.list.size();
	}
	size_t Engine::num_bodies() const noexcept {
		return bodies.list.size();
	}
	size_t Engine::num_constraints() const noexcept {
		return constraints.size();
	}

	// Run one iteration of the solver
	void Engine::solve() {
		prepare_round();
		
		// Collision constraints would be generated here!
		// If needed we can predict where the positions will be roughly, then find the collisions.
		
		// Run the iteration substeps.
		float sdt = dt / float(substeps);
		for (int i = 0; i < substeps; ++i) {
			integrate_changes(sdt);
			apply_constraints(sdt);
			update_states(sdt);
		}

		clear_forces();
	}

	void Engine::integrate_changes(float sdt) {
		// Store these starting positions for comparison in the constraints.
		save_previous();

		for (int64_t i = 0, count = num_particles(); i < count; ++i) {
			float imass = particles.list[i].imass;
			if (imass < 1e-5f) {
				continue;
			}

			glm::vec3 & velocity = particles.list[i].velocity;
			glm::vec3 & position = particles.list[i].position;

			// External forces
			const glm::vec3 & force = particles.forces[i];
			
			// Simple euler integration
			velocity = velocity + sdt * (gravity + force * imass);
			position = position + sdt * velocity;
		}

		for (int64_t i = 0, count = num_bodies(); i < count; ++i) {
			float imass = bodies.list[i].imass;
			if (imass < 1e-5f) {
				continue;
			}

			RigidBody & body = bodies.list[i];

			glm::vec3& velocity = body.velocity;
			glm::vec3& position = body.position;

			// External forces
			const glm::vec3& force = bodies.forces[i];

			// Simple euler integration
			velocity = velocity + sdt * (gravity + force * imass);
			position = position + sdt * velocity;


			glm::vec3& angular_velocity = body.angular_velocity;
			glm::quat& orientation = body.orientation;

			// External torques
			const glm::vec3 & torque = bodies.torques[i];

			// We are assuming that inertia is non-zero.
			glm::vec3 finertia = 1.f / body.inertia;

			angular_velocity = angular_velocity + sdt * body.inertia * (torque - glm::cross(angular_velocity, finertia * angular_velocity));
			orientation = orientation + 0.5f * sdt * (glm::quat(0.f, angular_velocity.x, angular_velocity.y, angular_velocity.z) * orientation);
			orientation = glm::normalize(orientation);
		}
	}
	void Engine::apply_constraints(float sdt) {
		// Run all the constraints

		float rdt2 = 1.f / (sdt * sdt);
		for (int64_t i = 0, count = constraints.size(); i < count; ++i) {
			constraints[i].eval(*this, rdt2);
		}
	}
	void Engine::update_states(float sdt) {
		sdt = 1.f / sdt;

		{
			auto pit = particles.prevPos.begin();
			for (Particle & particle: particles.list) {
				particle.velocity = (particle.position - *pit) * sdt;
				++pit;
			}
		}
		{
			auto pit = bodies.prevPos.begin();
			auto oit = bodies.prevOrientation.begin();
			for (RigidBody & body: bodies.list) {
				body.velocity = (body.position - *pit) * sdt;
				glm::quat deltao = body.orientation * glm::inverse(*oit);
				body.angular_velocity = 2.f * glm::vec3(deltao.x, deltao.y, deltao.z) * sdt;

				// Correct the sign
				if (deltao.w < 0.f) {
					body.angular_velocity = -body.angular_velocity;
				}

				++pit;
				++oit;
			}

			/*
			Solve velocities? 
			
			The collision constraints for both particles and bodies are fairly straight forward, and
			can be added to the constraint solve step.

			The velocity solve here however is specific to the body collisions (but can be applied to particle collisions).
			The velocity pass is used to solve for dynamic friction, restitution, and joint damping.

			This velocity step also solves an issue with PBD, that the velocities are only meaningful if no collision has occurred.
			This issue leads to timing dependent behavior (inconsistent, non-realistic), and large velocities when objects are
			created in an overlapped state.

			I think that this calls for an entirely new method for computing collisions. One that is more uniform and 
			easier to understand and integrate.

			It's a good idea to test things out here, just to see if it makes a significant difference.
			Also, we could make this step optional. That would allow people to choose performance over collision realism.
			*/
		}
	}

	void Engine::prepare_round() {
		particles.forces.resize(num_particles(), glm::vec3(0));
		bodies.forces.resize(num_bodies(), glm::vec3(0));
		bodies.torques.resize(num_bodies(), glm::vec3(0));

		particles.prevPos.resize(num_particles(), glm::vec3(0));
		bodies.prevPos.resize(num_bodies(), glm::vec3(0));
		bodies.prevOrientation.resize(num_bodies(), glm::quat(1.f, 0.f, 0.f, 0.f));
	}

	void Engine::clear_forces() {
		std::fill(particles.forces.begin(), particles.forces.end(), glm::vec3(0.f));
		std::fill(bodies.forces.begin(), bodies.forces.end(), glm::vec3(0.f));
		std::fill(bodies.torques.begin(), bodies.torques.end(), glm::vec3(0.f));
	}
	void Engine::save_previous() {
		{
			auto it = particles.prevPos.begin();
			for (const Particle& particle: particles.list) {
				*it = particle.position;
				++it;
			}
		}
		{
			auto pit = bodies.prevPos.begin();
			auto oit = bodies.prevOrientation.begin();
			for (const RigidBody & body: bodies.list) {
				*pit = body.position;
				*oit = body.orientation;

				++pit;
				++oit;
			}
		}
	}
}