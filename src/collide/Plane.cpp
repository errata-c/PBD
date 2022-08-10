#include <pbd/collide/Plane.hpp>
#include <pbd/Engine.hpp>

#include <glm/geometric.hpp>

namespace pbd {
	void CollidePlane::eval(Engine& engine, float rdt2) const {
		// C = dot((x0 - origin), normal)
		// grad(C, x0) = x0 * normal
		// No need for compliance
		// We do need friction however.

		glm::vec3& x0 = engine.particle.pos[id];
		float w0 = engine.particle.invMass[id];
		if (w0 < 1e-5f) {
			return;
		}

		float c = glm::dot(x0 - origin, normal);
		x0 += -c * normal;
	}
}