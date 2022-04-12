#include <pbd/constraint/CollidePlane.hpp>
#include <pbd/Engine3D.hpp>

#include <glm/geometric.hpp>

namespace pbd {
	void CollidePlane::eval(Engine3D& engine, int id) const {
		// C = dot((x0 - origin), normal)
		// grad(C, x0) = x0 * normal

		glm::vec3& x0 = engine.pos[id];
		float w0 = engine.invMass[id];
		if (w0 < 1e-5f) {
			return;
		}

		float c = glm::dot(x0 - origin, normal);
		x0 += -c * normal;
	}
}