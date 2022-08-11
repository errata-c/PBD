#include <EngineNode.hpp>

#include <MultiMesh.hpp>

#include <cassert>
#include <array>

#include <glm/geometric.hpp>

namespace godot {
	void EngineNode::_register_methods() {
		register_method("set_substeps", &EngineNode::set_substeps);
		register_method("set_timestep", &EngineNode::set_timestep);

		register_method("set_multi_mesh_instance", &EngineNode::set_multi_mesh_instance);
		register_method("add_particle", &EngineNode::add_particle);
		register_method("set_static_friction", &EngineNode::set_static_friction);
		register_method("set_kinetic_friction", &EngineNode::set_kinetic_friction);
		register_method("num_particles", &EngineNode::num_particles);

		register_method("add_distance_constraint", &EngineNode::add_distance_constraint);
		register_method("add_tetra_volume_constraint", &EngineNode::add_tetra_volume_constraint);
		register_method("add_nh_tetra_volume_constraint", &EngineNode::add_nh_tetra_volume_constraint);
		register_method("add_plane_collide", &EngineNode::add_plane_collide);

		register_method("add_plane_collide", &EngineNode::add_plane_collide);
		register_method("update_mesh", &EngineNode::update_mesh);
		register_method("solve", &EngineNode::solve);
	}

	EngineNode::EngineNode() 
		: mminst(nullptr)
	{

	}
	EngineNode::~EngineNode() {

	}

	void EngineNode::_init() {
		Godot::print("EngineNode initialized!");
	}

	void EngineNode::set_multi_mesh_instance(MultiMeshInstance* node) {
		mminst = node;
	}

	void EngineNode::set_substeps(int count) {
		count = std::max(count, 1);
		engine.numSubsteps = count;
	}
	void EngineNode::set_timestep(float delta) {
		delta = std::max(1e-5f, delta);
		engine.dt = delta;
	}

	void EngineNode::set_static_friction(float friction) {
		// Cannot be zero (?), cannot be negative
		friction = std::max(friction, 1e-5f);
		engine.staticFriction = friction;
	}
	void EngineNode::set_kinetic_friction(float friction) {
		// Cannot be zero (?), cannot be negative
		friction = std::max(friction, 1e-5f);
		engine.kineticFriction = friction;
	}

	int64_t EngineNode::num_particles() const {
		return engine.size();
	}

	void EngineNode::add_particle(Vector3 pos, float imass, float radius) {
		engine.addParticle(glm::vec3{ pos.x, pos.y, pos.z }, imass, radius);
	}

	void EngineNode::add_distance_constraint(int id0, int id1, float compliance) {
		glm::vec3
			p0 = engine.particle.pos[id0],
			p1 = engine.particle.pos[id1];

		engine.addConstraint(pbd::ConstraintDistance{ id0, id1, glm::length(p1 - p0), compliance });
	}

	void EngineNode::add_tetra_volume_constraint(int id0, int id1, int id2, int id3, float compliance) {
		glm::vec3
			p0 = engine.particle.pos[id0],
			p1 = engine.particle.pos[id1],
			p2 = engine.particle.pos[id2],
			p3 = engine.particle.pos[id3];

		glm::vec3
			t0 = p1 - p0,
			t1 = p2 - p0,
			t3 = p3 - p0;
		
		glm::vec3 t4 = glm::cross(t0, t1);

		engine.addConstraint(pbd::ConstraintTetraVolume{
			{id0, id1, id2, id3},
			glm::dot(t3, t4) / 6.f,
			compliance
		});
	}
	void EngineNode::add_nh_tetra_volume_constraint(int id0, int id1, int id2, int id3, float compliance) {
		glm::vec3
			p0 = engine.particle.pos[id0],
			p1 = engine.particle.pos[id1],
			p2 = engine.particle.pos[id2],
			p3 = engine.particle.pos[id3];
		glm::mat3 restPose;
		restPose[0] = p1 - p0;
		restPose[1] = p2 - p0;
		restPose[2] = p3 - p0;

		restPose = glm::inverse(restPose);

		engine.addConstraint(pbd::ConstraintNHTetraVolume{
			{id0, id1, id2, id3},
			restPose,
			compliance
		});
	}

	void EngineNode::add_plane_collide(int id, Vector3 o, Vector3 n) {
		glm::vec3 origin{
			o.x,
			o.y,
			o.z
		}, 
		normal{
			n.x,
			n.y,
			n.z
		};

		engine.addConstraint(pbd::CollidePlane{ id, origin, normal });
	}


	void EngineNode::update_mesh() {
		assert(mminst);

		auto mmesh = mminst->get_multimesh();

		if (buffer.size() < (engine.size() * 12)) {
			buffer.resize(engine.size() * 12);

			mmesh->set_instance_count(engine.size());
		}
		mmesh->set_visible_instance_count(engine.size());

		{
			auto write = buffer.write();

			std::array<float, 12> form;
			form.fill(0.f);
			form[0] = 1.f;
			form[5] = 1.f;
			form[10] = 1.f;

			float* data = write.ptr();
			for (int i = 0; i < engine.size(); ++i) {
				glm::vec3 pos = engine.particle.pos[i];
				form[3] = pos.x;
				form[7] = pos.y;
				form[11] = pos.z;
				
				for (float val : form) {
					*data++ = val;
				}
			}



			static int once = 0;
			once++;
			if (once == 480) {
				for (int i = 0; i < engine.size(); ++i) {
					glm::vec3 p = engine.particle.pos[i];
					String text = "pos: {0}, {1}, {2}";
					Godot::print(text.format(Array::make(p.x, p.y, p.z)));
				}
			}
		}

		mmesh->set_as_bulk_array(buffer);
	}

	void EngineNode::solve() {
		engine.solve();
	}
}