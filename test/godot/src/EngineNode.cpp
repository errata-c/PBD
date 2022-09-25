#include <EngineNode.hpp>

#include <MultiMesh.hpp>

#include <cassert>
#include <array>

#include <glm/geometric.hpp>
#include <glm/gtx/norm.hpp>

namespace godot {
	void EngineNode::_register_methods() {
		register_method("nearest_point", &EngineNode::nearest_point);
		register_method("get_position", &EngineNode::get_position);
		register_method("set_force", &EngineNode::set_force);

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

		register_method("set_tracker", &EngineNode::set_tracker);
		register_method("get_tracker_transform", &EngineNode::get_tracker_transform);

		register_method("create_empty_prefab", &EngineNode::create_empty_prefab);
	}

	EngineNode::EngineNode() 
		: mminst(nullptr)
	{}
	EngineNode::~EngineNode() {

	}

	void EngineNode::_init() {
		Godot::print("EngineNode initialized!");
	}

	PrefabRef* EngineNode::create_empty_prefab() {
		return PrefabRef::_new();
	}

	void EngineNode::set_multi_mesh_instance(MultiMeshInstance* node) {
		mminst = node;
	}

	void EngineNode::set_force(int32_t id, Vector3 force) {
		if (id < 0 || id >= engine.size()) {
			return;
		}

		engine.particle.force[id] = glm::vec3(force.x, force.y, force.z);
	}

	int32_t EngineNode::nearest_point(Vector3 _origin, Vector3 _normal) const {
		glm::vec3 o(_origin.x, _origin.y, _origin.z);
		glm::vec3 n(_normal.x, _normal.y, _normal.z);

		int32_t nearest = -1;
		float ndist = 1e10;
		for (size_t i = 0, count = engine.size(); i < count; ++i) {
			glm::vec3 d = engine.particle.pos[i] - o;
			float t = glm::dot(d, n);
			float dist = glm::length2(engine.particle.pos[i] - (n * t + o));

			if (dist < ndist) {
				nearest = static_cast<int32_t>(i);
				ndist = dist;
			}
		}

		return nearest;
	}

	Vector3 EngineNode::get_position(int32_t id) const {
		if (id < 0 || id  >= engine.size()) {
			return Vector3(0,0,0);
		}
		const glm::vec3 & p = engine.particle.pos[id];
		return Vector3(p.x, p.y, p.z);
	}

	void EngineNode::set_substeps(int count) {
		count = std::max(count, 1);
		engine.substeps = count;
	}
	void EngineNode::set_timestep(float delta) {
		delta = std::max(1e-5f, delta);
		engine.dt = delta;
	}

	void EngineNode::set_static_friction(float friction) {
		// Cannot be zero (?), cannot be negative
		friction = std::max(friction, 1e-5f);
		engine.static_friction = friction;
	}
	void EngineNode::set_kinetic_friction(float friction) {
		// Cannot be zero (?), cannot be negative
		friction = std::max(friction, 1e-5f);
		engine.kinetic_friction = friction;
	}

	int64_t EngineNode::num_particles() const {
		return engine.size();
	}

	void EngineNode::add_particle(Vector3 pos, float imass, float radius) {
		engine.particle.add(glm::vec3{ pos.x, pos.y, pos.z }, imass, radius);
	}

	void EngineNode::add_distance_constraint(int id0, int id1, float compliance) {
		glm::vec3
			p0 = engine.particle.pos[id0],
			p1 = engine.particle.pos[id1];

		engine.add_constraint(pbd::ConstraintDistance{ id0, id1, glm::length(p1 - p0), compliance });
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

		engine.add_constraint(pbd::ConstraintTetraVolume{
			{id0, id1, id2, id3},
			glm::dot(t3, t4) / 6.f,
			compliance
		});
	}
	void EngineNode::add_nh_tetra_volume_constraint(int id0, int id1, int id2, int id3, float hydrostatic_compliance, float deviatoric_compliance) {
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

		engine.add_constraint(pbd::ConstraintNHTetraVolume{
			{id0, id1, id2, id3},
			restPose,
			hydrostatic_compliance,
			deviatoric_compliance
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

		engine.add_constraint(pbd::CollidePlane{ id, origin, normal });
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

		if (tracker) {
			tracker.update(engine);
		}
	}

	void EngineNode::set_tracker(int id0, int id1, int id2, int id3) {
		for (int id : {id0, id1, id2, id3}) {
			if (id < 0 || id >= engine.num_particles()) {
				Godot::print_error("Tracker id is invalid!", __FUNCTION__, __FILE__, __LINE__);
				return;
			}
		}
		
		tracker.reset(id0, id1, id2, id3, engine);
	}
	Transform EngineNode::get_tracker_transform() {
		if (!tracker) {
			Godot::print_warning("Attempted to get tracker transform without initializing the tracker!", __FUNCTION__, __FILE__, __LINE__);
			return Transform{};
		}

		auto convert = [](const glm::vec3 & vec) {
			return Vector3(vec.x, vec.y, vec.z);
		};
		
		glm::mat3 rmat = glm::transpose(tracker.basis());

		// Previous version of the convert code was broken, this works.
		Basis basis(
			convert(rmat[0]),
			convert(rmat[1]),
			convert(rmat[2]));
		
		glm::vec3 origin = tracker.position();

		return Transform(basis, convert(origin));
	}
}