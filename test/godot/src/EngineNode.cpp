#include <EngineNode.hpp>

#include <MultiMesh.hpp>

#include <cassert>
#include <array>

#include <glm/geometric.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/handed_coordinate_space.hpp>
#include <glm/gtx/orthonormalize.hpp>

namespace godot {
	void EngineNode::_register_methods() {
		register_method("nearest_point", &EngineNode::nearest_point);
		register_method("get_position", &EngineNode::get_position);
		register_method("set_force", &EngineNode::set_force);

		register_method("set_substeps", &EngineNode::set_substeps);
		register_method("set_timestep", &EngineNode::set_timestep);

		register_method("set_multi_mesh_instance", &EngineNode::set_multi_mesh_instance);
		register_method("set_static_friction", &EngineNode::set_static_friction);
		register_method("set_kinetic_friction", &EngineNode::set_kinetic_friction);
		register_method("num_particles", &EngineNode::num_particles);
		
		register_method("update_mesh", &EngineNode::update_mesh);
		register_method("solve", &EngineNode::solve);

		register_method("set_tracker", &EngineNode::set_tracker);
		register_method("get_tracker_transform", &EngineNode::get_tracker_transform);

		register_method("create_empty_prefab", &EngineNode::create_empty_prefab);
		register_method("instance_prefab", &EngineNode::instance_prefab);
		register_method("destroy_prefab", &EngineNode::destroy_prefab);
		register_method("destroy_queued", &EngineNode::destroy_queued);
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
		if (id < 0 || id >= engine.num_particles()) {
			return;
		}


		//engine.particle.force[id] = glm::vec3(force.x, force.y, force.z);
	}

	int32_t EngineNode::nearest_point(Vector3 _origin, Vector3 _normal) const {
		glm::vec3 o(_origin.x, _origin.y, _origin.z);
		glm::vec3 n(_normal.x, _normal.y, _normal.z);

		int32_t nearest = -1;
		float ndist = 1e10;
		for (size_t i = 0, count = engine.num_particles(); i < count; ++i) {
			glm::vec3 d = engine.particles().pos[i] - o;
			float t = glm::dot(d, n);
			float dist = glm::length2(engine.particles().pos[i] - (n * t + o));

			if (dist < ndist) {
				nearest = static_cast<int32_t>(i);
				ndist = dist;
			}
		}

		return nearest;
	}

	Vector3 EngineNode::get_position(int32_t id) const {
		if (id < 0 || id  >= engine.num_particles()) {
			return Vector3(0,0,0);
		}
		const glm::vec3 & p = engine.particles().pos[id];
		return Vector3(p.x, p.y, p.z);
	}

	void EngineNode::set_substeps(int count) {
		count = std::max(count, 1);
		engine.set_substeps(count);
	}
	void EngineNode::set_timestep(float delta) {
		delta = std::max(1e-5f, delta);
		engine.set_timestep(delta);
	}

	void EngineNode::set_static_friction(float friction) {
		// Cannot be zero (?), cannot be negative
		friction = std::max(friction, 1e-5f);
		engine.set_static_friction(friction);
	}
	void EngineNode::set_kinetic_friction(float friction) {
		// Cannot be zero (?), cannot be negative
		friction = std::max(friction, 1e-5f);
		engine.set_kinetic_friction(friction);
	}

	uint64_t EngineNode::instance_prefab(PrefabRef* ref) {
		pbd::Prefab copy = ref->prefab;

		glm::vec3
			origin{ 0,0,0 },
			normal{ 0,1,0 };

		int32_t count = copy.num_particles();
		for (int32_t i = 0; i < count; ++i) {
			copy.add_constraint(pbd::CollidePlane{ i, origin, normal });
		}
		
		pbd::ObjectID id = engine.create(copy);
		return static_cast<uint64_t>(id);
	}
	void EngineNode::destroy_prefab(uint64_t _id) {
		auto id = static_cast<pbd::ObjectID>(_id);
		engine.queue_destroy(id);
	}
	void EngineNode::destroy_queued() {
		int32_t first = engine.num_particles();
		engine.destroy_queued();
		int32_t last = engine.num_particles();
	}

	int64_t EngineNode::num_particles() const {
		return engine.num_particles();
	}

	void EngineNode::update_mesh() {
		assert(mminst);

		auto mmesh = mminst->get_multimesh();

		if (buffer.size() < (engine.num_particles() * 12)) {
			buffer.resize(engine.num_particles() * 12);

			mmesh->set_instance_count(engine.num_particles());
		}
		mmesh->set_visible_instance_count(engine.num_particles());

		{
			auto write = buffer.write();

			std::array<float, 12> form;
			form.fill(0.f);
			form[0] = 1.f;
			form[5] = 1.f;
			form[10] = 1.f;

			auto& particles = engine.particles();

			float* data = write.ptr();
			for (int i = 0; i < engine.num_particles(); ++i) {
				glm::vec3 pos = particles.pos[i];
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
				for (int i = 0; i < engine.num_particles(); ++i) {
					glm::vec3 p = particles.pos[i];
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
			tracker.update(engine.particles());
		}
	}

	void EngineNode::set_tracker(int id0, int id1, int id2, int id3) {
		for (int id : {id0, id1, id2, id3}) {
			if (id < 0 || id >= engine.num_particles()) {
				Godot::print_error("Tracker id is invalid!", __FUNCTION__, __FILE__, __LINE__);
				return;
			}
		}
		
		const glm::vec3 
			& p0 = engine.particles().pos[id0],
			& p1 = engine.particles().pos[id1],
			& p2 = engine.particles().pos[id2],
			& p3 = engine.particles().pos[id3];

		glm::mat3 tmp{
			p1 - p0,
			p2 - p0,
			p3 - p0,
		};
		tmp = glm::orthonormalize(tmp);

		if (glm::leftHanded(tmp[1], tmp[0], tmp[2])) {
			tracker.reset(id0, id1, id2, id3, engine.particles());
		}
		else {
			tracker.reset(id0, id3, id2, id1, engine.particles());
		}
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