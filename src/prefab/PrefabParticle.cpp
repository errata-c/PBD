#include <pbd/prefab/PrefabParticle.hpp>
#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	PrefabParticle::PrefabParticle()
		: PrefabParticle(glm::vec3(0.f), 1.f, 1.f, 0u, 0u)
	{}
	PrefabParticle::PrefabParticle(const glm::vec3& _pos, float _imass, float _radius, uint32_t _cgroups, uint32_t _cmask)
		: PrefabParticle(_pos, glm::vec3(0.f), _imass, _radius, _cgroups, _cmask)
	{}
	PrefabParticle::PrefabParticle(const glm::vec3& _pos, const glm::vec3& _vel, float _imass, float _radius, uint32_t _cgroups, uint32_t _cmask)
		: position(_pos)
		, velocity(_vel)
		, imass(_imass)
		, radius(_radius)
		, collision_groups(_cgroups)
		, collision_mask(_cmask)
	{}

	void PrefabParticle::serialize(const PrefabParticle& particle, std::string& output) {
		auto svec = [](const glm::vec3& value, std::string& output) {
			ez::serialize::f32(value.x, output);
			ez::serialize::f32(value.y, output);
			ez::serialize::f32(value.z, output);
		};

		svec(particle.position, output);
		svec(particle.velocity, output);
		ez::serialize::f32(particle.imass, output);
		ez::serialize::f32(particle.radius, output);
		ez::serialize::u32(particle.collision_groups, output);
		ez::serialize::u32(particle.collision_mask, output);
	}
	const char* PrefabParticle::deserialize(const char* first, const char* last, PrefabParticle& particle) {
		auto svec = [](const char* first, const char* last, glm::vec3 & result) {
			first = ez::deserialize::f32(first, last, result.x);
			first = ez::deserialize::f32(first, last, result.y);
			first = ez::deserialize::f32(first, last, result.z);
			return first;
		};

		first = svec(first, last, particle.position);
		first = svec(first, last, particle.velocity);
		first = ez::deserialize::f32(first, last, particle.imass);
		first = ez::deserialize::f32(first, last, particle.radius);
		first = ez::deserialize::u32(first, last, particle.collision_groups);
		first = ez::deserialize::u32(first, last, particle.collision_mask);

		return first;
	}
}