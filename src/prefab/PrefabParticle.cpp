#include <pbd/prefab/PrefabParticle.hpp>
#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	PrefabParticle::PrefabParticle()
		: position(0.f)
		, velocity(0.f)
		, imass(1.f)
		, radius(1.f)
		, flags(1)
	{}
	PrefabParticle::PrefabParticle(const glm::vec3& _pos, float _imass, float _radius, uint32_t _flags)
		: position(_pos)
		, velocity(0.f)
		, imass(_imass)
		, radius(_radius)
		, flags(_flags)
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
		ez::serialize::u32(particle.flags, output);
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
		first = ez::deserialize::u32(first, last, particle.flags);

		return first;
	}
}