#include <pbd/prefab/PrefabTracker.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	PrefabTracker::PrefabTracker()
		: particles{0,0,0,0}
	{}
	PrefabTracker::PrefabTracker(std::string_view _name, int32_t p0, int32_t p1, int32_t p2, int32_t p3)
		: name(_name)
		, particles{p0, p1, p2, p3}
	{}

	void PrefabTracker::serialize(const PrefabTracker& tracker, std::string& output) {
		ez::serialize::string(tracker.name, output);
		ez::serialize::i32(tracker.particles[0], output);
		ez::serialize::i32(tracker.particles[1], output);
		ez::serialize::i32(tracker.particles[2], output);
		ez::serialize::i32(tracker.particles[3], output);
	}
	const char* PrefabTracker::deserialize(const char* first, const char* last, PrefabTracker& tracker) {
		first = ez::deserialize::string(first, last, tracker.name);
		first = ez::deserialize::i32(first, last, tracker.particles[0]);
		first = ez::deserialize::i32(first, last, tracker.particles[1]);
		first = ez::deserialize::i32(first, last, tracker.particles[2]);
		first = ez::deserialize::i32(first, last, tracker.particles[3]);

		return first;
	}
}