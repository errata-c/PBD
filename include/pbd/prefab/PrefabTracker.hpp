#pragma once
#include <cinttypes>
#include <string>
#include <string_view>
#include <array>

namespace pbd {
	// Tracker for rotation.
	class PrefabTracker {
	public:
		static void serialize(const PrefabTracker& tracker, std::string& output);
		static const char* deserialize(const char* first, const char* last, PrefabTracker& tracker);

		PrefabTracker();
		PrefabTracker(std::string_view _name, int32_t p0, int32_t p1, int32_t p2, int32_t p3);

		// Name of the object we are tracking for.
		std::string name;
		// Particle ids we track with.
		std::array<int32_t, 4> particles;
	};
}