#pragma once
#include <string>

namespace pbd {
	/*
	Custom container class to store the non-runtime data of the constraints.
	*/
	class PrefabConstraints {
	public:
		static void serialize(const PrefabConstraints& clist, std::string& output);
		static const char* deserialize(const char* first, const char* last, PrefabConstraints& clist);
		
	private:
		
	};
}