#pragma once
#include <ez/KVStore.hpp>
#include <deque>
#include <parallel_hashmap/phmap.h>

#include <pbd/prefab/Prefab.hpp>

namespace pbd {
	/*
	A simple filetype for storing prefabs.
	*/
	class PrefabDatabase {
	public:
		using name_container = std::deque<std::string>;
		using const_iterator = name_container::const_iterator;

		PrefabDatabase(PrefabDatabase&&) noexcept = default;
		PrefabDatabase & operator=(PrefabDatabase&&) noexcept = default;

		PrefabDatabase() = default;
		~PrefabDatabase() = default;

		void swap(PrefabDatabase & other) noexcept;
		
		bool isOpen() const noexcept;

		bool create(const std::filesystem::path& path, bool overwrite = false);
		bool open(const std::filesystem::path& path, bool readonly = false);
		void close();

		size_t numPrefabs() const;

		bool load(const_iterator it, Prefab& prefab) const;
		bool load(std::string_view name, Prefab& prefab) const;
		bool save(std::string_view name, const Prefab& prefab);

		bool erase(std::string_view name);
		const_iterator erase(const_iterator it);

		bool contains(std::string_view name) const;

		const_iterator begin() const;
		const_iterator end() const;

		const_iterator find(std::string_view name) const;
	private:
		ez::KVStore store;
		name_container prefabNames;
	};
}