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
		enum class Option {
			Overwrite,
			ReadOnly
		};

		struct Entry {
			std::shared_ptr<std::string> group, name;
		};
		using const_iterator = std::vector<Entry>::const_iterator;

		PrefabDatabase(PrefabDatabase&&) noexcept = default;
		PrefabDatabase & operator=(PrefabDatabase&&) noexcept = default;

		PrefabDatabase() = default;
		~PrefabDatabase();

		void swap(PrefabDatabase & other) noexcept;
		
		bool isOpen() const noexcept;

		bool create(const std::filesystem::path& path, bool overwrite = false);
		bool open(const std::filesystem::path& path, bool readonly = false);
		void close();

		size_t num_prefabs() const;
		size_t num_groups() const;

		bool load(const_iterator it, Prefab& prefab) const;
		bool load(std::string_view group, std::string_view name, Prefab& prefab) const;
		bool save(std::string_view group, std::string_view name, const Prefab& prefab);

		bool erase(std::string_view group, std::string_view name);
		const_iterator erase(const_iterator it);

		bool contains(std::string_view group, std::string_view name) const;

		const_iterator begin() const;
		const_iterator end() const;

		const_iterator find(std::string_view group, std::string_view name) const;

		// Find all the entries in a group
		std::vector<Entry> find_group(std::string_view group) const;
		// Find all the entries with a given name
		std::vector<Entry> find_name(std::string_view name) const;
	private:
		ez::KVStore store;

		phmap::flat_hash_map<uint64_t, std::shared_ptr<std::string>> groups;
		std::vector<Entry> entries;

		void load_entries();
		void save_entries();
	};
}