#include <pbd/prefab/PrefabDatabase.hpp>

#include <fmt/format.h>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

#include <cppitertools/itertools.hpp>

#include <xxhash.h>

static uint64_t hash(std::string_view text) {
	return static_cast<uint64_t>(XXH3_64bits(text.data(), text.length()));
}
static uint64_t hash(std::string_view text, uint64_t seed) {
	return static_cast<uint64_t>(XXH64(text.data(), text.length(), seed));
}

static uint64_t hash(std::string_view text0, std::string_view text1) {
	return hash(text1, hash(text0));
}

namespace pbd {
	using const_iterator = PrefabDatabase::const_iterator;

	struct EntryName {
		EntryName(std::string_view group, std::string_view name)
		{
			uint64_t id = hash(group, name);
			ez::serialize::u64(id, data.data(), data.data() + data.size());
		}
		EntryName(const PrefabDatabase::Entry & entry)
			: EntryName(*entry.group, *entry.name)
		{}
		operator std::string_view() noexcept {
			return std::string_view(data.data(), data.size());
		}

		std::array<char, 8> data;
	};

	static constexpr std::string_view 
		file_kind("pbd_prefab"),
		name_prefix("nm_"),
		group_prefix("gp_"),
		entry_prefix("en_");

	PrefabDatabase::~PrefabDatabase() {
		close();
	}

	void PrefabDatabase::swap(PrefabDatabase& other) noexcept {
		store.swap(other.store);
		entries.swap(other.entries);
		groups.swap(other.groups);
	}

	bool PrefabDatabase::isOpen() const noexcept {
		return store.isOpen();
	}

	bool PrefabDatabase::create(const std::filesystem::path& path, bool overwrite) {
		if (store.create(path, overwrite)) {
			store.setKind(file_kind);
			return true;
		}
		else {
			return false;
		}
	}
	bool PrefabDatabase::open(const std::filesystem::path& path, bool readonly) {
		if (!store.open(path, readonly)) {
			return false;
		}

		if (store.getKind() != file_kind) {
			store.close();
			return false;
		}

		load_entries();

		return true;
	}
	void PrefabDatabase::close() {
		if (isOpen()) {
			save_entries();
			store.close();
		}
		groups.clear();
		entries.clear();
	}

	size_t PrefabDatabase::num_prefabs() const {
		return entries.size();
	}
	size_t PrefabDatabase::num_groups() const {
		return groups.size();
	}

	bool PrefabDatabase::load(const_iterator it, Prefab& prefab) const {
		assert(isOpen());
		return load(*it->group, *it->name, prefab);
	}
	bool PrefabDatabase::load(std::string_view group, std::string_view name, Prefab& prefab) const {
		if (!isOpen()) {
			return false;
		}
		if (!contains(group, name)) {
			return false;
		}

		std::string data;
		bool res = store.get(name, data);
		assert(res);

		Prefab::deserialize(data.data(), data.data() + data.size(), prefab);

		return true;
	}
	bool PrefabDatabase::save(std::string_view group, std::string_view name, const Prefab& prefab) {
		assert(isOpen());
		if (!isOpen()) {
			return false;
		}

		std::string data;
		Prefab::serialize(prefab, data);

		EntryName ename(group, name);
		if (!store.contains(ename)) {
			uint64_t grp = hash(group);
			auto it = groups.find(grp);
			if (it == groups.end()) {
				auto tmp = groups.insert(std::make_pair(grp, std::make_shared<std::string>(group)));
				it = tmp.first;
			}

			entries.push_back(Entry{it->second, std::make_shared<std::string>(name)});
		}

		bool res = store.set(ename, data);
		assert(res);

		return true;
	}

	// Done
	const_iterator PrefabDatabase::find(std::string_view group, std::string_view name) const {
		const_iterator it = begin();
		for (const_iterator last = end(); it != last; ++it) {
			if (*it->group == group && *it->name == name) {
				return it;
			}
		}
		return end();
	}

	//Done
	bool PrefabDatabase::contains(std::string_view group, std::string_view name) const {
		EntryName ename(group, name);
		return store.contains(ename);
	}

	// Done
	const_iterator PrefabDatabase::begin() const {
		return entries.begin();
	}
	const_iterator PrefabDatabase::end() const {
		return entries.end();
	}

	// Done
	bool PrefabDatabase::erase(std::string_view group, std::string_view name) {
		const_iterator it = find(group, name);
		if (it == end()) {
			return false;
		}
		
		erase(it);
		return true;
	}

	// Done
	const_iterator PrefabDatabase::erase(const_iterator it) {
		assert(it < end());

		auto git = groups.find(hash(*it->group));
		// The iterator should be valid, and from THIS database
		assert(git != groups.end());
		
		EntryName ename(*it);
		bool res = store.erase(ename);
		assert(res);

		const_iterator result = entries.erase(it);

		if (git->second.use_count() == 1) {
			groups.erase(git);
		}

		return result;
	}

	void PrefabDatabase::load_entries() {
		std::string group_list, name_list;
		bool res = store.get("groups", group_list);
		assert(res);

		res = store.get("names", group_list);
		assert(res);

		entries.clear();
		groups.clear();

		uint64_t ngroups = 0;
		{
			const char * read = group_list.data();
			const char * last = read + group_list.size();

			read = ez::deserialize::u64(read, last, ngroups);
			for (uint64_t _: iter::range(ngroups)) {
				std::shared_ptr<std::string> name(new std::string{});

				read = ez::deserialize::string(read, last, *name);
				uint64_t id = hash(*name);

				groups.insert(std::make_pair(id, std::move(name)));
			}
		}
		uint64_t nnames = 0;
		{
			const char* read = name_list.data();
			const char* last = read + name_list.size();

			read = ez::deserialize::u64(read, last, nnames);
			for (uint64_t _ : iter::range(nnames)) {
				uint64_t grp = 0;
				std::shared_ptr<std::string> name(new std::string{});

				// Get the associated group id
				read = ez::deserialize::u64(read, last, grp);

				// Get the actual name string
				read = ez::deserialize::string(read, last, *name);

				// Create the entry, linking to the pre-existing group name.
				entries.push_back(Entry{groups[grp], std::move(name)});
			}
		}
	}
	void PrefabDatabase::save_entries() {
		std::string group_list, name_list;

		ez::serialize::u64(groups.size(), group_list);
		for (auto& group : groups) {
			ez::serialize::string(*group.second, group_list);
		}

		ez::serialize::u64(entries.size(), name_list);
		for (auto & entry: entries) {
			uint64_t grhv = hash(*entry.group);
			uint64_t enhv = hash(*entry.name, grhv);

			// Store the group id
			ez::serialize::u64(grhv, name_list);

			// Store the name
			ez::serialize::string(*entry.name, name_list);
		}

		bool res = store.set("groups", group_list);
		assert(res);

		res = store.set("names", name_list);
		assert(res);
	}
}