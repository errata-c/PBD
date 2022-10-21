#include <vector>
#include <deque>
#include <memory>
#include <pbd/common/Span.hpp>

#include <cppitertools/itertools.hpp>

#include <catch2/catch_all.hpp>

using namespace pbd;

TEST_CASE("Span vector") {
	using container_t = std::vector<int>;
	using iterator = container_t::iterator;
	using span_t = Span<iterator>;
	using cspan_t = Span<container_t::const_iterator>;

	container_t data;

	for (int i : iter::range(32)) {
		data.push_back(i);
	}
	
	span_t test;
	REQUIRE(test.empty());
	REQUIRE(test.size() == 0);

	auto init_offset = GENERATE(false, true);
	CAPTURE(init_offset);

	if (init_offset) {
		test = span_t(data.begin(), data.size());
	}
	else {
		test = span_t(data.begin(), data.end());
	}

	REQUIRE(test.size() == data.size());
	REQUIRE(!test.empty());
	REQUIRE(test.begin() == data.begin());
	REQUIRE(test.end() == data.end());

	for (int i : iter::range(test.size())) {
		CAPTURE(i);
		REQUIRE(test[i] == data[i]);
	}

	for (int i : iter::range(test.size())) {
		CAPTURE(i);
		REQUIRE(test.at(i) == data[i]);
	}

	REQUIRE_THROWS(test.at(data.size() + 1));

	REQUIRE(test.front() == data.front());
	REQUIRE(test.back() == data.back());

	{
		int v = data.size()-1;
		for (int i : iter::reversed(test)) {
			CAPTURE(i, v);
			REQUIRE(i == v);
			--v;
		}
	}

	span_t other(data.begin(), data.end());
	REQUIRE(test == other);

	test = test.subspan(0, 10);
	other = span_t(data.begin(), 10);

	REQUIRE(test == other);
}

TEST_CASE("Span deque") {
	using container_t = std::deque<int>;
	using iterator = container_t::iterator;
	using span_t = Span<iterator>;

	container_t data;

	for (int i : iter::range(32)) {
		data.push_back(i);
	}

	span_t test;
	REQUIRE(test.empty());
	REQUIRE(test.size() == 0);

	auto init_offset = GENERATE(false, true);
	CAPTURE(init_offset);

	if (init_offset) {
		test = span_t(data.begin(), data.size());
	}
	else {
		test = span_t(data.begin(), data.end());
	}

	REQUIRE(test.size() == data.size());
	REQUIRE(!test.empty());
	REQUIRE(test.begin() == data.begin());
	REQUIRE(test.end() == data.end());

	for (int i : iter::range(test.size())) {
		CAPTURE(i);
		REQUIRE(test[i] == data[i]);
	}

	for (int i : iter::range(test.size())) {
		CAPTURE(i);
		REQUIRE(test.at(i) == data[i]);
	}

	REQUIRE_THROWS(test.at(data.size() + 1));

	REQUIRE(test.front() == data.front());
	REQUIRE(test.back() == data.back());

	{
		int v = data.size() - 1;
		for (int i : iter::reversed(test)) {
			CAPTURE(i, v);
			REQUIRE(i == v);
			--v;
		}
	}

	span_t other(data.begin(), data.end());
	REQUIRE(test == other);

	test = test.subspan(0, 10);
	other = span_t(data.begin(), 10);

	REQUIRE(test == other);
}

TEST_CASE("Span pointer") {
	std::unique_ptr<int[]> data(new int[32]);
	using iterator = int*;
	using span_t = Span<iterator>;

	size_t size = 32;
	iterator begin = data.get();
	iterator end = begin + size;

	for (int i : iter::range(32)) {
		data[i] = i;
	}

	span_t test;
	REQUIRE(test.empty());
	REQUIRE(test.size() == 0);

	auto init_offset = GENERATE(false, true);
	CAPTURE(init_offset);

	if (init_offset) {
		test = span_t(begin, size);
	}
	else {
		test = span_t(begin, end);
	}

	REQUIRE(test.size() == size);
	REQUIRE(!test.empty());
	REQUIRE(test.begin() == begin);
	REQUIRE(test.end() == end);

	for (int i : iter::range(test.size())) {
		CAPTURE(i);
		REQUIRE(test[i] == data[i]);
	}

	for (int i : iter::range(test.size())) {
		CAPTURE(i);
		REQUIRE(test.at(i) == data[i]);
	}

	REQUIRE_THROWS(test.at(size + 1));

	REQUIRE(test.front() == *begin);
	REQUIRE(test.back() == *(end-1));

	{
		int v = size - 1;
		for (int i : iter::reversed(test)) {
			CAPTURE(i, v);
			REQUIRE(i == v);
			--v;
		}
	}

	span_t other(begin, end);
	REQUIRE(test == other);

	test = test.subspan(0, 10);
	other = span_t(begin, 10);

	REQUIRE(test == other);
}


TEST_CASE("Constant Span vector") {
	using container_t = std::vector<int>;
	using iterator = container_t::iterator;
	using span_t = Span<iterator>;
	using cspan_t = Span<container_t::const_iterator>;

	container_t data;

	for (int i : iter::range(32)) {
		data.push_back(i);
	}

	cspan_t test;
	REQUIRE(test.empty());
	REQUIRE(test.size() == 0);

	auto init_offset = GENERATE(false, true);
	CAPTURE(init_offset);

	if (init_offset) {
		test = span_t(data.begin(), data.size());
	}
	else {
		test = span_t(data.begin(), data.end());
	}

	REQUIRE(test.size() == data.size());
	REQUIRE(!test.empty());
	REQUIRE(test.begin() == data.begin());
	REQUIRE(test.end() == data.end());

	for (int i : iter::range(test.size())) {
		CAPTURE(i);
		REQUIRE(test[i] == data[i]);
	}

	for (int i : iter::range(test.size())) {
		CAPTURE(i);
		REQUIRE(test.at(i) == data[i]);
	}

	REQUIRE_THROWS(test.at(data.size() + 1));

	REQUIRE(test.front() == data.front());
	REQUIRE(test.back() == data.back());

	{
		int v = data.size() - 1;
		for (int i : iter::reversed(test)) {
			CAPTURE(i, v);
			REQUIRE(i == v);
			--v;
		}
	}

	span_t other(data.begin(), data.end());
	REQUIRE(test == other);

	test = test.subspan(0, 10);
	other = span_t(data.begin(), 10);

	REQUIRE(test == other);
}