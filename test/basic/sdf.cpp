#include <catch2/catch_all.hpp>

#include <cmath>

#include <fmt/core.h>

#include <glm/vec2.hpp>
#include <glm/mat2x2.hpp>
#include <glm/common.hpp>
#include <glm/geometric.hpp>

using vec2 = glm::vec2;
using mat2 = glm::mat2;

float sdCircle(const vec2& p, float r) {
	return glm::length(p) - r;
}
float sdBox(const vec2& p, const vec2 & b) {
	vec2 d = glm::abs(p) - b;
	return glm::length(glm::max(d, vec2(0.0f))) + glm::min(glm::max(d.x, d.y), 0.0f);
}
/*
This one was not explained at all. The args 'a' and 'b' are the start and end of the box, while 'th' is its thickness.
*/
float sdOrientedBox(const vec2& p, const vec2& a, const vec2& b, float th)
{
	float l = length(b - a);
	vec2  d = (b - a) / l;
	vec2  q = (p - (a + b) * 0.5f);
	q = mat2(d.x, -d.y, d.y, d.x) * q;
	q = abs(q) - vec2(l, th) * 0.5f;
	return glm::length(glm::max(q, vec2(0.f))) + glm::min(glm::max(q.x, q.y), 0.f);
}

TEST_CASE("sdf") {
	// Just trying out a method for finding overlaps or intersections with SDF functions.

	// We know for certain that if the distance of the origin of a circle to a SDF if greater than radius, they overlap.

	vec2 a(std::sqrt(0.5)), b(std::sqrt(0.5));
	a.x = -a.x;

	vec2 p = 2.f * b;

	float d = glm::length(p);

	float d0 = sdBox(p, b);
	float d1 = sdOrientedBox(vec2(0, 0), p + a, p - a, 2.0);

	fmt::print("d = {}, d0 = {}, d1 = {}\n", d, d0, d1);
}