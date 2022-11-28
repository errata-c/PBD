#pragma once
#include <pbd/common/Types.hpp>
#include <glm/gtx/io.hpp>
#include <fmt/core.h>
#include <fmt/ostream.h>

template <> struct fmt::formatter<pbd::vec3_t> : ostream_formatter {};
template <> struct fmt::formatter<pbd::vec2_t> : ostream_formatter {};
template <> struct fmt::formatter<pbd::quat_t> : ostream_formatter {};
