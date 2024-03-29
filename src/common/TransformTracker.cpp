#include <pbd/common/TransformTracker.hpp>
#include <pbd/engine/ParticleList.hpp>

namespace pbd {

	TransformTracker::TransformTracker()
		: mids{-1,-1,-1,-1}
		, mposition(0.f)
	{}
	TransformTracker::TransformTracker(int i0, int i1, int i2, int i3, const ParticleList& particles)
	{
		reset(i0, i1, i2, i3, particles);
	}

	TransformTracker::operator bool() const noexcept {
		return
			mids[0] != -1 &&
			mids[1] != -1 &&
			mids[2] != -1 &&
			mids[3] != -1;
	}

	const std::array<int, 4>& TransformTracker::ids() const noexcept {
		return mids;
	}

	const glm::mat3& TransformTracker::basis() const noexcept {
		return mextractor.rotation;
	}
	const vec3_t& TransformTracker::position() const noexcept {
		return mposition;
	}

	void TransformTracker::reset(int i0, int i1, int i2, int i3, const ParticleList& particles) {
		mids = {i0, i1, i2, i3};
		quat_t q(1.f, 0.f, 0.f, 0.f);

		const vec3_t
			& p0 = particles[mids[0]].position,
			& p1 = particles[mids[1]].position,
			& p2 = particles[mids[2]].position,
			& p3 = particles[mids[3]].position;
		mextractor.reset(p0, p1, p2, p3);
		mposition = p0;// (p0 + p1 + p2 + p3) * 0.25f;
	}

	void TransformTracker::update(const ParticleList& particles) {
		const vec3_t
			& p0 = particles[mids[0]].position,
			& p1 = particles[mids[1]].position,
			& p2 = particles[mids[2]].position,
			& p3 = particles[mids[3]].position;
		mextractor.update(p0, p1, p2, p3);
		mposition = p0;//(p0 + p1 + p2 + p3) * 0.25f;
	}
}