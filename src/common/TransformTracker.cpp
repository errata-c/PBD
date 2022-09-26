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
	const glm::vec3& TransformTracker::position() const noexcept {
		return mposition;
	}

	void TransformTracker::reset(int i0, int i1, int i2, int i3, const ParticleList& particles) {
		mids = {i0, i1, i2, i3};
		glm::quat q(1.f, 0.f, 0.f, 0.f);

		const glm::vec3
			& p0 = particles.pos[mids[0]],
			& p1 = particles.pos[mids[1]],
			& p2 = particles.pos[mids[2]],
			& p3 = particles.pos[mids[3]];
		mextractor.reset(p0, p1, p2, p3);
		mposition = p0;// (p0 + p1 + p2 + p3) * 0.25f;
	}

	void TransformTracker::update(const ParticleList& particles) {
		const glm::vec3
			& p0 = particles.pos[mids[0]],
			& p1 = particles.pos[mids[1]],
			& p2 = particles.pos[mids[2]],
			& p3 = particles.pos[mids[3]];
		mextractor.update(p0, p1, p2, p3);
		mposition = p0;//(p0 + p1 + p2 + p3) * 0.25f;
	}
}