#pragma once
#include <array>
#include <string>
#include <pbd/common/RotationExtractor.hpp>

namespace pbd {
	class Engine;

	// Class to handle the robust extraction of rotation matricies from deformable objects.
	// This class will calculate a matrix that represents the rotation of a deformed tetrahedron.
	// If the bones for your mesh are not prealigned with the initial rotation of the tetrahedron, you have to apply an inverse transform to them.
	class TransformTracker {
	public:
		static void serialize(const TransformTracker& clist, std::string& output);
		static const char* deserialize(const char* first, const char* last, TransformTracker& clist);


		TransformTracker();
		TransformTracker(int i0, int i1, int i2, int i3, const Engine & engine);

		explicit operator bool() const noexcept;

		void reset(int i0, int i1, int i2, int i3, const Engine & engine);
		
		void update(const Engine & engine);

		const std::array<int, 4>& ids() const noexcept;
		
		const glm::mat3 & basis() const noexcept;
		const glm::vec3 & position() const noexcept;
	private:
		// Ids of the tetrahedra this rotation is extracted from.
		std::array<int, 4> mids;
		RotationExtractor mextractor;
		glm::vec3 mposition;
	};
}