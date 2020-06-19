
#pragma once

#include <array>
#include <Eigen/Core>

#include <Discregrid/common.hpp>

namespace Discregrid
{

enum class NearestEntity
{
	VN0, VN1, VN2, EN0, EN1, EN2, FN
};

Real point_triangle_sqdistance(Vector3r const& point, 
	std::array<Vector3r const*, 3> const& triangle,
	Vector3r* nearest_point = nullptr,
	NearestEntity* ne = nullptr);

}

