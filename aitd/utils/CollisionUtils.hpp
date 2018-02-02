#pragma once

#include <utils/Geometry.hpp>

namespace aitd {
namespace CollisionUtils {

	bool IntersectMTD(Geometry::Polygon<Vec2f> A, Geometry::Polygon<Vec2f> B, Vec2f& MTD);

};
}
