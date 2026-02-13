#include "vec.h"

std::ostream& operator<<(std::ostream& strm, const Vector3D& vec)
{
	strm << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
	return strm;
}

std::ostream& operator<<(std::ostream& strm, const Cuboid& cuboid)
{
	strm << "[origin: " << cuboid.origin << "; size: " << cuboid.size << "]";
	return strm;
}

std::ostream& operator<<(std::ostream& strm, const CuboidCrnr& cuboid)
{
	strm << "[min: " << cuboid.min << "; max: " << cuboid.max << "]";
	return strm;
}
 