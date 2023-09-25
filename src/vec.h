#pragma once

struct Vector3D
{
	Vector3D() : x(0), y(0), z(0) {}

	Vector3D(int x, int y, int z) : x(x), y(y), z(z) {}

	int x, y, z;
	
	bool operator==(const Vector3D& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

	Vector3D operator+(const Vector3D& other) const {
        return Vector3D(x + other.x, y + other.y, z + other.z );
    }

    Vector3D operator-(const Vector3D& other) const {
        return Vector3D(x - other.x, y - other.y, z - other.z );
    }
};

struct Cuboid
{
	Cuboid(const Vector3D& origin, const Vector3D& size)
		: origin(origin), size(size) {};

	Vector3D origin;	// Top Left Back (0, 0, 0)
	Vector3D size;

	Vector3D end() const {
		return origin + size;
	}
};

