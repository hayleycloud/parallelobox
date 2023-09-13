#pragma once

#include "cgals.h"

class Grid
{
public:
	struct Coord
	{
		size_t x, y, z;
	};

	Grid(const K::Point_3& min, const K::Point_3& max, double granularity);

	[[nodiscard]] size_t getNumBoxesX() const { return m_NumBoxesX; }

	[[nodiscard]] size_t getNumBoxesY() const { return m_NumBoxesY; }

	[[nodiscard]] size_t getNumBoxesZ() const { return m_NumBoxesZ; }

	[[nodiscard]] double getElementSize() const { return m_ElementSize; }

	[[nodiscard]] const K::Point_3& getOrigin() const { return m_Origin; }

	[[nodiscard]] const auto& get() const { return m_Grid; }

	[[nodiscard]] const K::Iso_cuboid_3& get(size_t x, size_t y, size_t z) const;

	[[nodiscard]] const K::Iso_cuboid_3& get(const Coord& coord) const;

	void clipVolume(Mesh& mesh, std::vector<Mesh>& outMeshes) const;

	void clipVolume(Mesh& mesh, size_t x, size_t y, size_t z) const;

	void clipVolume(Mesh& mesh, const Coord& coord) const;

	void clipSurface(Mesh& mesh, std::vector<Mesh>& outMeshes) const;

	void clipSurface(Mesh& mesh, size_t x, size_t y, size_t z) const;

	void clipSurface(Mesh& mesh, const Coord& coord) const;

private:
	const double m_Width, m_Height, m_Depth;
	const double m_ElementSize;
	const size_t m_NumBoxesX, m_NumBoxesY, m_NumBoxesZ;
	const size_t m_ZStride, m_YStride;
	const K::Point_3 m_Origin;
	std::vector<K::Iso_cuboid_3> m_Grid;

	size_t getOffset(size_t x, size_t y, size_t z) const;

	void clip(Mesh& mesh, std::vector<Mesh>& outMeshes, bool fill) const;

	void clip(Mesh& mesh, size_t x, size_t y, size_t z, bool fill) const;
};

