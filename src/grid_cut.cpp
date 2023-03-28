#include "grid_cut.h"

#include <omp.h>

Grid::Grid(
	double width, double height, double thickness,
	size_t xElements, size_t yElements, size_t zElements)
: m_Width(xElements), m_Height(yElements), m_Thickness(zElements),
  m_ZStride(xElements * yElements), m_YStride(xElements)
{
	const K::Point_3 origin(width * -0.5, height * -0.5, thickness * -0.5);

	const double xStep = width / static_cast<double>(xElements);
	const double yStep = height / static_cast<double>(yElements);
	const double zStep = thickness / static_cast<double>(zElements);

	m_Grid.reserve(xElements * yElements * zElements);

	for(size_t z = 0; z < zElements; ++z)
	{
		for(size_t y = 0; y < yElements; ++y)
		{
			for(size_t x = 0; x < xElements; ++x)
			{
				K::Vector_3 currOffset = 
					K::Vector_3(x * xStep, y * yStep, z * zStep);
				K::Vector_3 nextOffset = 
					K::Vector_3((x+1) * xStep, (y+1) * yStep, (z+1) * zStep);

				m_Grid.push_back(CGAL::Iso_cuboid_3<K>(
					origin + currOffset, 
					origin + nextOffset));

				std::cout << "min: " << m_Grid.back().min() 
					      << ", max: " << m_Grid.back().max()
						  << std::endl;;
			}
		}
	}
}

size_t Grid::getOffset(size_t x, size_t y, size_t z) const
{
	return (m_ZStride * z) + (m_YStride * y) + x;
}

const K::Iso_cuboid_3& Grid::get(size_t x, size_t y, size_t z) const
{
	return m_Grid[getOffset(x, y, z)];
}

const K::Iso_cuboid_3& Grid::get(const Coord& coord) const
{
	return m_Grid[getOffset(coord.x, coord.y, coord.z)];
}

void Grid::clipVolume(Mesh& mesh, std::vector<Mesh>& outMeshes) const
{
	clip(mesh, outMeshes, true);
}

void Grid::clipVolume(Mesh& mesh, size_t x, size_t y, size_t z) const
{
	clip(mesh, x, y, z, true);
}

void Grid::clipVolume(Mesh& mesh, const Coord& coord) const
{
	clip(mesh, coord.x, coord.y, coord.z, true);
}

void Grid::clipSurface(Mesh& mesh, std::vector<Mesh>& outMeshes) const
{
	clip(mesh, outMeshes, false);
}

void Grid::clipSurface(Mesh& mesh, size_t x, size_t y, size_t z) const
{
	clip(mesh, x, y, z, false);
}

void Grid::clipSurface(Mesh& mesh, const Coord& coord) const
{
	clip(mesh, coord.x, coord.y, coord.z, false);
}

void Grid::clip(Mesh& mesh, std::vector<Mesh>& outMeshes, bool fill) const
{
	outMeshes.resize(m_Width * m_Height * m_Thickness);

	//#pragma omp parallel for
	for(size_t z = 0; z < m_Thickness; ++z)
	{
		for(size_t y = 0; y < m_Height; ++y)
		{
			for(size_t x = 0; x < m_Width; ++x)
			{
				Mesh out(mesh);
				clip(out, x, y, z, fill);
				outMeshes[getOffset(x, y, z)] = out;
			}
		}
	}
}

void Grid::clip(Mesh& mesh, size_t x, size_t y, size_t z, bool fill) const
{
	const K::Iso_cuboid_3& bbox = get(x, y, z);
	//std::cout << bbox << std::endl;
	PMP::clip(mesh, bbox, CGAL::parameters::clip_volume(fill));
}

