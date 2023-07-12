#include "grid_cut.h"

#include <omp.h>

Grid::Grid(
	double width, double height, double thickness,
	size_t xElements, size_t yElements, size_t zElements)
: m_Width(xElements), m_Height(yElements), m_Thickness(zElements),
  m_ZStride(xElements * yElements), m_YStride(xElements),
  m_XStep(width / static_cast<double>(xElements)),
  m_YStep(height / static_cast<double>(yElements)),
  m_ZStep(thickness / static_cast<double>(zElements)),
  m_Origin(width * -0.5, height * -0.5, thickness * -0.5)
{
	m_Grid.reserve(xElements * yElements * zElements);

	for(size_t z = 0; z < zElements; ++z)
	{
		for(size_t y = 0; y < yElements; ++y)
		{
			for(size_t x = 0; x < xElements; ++x)
			{
				K::Vector_3 currOffset = 
					K::Vector_3(x * m_XStep, y * m_YStep, z * m_ZStep);
				K::Vector_3 nextOffset = 
					K::Vector_3((x+1) * m_XStep, (y+1) * m_YStep, (z+1) * m_ZStep);

				m_Grid.push_back(CGAL::Iso_cuboid_3<K>(
					m_Origin + currOffset, 
					m_Origin + nextOffset));

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

