#include "grid_cut.h"

#include <omp.h>

Grid::Grid(const K::Point_3& min, const K::Point_3& max, double granularity)
: m_Width(std::abs(max.x() - min.x())),
  m_Height(std::abs(max.y() - min.y())),
  m_Depth(std::abs(max.z() - min.z())),
  m_ElementSize(std::cbrt((m_Width * m_Height * m_Depth) * granularity)),
  m_NumBoxesX(std::ceil(m_Width / m_ElementSize)),
  m_NumBoxesY(std::ceil(m_Height / m_ElementSize)),
  m_NumBoxesZ(std::ceil(m_Depth / m_ElementSize)),
  m_ZStride(m_NumBoxesX * m_NumBoxesY),
  m_YStride(m_NumBoxesX),
  m_Origin(min)
{
	const size_t numBoxes = m_NumBoxesX * m_NumBoxesY * m_NumBoxesZ;

	std::cout << "Creating grid of block size " << m_ElementSize << " ";
	std::cout << "(" << m_NumBoxesX << ", " << m_NumBoxesY << ", " << m_NumBoxesZ << ")";
	std::cout << " [" << numBoxes << " boxes]... ";

	m_Grid.reserve(numBoxes);

	for(size_t z = 0; z < m_NumBoxesZ; ++z)
	{
		for(size_t y = 0; y < m_NumBoxesY; ++y)
		{
			for(size_t x = 0; x < m_NumBoxesX; ++x)
			{
				K::Vector_3 currOffset = K::Vector_3(
					x * m_ElementSize, y * m_ElementSize, z * m_ElementSize);
				K::Vector_3 nextOffset = K::Vector_3(
					(x+1) * m_ElementSize, (y+1) * m_ElementSize, (z+1) * m_ElementSize);

				m_Grid.push_back(CGAL::Iso_cuboid_3<K>(
					m_Origin + currOffset, 
					m_Origin + nextOffset));

				//std::cout << "min: " << m_Grid.back().min() 
				//	      << ", max: " << m_Grid.back().max()
				//		  << std::endl;;
			}
		}
	}

	std::cout << "done." << std::endl;
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
	outMeshes.resize(m_NumBoxesX * m_NumBoxesY * m_NumBoxesZ);

	//#pragma omp parallel for
	for(size_t z = 0; z < m_NumBoxesZ; ++z)
	{
		for(size_t y = 0; y < m_NumBoxesY; ++y)
		{
			for(size_t x = 0; x < m_NumBoxesX; ++x)
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

