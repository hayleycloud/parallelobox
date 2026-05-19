#include "align.h"
#include "symmetry.h"
#include <limits>

void bounds(const Mesh& mesh, K::Point_3& min, K::Point_3& max)
{
	constexpr double minDbl = -std::numeric_limits<double>::max();
	constexpr double maxDbl = std::numeric_limits<double>::max();
	double minX = maxDbl, minY = maxDbl, minZ = maxDbl;
	double maxX = minDbl, maxY = minDbl, maxZ = minDbl;

	for(Mesh::Vertex_index vIndex: mesh.vertices())
	{
		const K::Point_3& p = mesh.point(vIndex);
		if(p.x() < minX)
			minX = p.x();
		if(p.x() > maxX)
			maxX = p.x();
		if(p.y() < minY)
			minY = p.y();
		if(p.y() > maxY)
			maxY = p.y();
		if(p.z() < minZ)
			minZ = p.z();
		if(p.z() > maxZ)
			maxZ = p.z();
	}

	min = K::Point_3(minX, minY, minZ);
	max = K::Point_3(maxX, maxY, maxZ);
}

void boundsLocal(const Mesh& mesh, K::Point_3& min, K::Point_3& max)
{
	Mesh::Vertex_index firstVertex = *(mesh.vertices().begin());
	const K::Point_3& first = mesh.point(firstVertex);

	double minX = first.x(), minY = first.y(), minZ = first.z();
	double maxX = first.x(), maxY = first.y(), maxZ = first.z();

	for(Mesh::Vertex_index vIndex: mesh.vertices())
	{
		const K::Point_3& p = mesh.point(vIndex);
		if(p.x() < minX)
			minX = p.x();
		if(p.x() > maxX)
			maxX = p.x();
		if(p.y() < minY)
			minY = p.y();
		if(p.y() > maxY)
			maxY = p.y();
		if(p.z() < minZ)
			minZ = p.z();
		if(p.z() > maxZ)
			maxZ = p.z();
	}

	min = K::Point_3(minX, minY, minZ);
	max = K::Point_3(maxX, maxY, maxZ);
}

bool isFloor(
	const Mesh& mesh, const face_descriptor& fd, const K::Vector_3& floor)
{
	constexpr double eps = 0.0001;

	// TODO: *All* vertices must be on floor for face to be floor

	for(Mesh::Vertex_index vIndex: CGAL::vertices_around_face(
		mesh.halfedge(fd), mesh))
	{
		const auto& vertex = mesh.point(vIndex);

		if(floor.x() < -eps || floor.x() > eps)
			return std::abs(vertex.x() - floor.x()) <= eps;
		else if(floor.y() < -eps || floor.y() > eps)
			return std::abs(vertex.y() - floor.y()) <= eps;
		else if(floor.z() < -eps || floor.z() > eps)
			return std::abs(vertex.z() - floor.z()) <= eps;
	}

	return false;
}

double overhangArea(
	const Config& config, 
	const Mesh& mesh, 
	const MeshNormalsMap& fnormals,
	const K::Vector_3& up,
	const K::Vector_3& floor)
{
	// TODO: Optimisations can be done here! Everything's in degrees, and the 
	// trig can be avoided!
	const double maxOverhang = 90.0 + config.printer.overhangTolerance;

	double overhangSurfaceArea = 0.0;

	for(face_descriptor fd: faces(mesh))
	{
		auto n = fnormals[fd];
		double dot = n * up;
		double cos_a = acos(dot) * r;
#ifdef XVERBOSE
		std::cout << up << " | " << n << " = " << cos_a;
#endif
		if(cos_a >= maxOverhang)
		{
			if(isFloor(mesh, fd, floor)) {
#ifdef XVERBOSE
				std::cout << " [Floor]";
#endif
			}
			else
			{
				// TODO: Scale by distance to floor
				double farea = PMP::face_area(fd, mesh);
				double projArea = farea * std::abs(dot);
				overhangSurfaceArea += projArea;
#ifdef XVERBOSE
				std::cout << " [Overhang = " << farea << " | " << projArea << "]";
#endif
			}
		}

#ifdef XVERBOSE
		std::cout << std::endl;
#endif

		//std::cout << n << " (" << cos_a << ")" << std::endl;
	}

	return overhangSurfaceArea;
}


void recenter(Mesh& mesh)
{
	K::Point_3 c = CGAL::centroid(
		mesh.points().begin(), 
		mesh.points().end(),
		CGAL::Dimension_tag<0>());
	K::Vector_3 dc(-c.x(), -c.y(), -c.z());

	for(Mesh::Vertex_index vIndex: mesh.vertices())
		mesh.point(vIndex) += dc;
}

void rotationAligning(
	const K::Vector_3& v1, const K::Vector_3& v2, K::Aff_transformation_3& matrix)
{
	K::Vector_3 axis = CGAL::cross_product(v1, v2);
	const float cosA = v1 * v2;
	const float k = 1.0f / (1.0f + cosA);

	matrix = K::Aff_transformation_3(
		(axis.x() * axis.x() * k) + cosA,
		(axis.y() * axis.x() * k) - axis.z(),
		(axis.z() * axis.x() * k) + axis.y(),
		(axis.x() * axis.y() * k) + axis.z(),
		(axis.y() * axis.y() * k) + cosA,
		(axis.z() * axis.y() * k) - axis.x(),
		(axis.x() * axis.z() * k) - axis.y(),
		(axis.y() * axis.z() * k) + axis.x(),
		(axis.z() * axis.z() * k) + cosA
	);
}

void alignMeshSymmetryToGrid(Mesh& mesh)
{
	std::vector<HardPlane> symmetries;
	findSymmetries(mesh, &symmetries);

	assert(symmetries.size() > 0);
	const HardPlane& bestSymmetry = symmetries[0];
	const K::Vector_3 planeNormal = bestSymmetry.second;

	const K::Vector_3 originAxis(0, 0, 1);
	K::Aff_transformation_3 matrix;
	rotationAligning(planeNormal, originAxis, matrix);

	CGAL::Polygon_mesh_processing::transform(matrix, mesh);
}

std::array<K::Point_3, 8> getOBBPointsFrom(Mesh& mesh)
{
	std::array<K::Point_3, 8> points;
	CGAL::oriented_bounding_box(mesh, points);
	return points;
}

K::Vector_3 normalize(const K::Vector_3& v)
{
	return v * (1.0 / CGAL::sqrt(v.squared_length()));
}

std::array<K::Vector_3, 6> 
getPlaneNormalsFrom(const std::array<K::Point_3, 8>& obb)
{
	K::Vector_3 v01(obb[0], obb[1]);
	K::Vector_3 v03(obb[0], obb[3]);
	K::Vector_3 v05(obb[0], obb[5]);

	std::array<K::Vector_3, 6> normals;
	normals[0] = CGAL::cross_product(v01, v03);
	normals[1] = CGAL::cross_product(v01, v05);
	normals[2] = CGAL::cross_product(v03, v05);
	normals[0] = normalize(normals[0]);
	normals[1] = normalize(normals[1]);
	normals[2] = normalize(normals[2]);

	normals[3] = -normals[0];
	normals[4] = -normals[1];
	normals[5] = -normals[2];

	return normals;
}

void alignMeshToBoundingBox(Mesh& mesh)
{
	auto points = getOBBPointsFrom(mesh);
	auto planeNormals = getPlaneNormalsFrom(points);

	// TODO: Use these to determine how to align the object, after having been
	// aligned symmetrically already!
}

void alignMeshToGrid(Mesh& mesh) {
	alignMeshSymmetryToGrid(mesh);
}

void getTriangles(const Mesh& mesh, std::vector<K::Triangle_3>& triangles)
{
	for(Mesh::Face_index fIndex: mesh.faces())
	{
		std::vector<K::Point_3> triangle;

		for(Mesh::Vertex_index vIndex: CGAL::vertices_around_face(
			mesh.halfedge(fIndex), mesh))
		{
			triangle.push_back(mesh.point(vIndex));
		}

		triangles.push_back(K::Triangle_3(
			triangle[0], triangle[1], triangle[2]));
	}
}

/*K::Plane_3 pca(Mesh& mesh)
{
	K::Plane_3 principalPlane;

//	std::vector<K::Triangle_3> triangles;
//	getTriangles(mesh, triangles);
//
//	CGAL::linear_least_squares_fitting_3(
//		triangles.begin(),
//		triangles.end(),
//		principalPlane,
//		CGAL::Dimension_tag<2>());

	CGAL::linear_least_squares_fitting_3(
		mesh.points().begin(),
		mesh.points().end(),
		principalPlane,
		CGAL::Dimension_tag<0>());

	return principalPlane;
}

K::Vector_3 rotationFromPCA(const K::Plane_3& pcaPlane)
{
	return K::Vector_3(0.0, 0.0, 0.0);
}

void orient(Mesh& mesh)
{
	K::Plane_3 principalPlane = pca(mesh);

	K::Vector_3 ortho = principalPlane.orthogonal_vector();

	std::cout << "P: " << ortho << std::endl;

	K::Vector_3 zx = K::Vector_3(ortho.x(), 0.0, ortho.z());
	K::Vector_3 z = K::Vector_3(0.0, 0.0, 1.0);
	double a = CGAL::approximate_angle(z, zx);
	if(zx.x() < 0)
		a = -a;

	std::cout << "a: " << a << std::endl;
	//K::Vector_3 xy = K::Vector_3(
}
*/

 
