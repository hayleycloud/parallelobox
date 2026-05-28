#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Iso_cuboid_3.h>
#include <CGAL/centroid.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/optimal_bounding_box.h>

#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/transform.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/autorefinement.h>

#include <numbers>

namespace PMP = CGAL::Polygon_mesh_processing;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
typedef Mesh::Property_map<face_descriptor, K::Vector_3> MeshNormalsMap;

constexpr double R = 180.0 / std::numbers::pi;

enum class Direction {
	Right,  // +X
	Left,   // -X
	Up,     // +Y
	Down,   // -Y
	In,     // +Z
	Out     // -Z
};

std::string toText(Direction direction);

std::string toTextSide(Direction direction);

K::Vector_3 toVector(Direction direction);

K::Vector_3 normalize(const K::Vector_3& v);

enum class MeshErrors
{
	NonManifold = 1 << 0,
	Discontinuous = 1 << 1,
	NonTriangular = 1 << 2,
	SelfIntersects = 1 << 3,
	UncertainManifoldness = 1 << 4
};

typedef unsigned int MeshErrorSet;

#define NO_MESH_ERRORS	0
#define ALL_MESH_ERRORS	(std::numeric_limits<MeshErrorSet>::max())
#define SET_MESH_ERROR(errors, error) (errors |= static_cast<MeshErrorSet>(error))
#define INVALID(errorRet) ((errorRet != 0) ? true : false)
#define VALID(errorRet) ((errorRet == 0) ? true : false)
#define HAS_ERROR(error, errorRet) ((errorRet & static_cast<MeshErrorSet>(error)) != 0 ? true : false)

MeshErrorSet validate(
	Mesh& mesh, bool throwOnFail, MeshErrorSet filter = ALL_MESH_ERRORS);
 
