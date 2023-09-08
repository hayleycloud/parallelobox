#include "symmetry.h"
#include "random.h"
#include <algorithm>
#include <numbers>

constexpr unsigned int ITERATIONS = 3;
constexpr unsigned int NUM_SAMPLES = 100;
constexpr unsigned int FILTER_SIZE = 5;
constexpr double HARD_ERROR_TOLERANCE = 1.0;
constexpr double INITIAL_SEARCH_RADIUS = 0.1;


///////////////////////////////////////////////////////////////////////////////
// Utility Functions

void normalizeSpherical(double* variate)
{
	if(*variate < 0.0)
		*variate += 1.0;
	else if(*variate > 1.0)
		*variate -= 1.0;
}

const K::Vector_3 normalize2(const K::Vector_3& v)
{
	double l = sqrt(v.squared_length());
	return K::Vector_3(v.x() / l, v.y() / l, v.z() / l);
}

double dot(const K::Vector_3& a, const K::Vector_3& b)
{
	return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
}

inline const std::pair<K::Point_3, K::Vector_3> hardPlane(
	const K::Point_3& centroid, const K::Vector_3& normal)
{
	return std::make_pair(centroid, normal);
}

LLNormal::LLNormal(double u, double v) : u(u), v(v)
{
	theta = std::acos((2.0 * v) - 1.0);
	phi = 2.0 * std::numbers::pi * u;

	n = K::Vector_3(
		std::cos(phi) * std::sin(theta),
		std::sin(phi) * std::sin(theta),
		std::cos(theta));
}

double LLNormal::len() const
{
	return std::sqrt(n.x() * n.x() + n.y() * n.y() + n.z() * n.z());
}

double LLNormal::angleBetween(const LLNormal& other) const
{
	return std::acos(
		n.x() * other.n.x() + n.y() * other.n.y() + n.z() * other.n.z());
}

///////////////////////////////////////////////////////////////////////////////
// Symmetry Determination
//

inline K::Point_3 getCentroid(const Mesh& mesh)
{
	const Mesh::Property_map<Mesh::Vertex_index,K::Point_3>& pnts = mesh.points();
	return CGAL::centroid(pnts.begin(), pnts.end(), CGAL::Dimension_tag<0>());
}

inline bool positivelyOrientedWithPlane(
	const HardPlane& symmetryPlane, const K::Point_3& v)
{
    K::Point_3 u = symmetryPlane.first;
	K::Vector_3 dv = 
        K::Vector_3(v.x(), v.y(), v.z()) - 
        K::Vector_3(u.x(), u.y(), u.z());
	return dot(dv, symmetryPlane.second) > 0.0;
}

inline double distanceToPlane(const K::Point_3& v, const HardPlane& plane)
{
    K::Point_3 u = plane.first;
	K::Vector_3 n = plane.second;
	K::Vector_3 dv = 
        K::Vector_3(v.x(), v.y(), v.z()) - 
        K::Vector_3(u.x(), u.y(), u.z());
	
	return dot(dv, n);
}

inline void reflectInPlane(
	const K::Point_3& v, const HardPlane& plane, K::Point_3* vOut)
{
	K::Point_3 w = v - (plane.second * distanceToPlane(v, plane) * 2.0);
	*vOut = w;
}

inline double meanAbsoluteError(const std::vector<double>& errors)
{
	double sumErrors = 0.0;
	for(auto error: errors)
		sumErrors += error;
	return sumErrors / static_cast<double>(errors.size());
}

typedef CGAL::Search_traits_3<K> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Tree;

double symmetryFitness(
	const Mesh& cgalMesh,
	const HardPlane& symmetryPlane,
	double planeThickness)
{
	std::vector<K::Point_3> pointsSide1, pointsSide2;
	std::vector<K::Point_3> sampleSide1, sampleSide2;
	for(K::Point_3 vertex: cgalMesh.points())
	{
		K::Point_3 reflPoint;
		double distToPlane = distanceToPlane(vertex, symmetryPlane); 
		reflectInPlane(vertex, symmetryPlane, &reflPoint);

		if(positivelyOrientedWithPlane(symmetryPlane, vertex))
		{
			pointsSide1.push_back(vertex);
			sampleSide2.push_back(reflPoint);
		}
		else
		{
			pointsSide2.push_back(vertex);
			sampleSide1.push_back(reflPoint);
		}
	}

    Tree side1Tree(pointsSide1.begin(), pointsSide1.end());
    Tree side2Tree(pointsSide2.begin(), pointsSide2.end());
	std::vector<double> errors;

	//std::cout << "Processing side 1..." << std::endl;
	for(const K::Point_3& vertex: sampleSide1)
	{
        Neighbor_search search(side1Tree, vertex, 1);
		//std::cout << "Processed vertex: " << vertex.x() << "," << vertex.y() << "," << vertex.z() << "." <<  std::endl;
		//std::cout << search.begin()->first << " " << std::sqrt(search.begin()->second) << std::endl;
		errors.push_back(std::sqrt(search.begin()->second));
	}

	//std::cout << "Processing side 2..." << std::endl;
	for(const K::Point_3& vertex: sampleSide2)
	{
        Neighbor_search search(side2Tree, vertex, 1);
		//std::cout << "Processed vertex: " << vertex.x() << "," << vertex.y() << "," << vertex.z() << "." <<  std::endl;
		//std::cout << search.begin()->first << " " << std::sqrt(search.begin()->second) << std::endl;
		errors.push_back(std::sqrt(search.begin()->second));
	}

	return meanAbsoluteError(errors);
}

void stochasticNormalSample(
	std::vector<LLNormal>* normals, 
	unsigned int sampleSize,
	RNGReal<double>& uRange,
	RNGReal<double>& vRange)
{
	for(unsigned int i = 0; i < sampleSize; ++i)
	{
		double u = fetchRandom(uRange);
		double v = fetchRandom(vRange);

		normalizeSpherical(&u);
		normalizeSpherical(&v);

		//std::cout << "Lat: " << lat << ", Long: " << lng << std::endl;
		normals->push_back(LLNormal(u, v));
	}
}

void condense(std::list<NormalFitnessPair>* normals, double arcLength)
{
	for(auto n1itr = normals->begin(); n1itr != normals->end(); ++n1itr)
	{
		NormalFitnessPair normal1 = *n1itr;
		for(auto n2itr = n1itr; n2itr != normals->end(); ++n2itr)
		{
			NormalFitnessPair normal2 = *n2itr;
			if(n2itr != n1itr)
			{
				double a = normal1.first.angleBetween(normal2.first);
				if(a <= arcLength || a >= std::numbers::pi - arcLength)
				{
					n2itr = normals->erase(n2itr);
					--n2itr;
				}
			}
		}
	}
}

void findSymmetries(
	const Mesh& mesh, 
	K::Point_3* centroid, 
	std::vector<NormalFitnessPair>* symmetries)
{
	// Determine planes of symmetry using a stochastic method
	double searchRadius = INITIAL_SEARCH_RADIUS;
	K::Point_3 centroidLocal = getCentroid(mesh);
	std::vector<LLNormal> bestNormals;
	std::vector<NormalFitnessPair> normals;

	for(unsigned int i = 1; i <= ITERATIONS; ++i)
	{
		std::vector<LLNormal> sampleNormals;

		if(bestNormals.size() == 0)
		{
			RNGReal<double> uRange = makeRNGReal<double>(0.0, 1.0);
			RNGReal<double> vRange = makeRNGReal<double>(0.0, 1.0);
			stochasticNormalSample(&sampleNormals, NUM_SAMPLES, uRange, vRange);
		}
		else
		{
			const double halfSearchRad = searchRadius * 0.5;
			std::list<NormalFitnessPair> normalPairs;
			for(const LLNormal& normal: bestNormals)
			{
				const double u = normal.u, v = normal.v;
				RNGReal<double> uRange = 
					makeRNGReal<double>(u - halfSearchRad, u + halfSearchRad);
				RNGReal<double> vRange = 
					makeRNGReal<double>(v - halfSearchRad, v + halfSearchRad);
				stochasticNormalSample(
					&sampleNormals, NUM_SAMPLES / 3, uRange, vRange);
			}
		}

		std::list<NormalFitnessPair> eligibleNormals;

		#pragma omp parallel for
		for(const LLNormal& normal: sampleNormals)
		{
			HardPlane plane = hardPlane(centroidLocal, normal.n);
			double f = symmetryFitness(mesh, plane, 0.01);
			#pragma omp critical
			eligibleNormals.push_back(std::make_pair(normal, f));
		}

		eligibleNormals.sort([=](const NormalFitnessPair& op1, const NormalFitnessPair& op2){
			return op1.second < op2.second;
		});

		condense(&eligibleNormals, 0.5);

		std::list<NormalFitnessPair>::iterator nItr = eligibleNormals.begin();
		if(i == ITERATIONS)
		{
			for(unsigned int nIndex = 0; nIndex < FILTER_SIZE; ++nIndex)
			{
				if((*nItr).second < HARD_ERROR_TOLERANCE)
					normals.push_back(*nItr);
				if((++nItr) == eligibleNormals.end())
					break;
			}
		}
		else
		{
			symmetries->clear();
			for(unsigned int nIndex = 0; nIndex < FILTER_SIZE; ++nIndex)
			{
				symmetries->push_back(*nItr);
				if((++nItr) == eligibleNormals.end())
					break;
			}
		}
	}

	*centroid = centroidLocal;
}

void findSymmetries(
	const Mesh& mesh, std::vector<HardPlane>* planes)
{
	K::Point_3 centroid;
	std::vector<NormalFitnessPair> normals;

	findSymmetries(mesh, &centroid, &normals);

	for(const auto& n: normals)
		planes->push_back(hardPlane(centroid, n.first.n));
}

void symmetrySplit(
	const Mesh& mesh, const K::Plane_3& plane, Mesh* outLeft, Mesh* outRight)
{
	*outLeft = Mesh(mesh);
	CGAL::Polygon_mesh_processing::clip(
		*outLeft, plane, 
		CGAL::Polygon_mesh_processing::parameters::clip_volume(true));

	*outRight = Mesh(mesh);
	CGAL::Polygon_mesh_processing::clip(
		*outRight, plane.opposite(), 
		CGAL::Polygon_mesh_processing::parameters::clip_volume(true));
}

bool symmetrySplit(const Mesh& mesh, Mesh* outLeft, Mesh* outRight)
{
	// Find best symmetry
	std::vector<HardPlane> planes;
	findSymmetries(mesh, &planes);

	// Do nothing if no suitable symmetries detected
	if(planes.size() < 1)
		return false;

	const HardPlane& bestPlane = planes[0];

	// Split with the best symmetry!
	symmetrySplit(
		mesh, K::Plane_3(bestPlane.first, bestPlane.second), outLeft, outRight);

	return true;
}

