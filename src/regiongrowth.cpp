#include "regiongrowth.h"
#include "objective.h"
#include "multivec.h"
#include "resolve.h"
#include <unordered_map>
#include <optional>
#include <filesystem>
#include <sstream>

namespace fs = std::filesystem;

#define EXPORT_INTERMEDIATES	false
#define USE_CLUSTERS    		false

//void addCellsToMeshBox(
//	mv::vector3<GridCell>& gridCells, 
//	MeshBox& box,
//	const Vector3D& a, const Vector3D& b) 
//{
//	//std::cout << "Hello? " << a << " " << b << std::endl;
//	MeshBox* boxPtr = std::addressof(box);
//    for(int x = a.x; x <= b.x; ++x) 
//	{
//        for(int y = a.y; y <= b.y; ++y) 
//		{
//            for(int z = a.z; z <= b.z; ++z) 
//			{
//				GridCell* cell = std::addressof(mv::get(gridCells, x, y, z));
//
//				assert(cell);
//				assert(std::find(cell->parents.cbegin(), cell->parents.cend(), boxPtr)
//					== cell->parents.cend());
//
//				box.children.push_back(cell);
//				cell->parents.push_back(boxPtr);
//            }
//        }
//    }
//}

void printMeshBoxes(const std::vector<std::unique_ptr<MeshBox>>& meshboxes)
{
	int index = 1;
	for(const auto& box: meshboxes)
	{
		std::cout << "Mesh Box " << index << ": " << box->dims.corners() << std::endl;
		++index;
	}
}

void addCellsToMeshBox(MeshBox& box, std::vector<GridCell*>& cells)
{
	//std::cout << "Hello? " << a << " " << b << std::endl;
	MeshBox* boxPtr = std::addressof(box);
	for(GridCell* cell: cells)
	{
		assert(cell);
		assert(std::find(cell->parents.cbegin(), cell->parents.cend(), boxPtr)
			== cell->parents.cend());

		box.children.push_back(cell);
		cell->parents.push_back(boxPtr);
	}
}

std::vector<GridCell*> sampleCells(
	mv::vector3<GridCell>& gridCells,
	const Vector3D& a, const Vector3D& b)
{
	std::vector<GridCell*> cells;

    for(int x = a.x; x < b.x; ++x)
	{
        for(int y = a.y; y < b.y; ++y)
		{
            for(int z = a.z; z < b.z; ++z)
			{
				GridCell* cell = std::addressof(mv::get(gridCells, x, y, z));
				assert(cell);
				cells.push_back(cell);
            }
        }
    }

	return cells;
}

[[nodiscard]] std::vector<GridCell*> sampleExpand(
	Direction direction, mv::vector3<GridCell>& gridCells, const MeshBox& box)
{
	// Direction indicates the side being grown!
	switch(direction)
	{
		case Direction::Left:
		{
			//std::cout << "Growing Left..." << std::endl;

			Vector3D btmLeftOut = box.dims.origin;
			--btmLeftOut.x;

			Vector3D topLeftIn = 
				btmLeftOut + Vector3D(1, box.dims.size.y, box.dims.size.z);

			return sampleCells(gridCells, btmLeftOut, topLeftIn);
		}
		break;                                                      
		case Direction::Right:                                          
		{
			//std::cout << "Growing Right..." << std::endl;

			const Vector3D btmRightOut = 
				box.dims.origin + Vector3D(box.dims.size.x, 0, 0);
			const Vector3D topRightIn = 
				btmRightOut + Vector3D(1, box.dims.size.y, box.dims.size.z);

			return sampleCells(gridCells, btmRightOut, topRightIn);
		}
		break;
		case Direction::Down:
		{
			//std::cout << "Growing Down..." << std::endl;

			Vector3D btmLeftOut = box.dims.origin;
			--btmLeftOut.y;

			Vector3D btmRightIn =
				btmLeftOut + Vector3D(box.dims.size.x, 1, box.dims.size.z);

			return sampleCells(gridCells, btmLeftOut, btmRightIn);
		}
		break;
		case Direction::Up:
		{
			//std::cout << "Growing Up..." << std::endl;

			const Vector3D topLeftOut = 
				box.dims.origin + Vector3D(0, box.dims.size.y, 0);
			const Vector3D topRightIn = 
				topLeftOut + Vector3D(box.dims.size.x, 1, box.dims.size.z);

			return sampleCells(gridCells, topLeftOut, topRightIn);
		}
		break;
		case Direction::Out:
		{
			//std::cout << "Growing Out..." << std::endl;

			Vector3D btmLeftOut = box.dims.origin;
			--btmLeftOut.z;

			Vector3D topRightOut =
				btmLeftOut + Vector3D(box.dims.size.x, box.dims.size.y, 1);

			return sampleCells(gridCells, btmLeftOut, topRightOut);
		}
		break;
		case Direction::In:
		{
			//std::cout << "Growing In..." << std::endl;

			const Vector3D btmLeftIn = 
				box.dims.origin + Vector3D(0, 0, box.dims.size.z);
			const Vector3D topRightIn = 
				btmLeftIn + Vector3D(box.dims.size.x, box.dims.size.y, 1);

			return sampleCells(gridCells, btmLeftIn, topRightIn);
		}
		break;                                                      
	}

	return {};
}

void grow(Direction direction, mv::vector3<GridCell>& gridCells, MeshBox& box)
{
	// Direction indicates the side being grown!
	switch(direction)
	{
		case Direction::Left:
		{
			//std::cout << "Growing Left..." << std::endl;

			std::vector<GridCell*> cells = sampleExpand(Direction::Left, gridCells, box);

			--box.dims.origin.x;
			++box.dims.size.x;

			addCellsToMeshBox(box, cells);
		}
		break;                                                      
		case Direction::Right:                                          
		{
			//std::cout << "Growing Right..." << std::endl;

			std::vector<GridCell*> cells = sampleExpand(Direction::Right, gridCells, box);

			++box.dims.size.x;

			addCellsToMeshBox(box, cells);
		}
		break;
		case Direction::Down:
		{
			//std::cout << "Growing Down..." << std::endl;

			std::vector<GridCell*> cells = sampleExpand(Direction::Down, gridCells, box);

			--box.dims.origin.y;
			++box.dims.size.y;

			addCellsToMeshBox(box, cells);
		}
		break;
		case Direction::Up:
		{
			//std::cout << "Growing Up..." << std::endl;

			std::vector<GridCell*> cells = sampleExpand(Direction::Up, gridCells, box);

			++box.dims.size.y;

			addCellsToMeshBox(box, cells);
		}
		break;
		case Direction::Out:
		{
			//std::cout << "Growing Out..." << std::endl;

			std::vector<GridCell*> cells = sampleExpand(Direction::Out, gridCells, box);

			--box.dims.origin.z;
			++box.dims.size.z;

			addCellsToMeshBox(box, cells);
		}
		break;
		case Direction::In:
		{
			//std::cout << "Growing In..." << std::endl;

			std::vector<GridCell*> cells = sampleExpand(Direction::In, gridCells, box);

			++box.dims.size.z;

			addCellsToMeshBox(box, cells);
		}
		break;                                                      
	}
}

void resetMeshBoxChildren(MeshBox& meshBox)
{
	GridCell* origin = meshBox.children.front();
	meshBox.children.clear();
}

void sampleCells(
	mv::vector3<GridCell>& gridCells, 
	const Cuboid& cuboid,
	std::vector<GridCell*>& samples) 
{
    for(int x = 0; x < cuboid.size.x; x++) 
	{
		int X = cuboid.origin.x + x;
        for(int y = 0; y < cuboid.size.y; y++) 
		{
			int Y = cuboid.origin.y + y;
            for(int z = 0; z < cuboid.size.z; z++) 
			{
				int Z = cuboid.origin.z + z;

				GridCell* cell = std::addressof(mv::get(gridCells, X, Y, Z));

				assert(cell);
				samples.push_back(cell);
            }
        }
    }
}

std::optional<Cuboid> extendRegionIn(
	Direction direction, Cuboid cuboid, const Grid& grid)
{
	switch(direction)
	{
		case Direction::Left:
		{
			if(cuboid.origin.x == 0)
				return std::nullopt;

			--cuboid.origin.x;
			++cuboid.size.x;
		}
		break;                                                      
		case Direction::Right:                                          
		{
			++cuboid.size.x;
			if(cuboid.end().x > grid.getNumBoxesX())
				return std::nullopt;
		}
		break;                                                 
		case Direction::Up:                                             
		{
			++cuboid.size.y;
			if(cuboid.end().y > grid.getNumBoxesY())
				return std::nullopt;
		}
		break;                                                      
		case Direction::Down:                                         
		{
			if(cuboid.origin.y == 0)
				return std::nullopt;

			--cuboid.origin.y;
			++cuboid.size.y;
		}
		break;                                                      
		case Direction::In:                                             
		{
			++cuboid.size.z;
			if(cuboid.end().z > grid.getNumBoxesZ())
				return std::nullopt;
		}
		break;                                                      
		case Direction::Out:                                            
		{
			if(cuboid.origin.z == 0)
				return std::nullopt;

			--cuboid.origin.z;
			++cuboid.size.z;
		}
		break;
	}

	return cuboid;
}

[[nodiscard]] size_t enumerateBoundaries(const std::vector<GridCell*>& gridCells)
{
	size_t count = 0;
	for(const GridCell* cell: gridCells)
	{
		if(cell->type == GridCell::ContentType::Boundary)
			++count;
	}
	return count;
}

[[nodiscard]] bool isVoid(const std::vector<GridCell*>& cells)
{
	bool hasNonVoids = false;
	for(const GridCell* cell: cells)
		hasNonVoids |= cell->type != GridCell::ContentType::Empty;

	return hasNonVoids;
}

void enumerateUpDirections(
	const MeshBox& box, 
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	std::vector<Direction>& upDirs)
{
	const Vector3D& btmLeftOut = box.dims.origin;
	Vector3D topRightIn = box.dims.last();

	if(btmLeftOut.x == 0)
		upDirs.push_back(Direction::Left);
	else
	{
		std::vector<GridCell*> cells = sampleExpand(Direction::Left, gridCells, box);
		if(isVoid(cells))
			upDirs.push_back(Direction::Left);
	}

	if(btmLeftOut.y == 0)
		upDirs.push_back(Direction::Down);
	else
	{
		std::vector<GridCell*> cells = sampleExpand(Direction::Down, gridCells, box);
		if(isVoid(cells))
			upDirs.push_back(Direction::Down);
	}

	if(btmLeftOut.z == 0)
		upDirs.push_back(Direction::Out);
	else
	{
		std::vector<GridCell*> cells = sampleExpand(Direction::Out, gridCells, box);
		if(isVoid(cells))
			upDirs.push_back(Direction::Out);
	}

	if(topRightIn.x == (grid.getNumBoxesX() - 1))
		upDirs.push_back(Direction::Right);
	else
	{
		std::vector<GridCell*> cells = sampleExpand(Direction::Right, gridCells, box);
		if(isVoid(cells))
			upDirs.push_back(Direction::Right);
	}

	if(topRightIn.y == (grid.getNumBoxesY() - 1))
		upDirs.push_back(Direction::Up);
	else
	{
		std::vector<GridCell*> cells = sampleExpand(Direction::Up, gridCells, box);
		if(isVoid(cells))
			upDirs.push_back(Direction::Up);
	}

	if(topRightIn.z == (grid.getNumBoxesZ() - 1))
		upDirs.push_back(Direction::In);
	else
	{
		std::vector<GridCell*> cells = sampleExpand(Direction::In, gridCells, box);
		if(isVoid(cells))
			upDirs.push_back(Direction::In);
	}
}

void getFloorVectorsFrom(
	const Grid& grid,
	const MeshBox& meshBox, 
	const std::vector<Direction>& upDirections,
	std::vector<K::Vector_3>& floors)
{
	const Cuboid& bb = meshBox.dims;
	const Vector3D bbEnd = bb.last();

	const K::Point_3 min = grid.get(bb.origin.x, bb.origin.y, bb.origin.z).min();
	const K::Point_3 max = grid.get(bbEnd.x, bbEnd.y, bbEnd.z).max();

	for(Direction direction: upDirections)
	{
		switch(direction)
		{
			case Direction::Left:
			{
				floors.push_back(K::Vector_3(max.x(), 0.0, 0.0));
				break;
			}
			case Direction::Right:
			{
				floors.push_back(K::Vector_3(min.x(), 0.0, 0.0));
				break;
			}
			case Direction::Up:
			{
				floors.push_back(K::Vector_3(0.0, min.y(), 0.0));
				break;
			}
			case Direction::Down:
			{
				floors.push_back(K::Vector_3(0.0, max.y(), 0.0));
				break;
			}
			case Direction::In:
			{
				floors.push_back(K::Vector_3(0.0, 0.0, min.z()));
				break;
			}
			case Direction::Out:
			{
				floors.push_back(K::Vector_3(0.0, 0.0, max.z()));
				break;
			}
		}
	}
}

[[nodiscard]] size_t l1n(const Vector3D& a, const Vector3D& b)
{
	return std::abs(b.x - a.x) + std::abs(b.y - a.y) + std::abs(b.z - a.z);
}

[[nodiscard]] size_t l1nProximity(
	const Vector3D& centroid1, const Vector3D& size1,
	const Vector3D& centroid2, const Vector3D& size2)
{
	Vector3D size1Half(size1.x / 2, size1.y / 2, size1.z / 2);
	Vector3D size2Half(size2.x / 2, size2.y / 2, size2.z / 2);

	Vector3D D = (centroid1 - centroid2).abs();

	Vector3D d = Vector3D(
		D.x - (size1Half.x + size2Half.x),
		D.y - (size1Half.y + size2Half.y),
		D.z - (size1Half.z + size2Half.z));

	return d.x + d.y + d.z;
}

[[nodiscard]] double l2nSq(const Vector3D& a, const Vector3D& b)
{
	const Vector3D c = b - a;
	return (c.x * c.x) + (c.y * c.y) + (c.z * c.z);
}

[[nodiscard]] double l2n(const Vector3D& a, const Vector3D& b)
{
	return std::sqrt(l2nSq(a, b));
}

[[nodiscard]] std::optional<double> proximity(
	Direction direction, const MeshBox& region1, const MeshBox& region2)
{
	const Vector3D centroid1 = region1.dims.centroid();
	const Vector3D centroid2 = region2.dims.centroid();

	const Vector3D delta = centroid2 - centroid1;
	switch(direction)
	{
		case Direction::Left:
			if(delta.x >= 0)
				return std::nullopt;
			break;
		case Direction::Right:
			if(delta.x <= 0)
				return std::nullopt;
			break;
		case Direction::Up:
			if(delta.y <= 0)
				return std::nullopt;
			break;
		case Direction::Down:
			if(delta.y >= 0)
				return std::nullopt;
			break;
		case Direction::In:
			if(delta.z <= 0)
				return std::nullopt;
			break;
		case Direction::Out:
			if(delta.z >= 0)
				return std::nullopt;
			break;
	}

	size_t distance = l1nProximity(
		centroid1, region1.dims.size, centroid2, region2.dims.size);
	
	return distance;
}

[[nodiscard]] double proximityScore(
	Direction direction, 
	const MeshBox& region1, const MeshBox& region2,
	size_t modelLength)
{
	std::optional<double> prox = proximity(direction, region1, region2);
	if(!prox)
		return 1.0;

	double proxCoeff = *prox / static_cast<double>(modelLength);
	
	constexpr double BOUNDARY_COEFF = 0.5;
	return BOUNDARY_COEFF + (BOUNDARY_COEFF * proxCoeff);
}

[[nodiscard]] double computeScore(
	const Config& config,
	const Mesh& parent,
	Direction direction, 
	const MeshBox& region,
	const std::vector<std::unique_ptr<MeshBox>>& regions, 
	const Grid& grid,
	mv::vector3<GridCell>& gridCells)
{
	// TODO: Best objective: higher or lower?
	// Bigger distance = better
	// Bigger area/surf = better
	// Bigger overhang = worse
	// So maybe higher is better?
	
	// Prohibition of Discontinuities
	///////////////////////////////////////////////////////////////////////////
	
	std::optional<Cuboid> newRegion = extendRegionIn(direction, region.dims, grid);
	if(!newRegion)
	{
#ifdef VERBOSE
		std::cout << "\t\tCannot grow " << toText(direction) << "." << std::endl;
#endif
		return 0.0;
	}

	//std::vector<GridCell*> samplesOld, samplesNew;
	//sampleCells(gridCells, region.dims, samplesOld);
	//sampleCells(gridCells, *newRegion, samplesNew);

	//size_t numBoxesOld = enumerateBoundaries(samplesOld);
	//size_t numBoxesNew = enumerateBoundaries(samplesNew);
	
	std::vector<GridCell*> samples = sampleExpand(direction, gridCells, region);

	//std::cout << numBoxesOld << " " << numBoxesNew << std::endl;
	if(samples.empty())
	{
#ifdef VERBOSE
		std::cout << "\t\tSampling " << toText(direction) << " is empty." << std::endl;
#endif
		return 0.0;
	}

	bool empty = true;
	bool anyUnassigned = false;
	bool willOverlap = false;
	for(GridCell* cell: samples)
	{
		if(cell->type == GridCell::ContentType::Boundary)
		{
			empty = false;
			if(cell->parents.empty())
				anyUnassigned = true;
		}

		if(!cell->parents.empty())
			willOverlap = true;
	}

	if(empty)
	{
#ifdef VERBOSE
		std::cout << "\t\tCells " << toText(direction) << " are empty." << std::endl;
#endif
		return -1.0;
	}
	if(willOverlap)
	{
#ifdef VERBOSE
		std::cout << "\t\tGrowth " << toText(direction) << " will overlap." << std::endl;
#endif
		return -1.0;
	}
	if(!anyUnassigned)
	{
#ifdef VERBOSE
		std::cout << "\t\tAssigned cells " << toText(direction) << "." << std::endl;
#endif
		return -1.0;
	}

	// Compute new MeshBox
	///////////////////////////////////////////////////////////////////////////
	
	MeshBox newMeshBox(*newRegion);

	clipFromMesh(grid, parent, newMeshBox);
	
	// Printability Constraint
	///////////////////////////////////////////////////////////////////////////
	
	std::vector<Direction> upVectors;
	enumerateUpDirections(newMeshBox, grid, gridCells, upVectors);

	std::vector<Direction> allowedUpDirs;
	if(!fitsVolume(config, upVectors, newMeshBox.mesh, allowedUpDirs))
	{
#ifdef VERBOSE
		std::cout << "\tGrowth " << toText(direction) << " does not fit volume" << std::endl;
#endif
		return -1.0;
	}

	// Penalisation of Proximity
	///////////////////////////////////////////////////////////////////////////
	
	const size_t modelLength = l1n(
		Vector3D(0, 0, 0), Vector3D(
			grid.getNumBoxesX(), grid.getNumBoxesY(), grid.getNumBoxesZ()));
	double proxScore = 1.0;
	for(const auto& other: regions)
	{
		if(std::addressof(region) == other.get())
			continue;

		double score = proximityScore(direction, region, *other, modelLength);
		if(score < proxScore)
			proxScore = score;
	}

	// Calculate best orientation overhang
	///////////////////////////////////////////////////////////////////////////

	auto fnormals = newMeshBox.mesh.add_property_map<face_descriptor, K::Vector_3>(
		"f:normals", CGAL::NULL_VECTOR).first;
	PMP::compute_face_normals(newMeshBox.mesh, fnormals);
	
	std::vector<K::Vector_3> floors;
	getFloorVectorsFrom(grid, newMeshBox, allowedUpDirs, floors);

	double bestOverhangCost = std::numeric_limits<double>::max();
	for(const K::Vector_3& floor: floors)
	{
		double cost = overhangCost(config, newMeshBox.mesh, fnormals, floor);
		if(cost < 0.0)
		{
			std::cerr << "OOPS?" << std::endl;
		}

		if(cost < bestOverhangCost)
			bestOverhangCost = cost;
	}

	// Application of standard objective
	///////////////////////////////////////////////////////////////////////////

	double printCost = printingCost(config, newMeshBox.mesh);
	
	double score = std::abs(printCost + bestOverhangCost) * proxScore;
#ifdef VERBOSE
	std::cout << "\t\tScore for " << toText(direction)
			  << ":   \t" << score << " [Proximity: " << proxScore
			  << ", Printing: " << printCost 
			  << "]" << std::endl;
#endif

	return score;
}

Direction getBestDirection(const std::unordered_map<Direction,double>& scores)
{
	std::optional<Direction> bestDir = std::nullopt;
	double bestScore;
	for(auto& it: scores)
	{
		if(!bestDir)
		{
			bestDir = it.first;
			bestScore = it.second;
		}
		else
		{
			if(it.second > bestScore)
			{
				bestDir = it.first;
				bestScore = it.second;
			}
		}
	}

	//std::cout << toText(*bestDir) << ": " << bestScore << std::endl;

	assert(bestDir);
	return *bestDir;
}

void assignCellToBox(MeshBox& box, GridCell& cell)
{
	box.children.push_back(std::addressof(cell));
	cell.parents.push_back(std::addressof(box));
}

void assignCellToNearestBox(
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, GridCell& cell)
{
	MeshBox* bestBox = nullptr;
	size_t bestDist = std::numeric_limits<size_t>::max();
	for(auto& box: sourceBoxes)
	{
		size_t dist = l1n(box->dims.origin, cell.position);
		if((dist > 0) && (dist < bestDist))
		{
			bestDist = dist;
			bestBox = box.get();
		}
	}

	assert(bestBox);
	assignCellToBox(*bestBox, cell);
}

void recomputeAABBs(std::vector<std::unique_ptr<MeshBox>>& boxes)
{
	for(auto& box: boxes)
	{
		Vector3D min = box->children.front()->position;
		Vector3D max = box->children.front()->position;
		for(GridCell* child: box->children)
		{
			assert(child);
			if(child->position.x < min.x)
				min.x = child->position.x;
			if(child->position.x > max.x)
				max.x = child->position.x;

			if(child->position.y < min.y)
				min.y = child->position.y;
			if(child->position.y > max.y)
				max.y = child->position.y;

			if(child->position.z < min.z)
				min.z = child->position.z;
			if(child->position.z > max.z)
				max.z = child->position.z;

#ifdef VERBOSE
			std::cout << child->position << " [Min: " << min << ", Max: " << max << "]" << std::endl;
#endif
		}
		box->dims = Cuboid(min, (max - min) + Vector3D(1));
#ifdef VERBOSE
		std::cout << box->dims << std::endl;
#endif
	}
}

void fastAssign(
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes,
	mv::vector3<GridCell>& gridCells)
{
	mv::forEach<GridCell>([&](GridCell& cell) {
		if(cell.type == GridCell::ContentType::Boundary && cell.parents.empty())
		{
			assignCellToNearestBox(sourceBoxes, cell);
		}
	}, gridCells);

	recomputeAABBs(sourceBoxes);
}

bool growBoxIfPossible(
	const Config& config,
	const Mesh& parent,
	MeshBox& targetBox,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes,
	mv::vector3<GridCell>& gridCells,
	Grid& grid)
{
#ifdef VERBOSE
	std::cout << "\tComputing score for " << std::addressof(targetBox)
	          << " " << targetBox.dims
	          << ":" << std::endl;
#endif

	const std::array<Direction,6> directions = {
		Direction::Left, Direction::Right,
		Direction::Up, Direction::Down,
		Direction::In, Direction::Out
	};
	std::unordered_map<Direction,double> scores;

	for(Direction direction: directions)
	{
		double score = computeScore(
			config, parent,
			direction,
			targetBox, sourceBoxes,
			grid, gridCells);
		scores[direction] = score;
	}

	Direction bestDirection = getBestDirection(scores);
	if(scores[bestDirection] > 0.0)
	{
#ifdef VERBOSE
		std::cout << "\tBest Score: " << toText(bestDirection)
		          << " = " << scores[bestDirection] << std::endl;
#endif
		grow(bestDirection, gridCells, targetBox);
		return true;
	}
	else
	{
#ifdef VERBOSE
		std::cout << "\tNo possible directions detected." << std::endl;
#endif
		return false;
	}
}

[[nodiscard]]
bool continueRegionGrowth(
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
	mv::vector3<GridCell>& gridCells)
{
	size_t initial = 0;
	size_t remainingCells = mv::reduce<GridCell,size_t>([](size_t& count, const GridCell& cell) {
		if(cell.type == GridCell::ContentType::Boundary)
		{
			if(cell.parents.empty())
				++count;
		}
	}, gridCells, initial);

#ifdef VERBOSE
	std::cout << "Remaining blocks: " << remainingCells << std::endl;
#endif

	return remainingCells > 0;
}

void saveMeshes(
	const std::string& directory,
	const std::vector<std::unique_ptr<MeshBox>>& meshes)
{
	unsigned int meshIndex = 0;
	for(const auto& mesh: meshes)
	{
		std::stringstream ss("");
		ss << meshIndex << ".stl";

		if(CGAL::IO::write_STL(directory + "/" + ss.str(), mesh->mesh))
			std::cout << "Saved " << directory << "/" << ss.str() << std::endl;
		else
			std::cerr << "Failed to write " << ss.str() << "!" << std::endl;

		++meshIndex;
	}
}

void regionGrowth(
	const Config& config,
	const Mesh& parent,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
	mv::vector3<GridCell>& gridCells,
	Grid& grid)
{
#if USE_CLUSTERS == true
	fastAssign(sourceBoxes, gridCells);

	#pragma omp parallel for default(none) shared(sourceBoxes, grid, parent)
	for(auto& sourceBox: sourceBoxes)
	{
		clipFromMesh(grid, parent, *sourceBox);
	}

	bool test = continueRegionGrowth(sourceBoxes, gridCells);
	printMeshBoxes(sourceBoxes);
#else
	unsigned int iterNum = 1;
	while(continueRegionGrowth(sourceBoxes, gridCells))
	{
#ifdef VERBOSE
		std::cout << "Region Growth Iteration " << iterNum << std::endl;
#endif

		typedef std::pair<MeshBox*,double> BoxScore;

		std::list<BoxScore> rankedBoxes;
		for(auto& sourceBox: sourceBoxes)
		{
			double printCost = printingCost(config, sourceBox->mesh);
			rankedBoxes.emplace_back(sourceBox.get(), printCost);
		}

		rankedBoxes.sort([](const BoxScore& a, const BoxScore& b) {
			return a.second < b.second;
		});

		bool onePassed = false;
		for(auto& box: rankedBoxes)
		{
			if(growBoxIfPossible(
				config, parent, *box.first, sourceBoxes, gridCells, grid))
			{
				onePassed = true;
				break;
			}
		}

		// Recompute meshes
		// (Not sure if necessary, but for sanity at this stage)
		#pragma omp parallel for default(none) shared(sourceBoxes, grid, parent)
		for(auto& sourceBox: sourceBoxes)
			clipFromMesh(grid, parent, *sourceBox);

		if(!onePassed)
		{
#ifdef VERBOSE
			std::cout << "Algorithm has locked." << std::endl;
#endif
			return;
		}

		if(!enumerateConflicts(sourceBoxes, gridCells))
		{
			std::cout << std::endl << std::endl << std::endl << std::endl;
			std::cout << "ERROR: SHOULD NOT OVERLAP!!!!" << std::endl;
			return;
		}

		++iterNum;
	}

#if EXPORT_INTERMEDIATES == true
	std::stringstream ss("");
	ss << config.outputDir << "/dir" << iterNum;
	fs::create_directory(ss.str());

	saveMeshes(ss.str(), sourceBoxes);
#endif
#endif
}

