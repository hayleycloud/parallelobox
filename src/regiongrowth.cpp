#include "regiongrowth.h"
#include "objective.h"
#include "multivec.h"
#include <unordered_map>
#include <optional>
#include <filesystem>
#include <sstream>

namespace fs = std::filesystem;

#define EXPORT_INTERMEDIATES	false
#define USE_CLUSTERS    		false

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

void grow(
	Direction direction, 
	const Mesh& parent,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells, 
	MeshBox& box)
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

	clipFromMesh(grid, parent, box);
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

inline void seReportError(Direction direction, const std::string& msg)
{
	std::cout << "\t\tCannot grow " << toText(direction) << ": " << msg << std::endl;
}

std::optional<Cuboid> safeExpand(
	Direction direction,
	const Cuboid& region,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells)
{
	std::optional<Cuboid> newRegion = extendRegionIn(direction, region, grid);
	if(!newRegion)
	{
#ifdef VERBOSE
		seReportError(direction, "Out Of Grid.");
#endif
		return std::nullopt;
	}

	std::vector<GridCell*> newCells = sampleExpand(direction, gridCells, region);
	if(newCells.empty())
	{
#ifdef VERBOSE
		seReportError(direction, "Into Void.");
#endif
		return std::nullopt;
	}

	bool anyBoundaries = false;
	bool anyUnassigned = false;
	bool willOverlap = false;
	for(GridCell* cell: newCells)
	{
		if(cell->type == GridCell::ContentType::Boundary)
		{
			anyBoundaries = true;
			if(cell->parents.empty())
				anyUnassigned = true;
		}

		if(!cell->parents.empty())
			willOverlap = true;
	}

	if(!anyBoundaries)
	{
#ifdef VERBOSE
		seReportError(direction, "No Boundary Cells.");
#endif
		return std::nullopt;
	}
	if(willOverlap)
	{
#ifdef VERBOSE
		seReportError(direction, "Overlap Not Allowed.");
#endif
		return std::nullopt;
	}
	if(!anyUnassigned)
	{
#ifdef VERBOSE
		seReportError(direction, "Assigned Cells Detected!");
#endif
		return std::nullopt;
	}

	return newRegion;
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
	std::cout << d << std::endl;

	return d.x + d.y + d.z;
}

// 1.0 is ahead, 0.0 is off to the side (perpendicular)
[[nodiscard]] double directionalFactor(
	Direction direction,
	const MeshBox& source, const MeshBox& other)
{
	const Vector3D dc = other.dims.centroid() - source.dims.centroid();
	const K::Vector_3 dc2 = normalize(K::Vector_3(dc.x, dc.y, dc.z));

	K::Vector_3 dirVec = toVector(direction);

	return std::max(0.0, dirVec * dc2);
}

[[nodiscard]] int diffBetween(int minA, int maxA, int minB, int maxB)
{
	if(maxA < minB)
		return minB - maxA;
	else if(maxB < minA)
		return minA - maxB;
	return 0;
}

[[nodiscard]] Vector3D diffBetween(const Cuboid& a, const Cuboid& b)
{
	const Vector3D maxA = a.end(), maxB = b.end();
	return {
		diffBetween(a.origin.x, maxA.x, b.origin.x, maxB.x),
		diffBetween(a.origin.y, maxA.y, b.origin.y, maxB.y),
		diffBetween(a.origin.z, maxA.z, b.origin.z, maxB.z)
	};
}

[[nodiscard]] int lInf(const Vector3D& d)
{
	return std::max(d.x, std::max(d.y, d.z));
}

[[nodiscard]] double lInfProximity(const Cuboid& a, const Cuboid& b)
{
	return lInf(diffBetween(a, b));
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

[[nodiscard]] bool isGrowthAway(
	Direction direction, const MeshBox& region1, const MeshBox& region2)
{
	const Vector3D delta = region2.dims.centroid() - region1.dims.centroid();
	switch(direction)
	{
		case Direction::Left:
			return delta.x >= 0;
		case Direction::Right:
			return delta.x <= 0;
		case Direction::Up:
			return delta.y <= 0;
		case Direction::Down:
			return delta.y >= 0;
		case Direction::In:
			return delta.z <= 0;
		case Direction::Out:
			return delta.z >= 0;
	}
	return false;
}

[[nodiscard]] double proximitylInf(
	Direction direction, const MeshBox& region1, const MeshBox& region2)
{
	if(isGrowthAway(direction, region1, region2))
		return -1.0;

	return lInfProximity(region1.dims, region2.dims);
}

[[nodiscard]] std::optional<double> proximityl1n(
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

[[nodiscard]] double proximityCost(
	Direction direction, 
	const MeshBox& region1, const MeshBox& region2,
	size_t modelLength)
{
	std::optional<double> prox = proximityl1n(direction, region1, region2);
	if(!prox)
		return 1.0;

	double proxCoeff = *prox / static_cast<double>(modelLength);
	
	constexpr double BOUNDARY_COEFF = 0.5;
	return BOUNDARY_COEFF + (BOUNDARY_COEFF * proxCoeff);
}

[[nodiscard]] double proximityCost(
	Direction direction,
	const MeshBox& source,
	const std::vector<const MeshBox*>& regions)
{
	constexpr double EPSILON = 0.5;

	double totalCost = 0.0;
	for(const MeshBox* other: regions)
	{
		//double dist = lInfProximity(source.dims, other->dims);
		double dist = proximitylInf(direction, source, *other);
		if(dist < 0.0)
			continue;
		//std::cout << toText(direction) << " " << directionalFactor(direction, source, *other) << " " << source.dims.centroid() << " " << other->dims.centroid() << " " << 
			//other->dims.centroid() - source.dims.centroid() << std::endl;

		// TODO: Does this need negating? df() says 1.0 if ahead yes?
		// Larger scores for aheadness
		double dirFactor = (1.0 - directionalFactor(direction, source, *other));
		double distCost = dirFactor / (1.0 / (dist + EPSILON));
		double distCost2 = distCost * distCost;
		//std::cout << dist << " => " << distCost2 << std::endl;;
		//std::cout << toText(direction) << " " << dist << " " << directionalDistCost << std::endl;
		totalCost += distCost2;
	}

	return totalCost;
}	

[[nodiscard]] double computeCost(
	const Config& config,
	const Mesh& parent,
	double currentPrintCost,
	Direction direction, 
	const MeshBox& region,
	const std::vector<std::unique_ptr<MeshBox>>& regions, 
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache)
{
	// REMEMBER: Lower is better, negative not allowed
	
	// Exclude target from comparison set
	std::vector<const MeshBox*> others;
	for(const auto& other: regions)
	{
		if(std::addressof(region) != other.get())
			others.push_back(other.get());
	}
	
	// Grow and Test for Constraint Violations
	///////////////////////////////////////////////////////////////////////////
	
	std::optional<Cuboid> newRegion = 
		safeExpand(direction, region.dims, grid, gridCells);
	if(!newRegion)
		return -1.0;

	// Compute new MeshBox
	///////////////////////////////////////////////////////////////////////////
	
	MeshBox newMeshBox(*newRegion);

	if(INVALID(clipFromMesh(grid, parent, newMeshBox)))
		return -1.0;
	
	// Printability Constraint
	///////////////////////////////////////////////////////////////////////////
	
	if(!fitsVolume(config, newMeshBox.mesh))
	{
#ifdef VERBOSE
		std::cout << "\tGrowth " << toText(direction) << " does not fit volume" << std::endl;
#endif
		return -1.0;
	}

	// Standard Objective
	///////////////////////////////////////////////////////////////////////////
	// This accounts for core printing cost + minimal support structure print costs
	double printCost = printingCost(config, grid, newMeshBox);
	double dPrintCost = printCost - currentPrintCost;

	// Penalisation of Proximity
	///////////////////////////////////////////////////////////////////////////
	
	const size_t modelLength = l1n(
		Vector3D(0, 0, 0), Vector3D(
			grid.getNumBoxesX(), grid.getNumBoxesY(), grid.getNumBoxesZ()));
	double beta = 0.5;
	double lambda = currentPrintCost * beta;
	double freeProxCost = proximityCost(direction, newMeshBox, others);
	double proxCost = lambda * freeProxCost;

	//std::cout << "\t\t" << proxCost << " (" << freeProxCost << ")" << std::endl;

	// Application of standard objective
	///////////////////////////////////////////////////////////////////////////

	double score = dPrintCost + proxCost;
#ifdef VERBOSE
	std::cout << "\t\tScore for " << toText(direction)
			  << ":   \t" << score << " [Proximity: " << proxCost
			  << ", +Printing: " << dPrintCost 
			  << "]" << std::endl;
#endif

	return score;
}

std::optional<Direction> getBestDirection(
	const std::unordered_map<Direction,double>& costs)
{
	std::optional<Direction> best = std::nullopt;
	double bestCost = std::numeric_limits<double>::max();
	for(auto& it: costs)
	{
		if(it.second >= 0.0 && it.second < bestCost)
		{
			best = it.first;
			bestCost = it.second;
		}
	}

	//std::cout << toText(*bestDir) << ": " << bestScore << std::endl;

	return best;
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
	double currentPrintCost,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes,
	mv::vector3<GridCell>& gridCells,
	const Grid& grid,
	PrintingCostCache& printCostCache)
{
#ifdef VERBOSE
	std::cout << "\tComputing cost for " << std::addressof(targetBox)
	          << " " << targetBox.dims
	          << ":" << std::endl;
#endif

	const std::array<Direction,6> directions = {
		Direction::Left, Direction::Right,
		Direction::Up, Direction::Down,
		Direction::In, Direction::Out
	};
	std::unordered_map<Direction,double> costs;
	for(Direction direction: directions)
		costs[direction] = -1.0;

	#pragma omp parallel for default(none) shared(directions, costs, config, parent, currentPrintCost, targetBox, sourceBoxes, grid, gridCells, printCostCache)
	for(Direction direction: directions)
	{
		costs[direction] = computeCost(
			config, parent,
			currentPrintCost,
			direction,
			targetBox, sourceBoxes,
			grid, gridCells,
			printCostCache);
	}

	std::optional<Direction> bestDirection = getBestDirection(costs);
	if(!bestDirection)
	{
#ifdef VERBOSE
		std::cout << "\tNo possible directions detected." << std::endl;
#endif
		return false;
	}

#ifdef VERBOSE
	std::cout << "\tBest Cost: " << toText(*bestDirection)
			  << " = " << costs[*bestDirection] << std::endl;
#endif
	grow(*bestDirection, parent, grid, gridCells, targetBox);

	return true;
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

/*void saveMeshes(
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
}*/

void regionGrowth(
	const Config& config,
	const Mesh& parent,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache)
{
#if USE_CLUSTERS == true
	fastAssign(sourceBoxes, gridCells);

	#pragma omp parallel for default(none) shared(sourceBoxes, grid, parent)
	for(auto& sourceBox: sourceBoxes)
		clipFromMesh(grid, parent, *sourceBox);

	bool test = continueRegionGrowth(sourceBoxes, gridCells);
	printMeshBoxes(sourceBoxes);
#else
	typedef std::pair<MeshBox*,double> BoxScore;

	unsigned int iterNum = 1;
	while(continueRegionGrowth(sourceBoxes, gridCells))
	{
#ifdef VERBOSE
		std::cout << "Region Growth Iteration " << iterNum << std::endl;
#endif

		std::list<BoxScore> rankedBoxes;
		for(auto& sourceBox: sourceBoxes)
			rankedBoxes.emplace_back(
				sourceBox.get(), 
				printingCost(config, grid, *sourceBox, printCostCache));

		// Fastest part so far gets priority
		rankedBoxes.sort([](const BoxScore& a, const BoxScore& b) {
			return a.second < b.second;
		});

		bool onePassed = false;
		for(auto& box: rankedBoxes)
		{
			if(growBoxIfPossible(
				config, parent, 
				*box.first, box.second, 
				sourceBoxes, gridCells, grid,
				printCostCache))
			{
				onePassed = true;
				break;
			}
		}

		// Recompute meshes
		// (Not sure if necessary, but for sanity at this stage)
		//#pragma omp parallel for default(none) shared(sourceBoxes, grid, parent)
		//for(auto& sourceBox: sourceBoxes)
		//	clipFromMesh(grid, parent, *sourceBox);

		if(!onePassed)
		{
#ifdef VERBOSE
			std::cout << "Algorithm has locked." << std::endl;
#endif
			return;
		}

		pruneCache(sourceBoxes, printCostCache);

		//if(!enumerateConflicts(sourceBoxes, gridCells))
		//{
		//	std::cout << std::endl << std::endl << std::endl << std::endl;
		//	std::cout << "ERROR: SHOULD NOT OVERLAP!!!!" << std::endl;
		//	return;
		//}

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

 
