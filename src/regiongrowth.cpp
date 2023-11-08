#include "regiongrowth.h"
#include "objective.h"
#include "multivec.h"
#include <unordered_map>
#include <optional>

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

    for(int x = a.x; x <= b.x; ++x) 
	{
        for(int y = a.y; y <= b.y; ++y) 
		{
            for(int z = a.z; z <= b.z; ++z) 
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
				btmLeftOut + Vector3D(0, box.dims.size.y, box.dims.size.z);

			return sampleCells(gridCells, btmLeftOut, topLeftIn);
		}
		break;                                                      
		case Direction::Right:                                          
		{
			//std::cout << "Growing Right..." << std::endl;

			const Vector3D btmRightOut = 
				box.dims.origin + Vector3D(box.dims.size.x + 1, 0, 0);
			const Vector3D topRightIn = 
				btmRightOut + Vector3D(0, box.dims.size.y, box.dims.size.z);

			return sampleCells(gridCells, btmRightOut, topRightIn);
		}
		break;                                                 
		case Direction::Up:                                             
		{
			//std::cout << "Growing Up..." << std::endl;

			const Vector3D topLeftOut = 
				box.dims.origin + Vector3D(0, box.dims.size.y + 1, 0);
			const Vector3D topRightIn = 
				topLeftOut + Vector3D(box.dims.size.x, 0, box.dims.size.z);

			return sampleCells(gridCells, topLeftOut, topRightIn);
		}
		break;                                                      
		case Direction::Down:                                           
		{
			//std::cout << "Growing Down..." << std::endl;

			Vector3D btmLeftOut = box.dims.origin;
			--btmLeftOut.y;

			Vector3D btmRightIn = 
				btmLeftOut + Vector3D(box.dims.size.x, 0, box.dims.size.z);

			return sampleCells(gridCells, btmLeftOut, btmRightIn);
		}
		break;                                                      
		case Direction::In:                                             
		{
			//std::cout << "Growing In..." << std::endl;

			const Vector3D btmLeftIn = 
				box.dims.origin + Vector3D(0, 0, box.dims.size.z + 1);
			const Vector3D topRightIn = 
				btmLeftIn + Vector3D(box.dims.size.x, box.dims.size.y, 0);

			return sampleCells(gridCells, btmLeftIn, topRightIn);
		}
		break;                                                      
		case Direction::Out:                                            
		{
			//std::cout << "Growing Out..." << std::endl;

			Vector3D btmLeftOut = box.dims.origin;
			--btmLeftOut.z;

			Vector3D topRightOut = 
				btmLeftOut + Vector3D(box.dims.size.x, box.dims.size.y, 0);

			return sampleCells(gridCells, btmLeftOut, topRightOut);
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

			--box.dims.origin.x;
			++box.dims.size.x;

			const Vector3D& btmLeftOut = box.dims.origin;
			const Vector3D topLeftIn = 
				btmLeftOut + Vector3D(0, box.dims.size.y, box.dims.size.z);

			std::vector<GridCell*> cells = sampleCells(gridCells, btmLeftOut, topLeftIn);
			addCellsToMeshBox(box, cells);
		}
		break;                                                      
		case Direction::Right:                                          
		{
			//std::cout << "Growing Right..." << std::endl;

			++box.dims.size.x;

			const Vector3D btmRightOut = 
				box.dims.origin + Vector3D(box.dims.size.x, 0, 0);
			const Vector3D topRightIn = 
				btmRightOut + Vector3D(0, box.dims.size.y, box.dims.size.z);

			std::vector<GridCell*> cells = sampleCells(gridCells, btmRightOut, topRightIn);
			addCellsToMeshBox(box, cells);
		}
		break;                                                 
		case Direction::Up:                                             
		{
			//std::cout << "Growing Up..." << std::endl;

			++box.dims.size.y;

			const Vector3D topLeftOut = 
				box.dims.origin + Vector3D(0, box.dims.size.y, 0);
			const Vector3D topRightIn = 
				topLeftOut + Vector3D(box.dims.size.x, 0, box.dims.size.z);

			std::vector<GridCell*> cells = sampleCells(gridCells, topLeftOut, topRightIn);
			addCellsToMeshBox(box, cells);
		}
		break;                                                      
		case Direction::Down:                                           
		{
			//std::cout << "Growing Down..." << std::endl;

			--box.dims.origin.y;
			++box.dims.size.y;

			const Vector3D& btmLeftOut = box.dims.origin;
			const Vector3D btmRightIn = 
				btmLeftOut + Vector3D(box.dims.size.x, 0, box.dims.size.z);

			std::vector<GridCell*> cells = sampleCells(gridCells, btmLeftOut, btmRightIn);
			addCellsToMeshBox(box, cells);
		}
		break;                                                      
		case Direction::In:                                             
		{
			//std::cout << "Growing In..." << std::endl;

			++box.dims.size.z;

			const Vector3D btmLeftIn = 
				box.dims.origin + Vector3D(0, 0, box.dims.size.z);
			const Vector3D topRightIn = 
				btmLeftIn + Vector3D(box.dims.size.x, box.dims.size.y, 0);

			std::vector<GridCell*> cells = sampleCells(gridCells, btmLeftIn, topRightIn);
			addCellsToMeshBox(box, cells);
		}
		break;                                                      
		case Direction::Out:                                            
		{
			//std::cout << "Growing Out..." << std::endl;

			--box.dims.origin.z;
			++box.dims.size.z;

			const Vector3D& btmLeftOut = box.dims.origin;
			const Vector3D topRightOut = 
				btmLeftOut + Vector3D(box.dims.size.x, box.dims.size.y, 0);

			std::vector<GridCell*> cells = sampleCells(gridCells, btmLeftOut, topRightOut);
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
			if(cuboid.end().x >= grid.getNumBoxesX())
				return std::nullopt;
		}
		break;                                                 
		case Direction::Up:                                             
		{
			++cuboid.size.y;
			if(cuboid.end().y >= grid.getNumBoxesY())
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
			if(cuboid.end().z >= grid.getNumBoxesZ())
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
		return 0.0;

	//std::vector<GridCell*> samplesOld, samplesNew;
	//sampleCells(gridCells, region.dims, samplesOld);
	//sampleCells(gridCells, *newRegion, samplesNew);

	//size_t numBoxesOld = enumerateBoundaries(samplesOld);
	//size_t numBoxesNew = enumerateBoundaries(samplesNew);
	
	std::vector<GridCell*> samples = sampleExpand(direction, gridCells, region);

	//std::cout << numBoxesOld << " " << numBoxesNew << std::endl;
	if(samples.empty())
		return 0.0;

	bool empty = true;
	bool unassigned = false;
	for(GridCell* cell: samples)
	{
		if(cell->type == GridCell::ContentType::Boundary)
		{
			empty = false;
			if(cell->parents.empty())
				unassigned = true;
		}
	}

	if(empty)
		return 0.0;
	if(!unassigned)
		return 0.0;

	// Compute new MeshBox
	///////////////////////////////////////////////////////////////////////////
	
	MeshBox newMeshBox(*newRegion);

	clipFromMesh(grid, parent, newMeshBox);
	
	// Printability Constraint
	///////////////////////////////////////////////////////////////////////////
	
	//std::cout << fitsVolume(config, newMeshBox.mesh) << std::endl;
	if(!fitsVolume(config, newMeshBox.mesh))
		return 0.0;

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

	// Application of Standard Objective
	// (Penalisation of Overhang, Incentivisation of Size)
	///////////////////////////////////////////////////////////////////////////

	double printCost = printingCost(config, newMeshBox.mesh);
	double overhangCost = /*overhangCost()*/ 0.0;
	// TODO: What the fuck is with our overhang function parameters?!??!
	// TODO: Also: overhang area should be penalised! So, negate?
	
	double score = std::abs(printCost + overhangCost) * proxScore;
	std::cout << "\t\tScore for " << toText(direction)
			  << ":   \t" << score << " [Proximity: " << proxScore
			  << ", Printing: " << printCost 
			  << "]" << std::endl;

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

			std::cout << child->position << " [Min: " << min << ", Max: " << max << "]" << std::endl;
		}
		box->dims = Cuboid(min, (max - min) + Vector3D(1));
		std::cout << box->dims << std::endl;
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

	std::cout << "Remaining blocks: " << remainingCells << std::endl;

	return remainingCells > 0;
}

void regionGrowth(
	const Config& config,
	const Mesh& parent,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
	mv::vector3<GridCell>& gridCells,
	Grid& grid)
{
	fastAssign(sourceBoxes, gridCells);

	#pragma omp parallel for default(none) shared(sourceBoxes, grid, parent)
	for(auto& sourceBox: sourceBoxes)
	{
		clipFromMesh(grid, parent, *sourceBox);
	}

	bool test = continueRegionGrowth(sourceBoxes, gridCells);
	printMeshBoxes(sourceBoxes);


	/*unsigned int iterNum = 1;
	while(continueRegionGrowth(sourceBoxes, gridCells))
	{
		std::cout << "Region Growth Iteration " << iterNum << std::endl;

		// Enumerate best directions of growth
		// TODO: Enable parallelism
		//#pragma omp parallel for
		
		MeshBox* worstBox = nullptr;
		double worstCost = std::numeric_limits<double>::max();
		for(auto& sourceBox: sourceBoxes)
		{
			double printCost = printingCost(config, sourceBox->mesh);
			if(printCost < worstCost)
			{
				worstBox = sourceBox.get();
				worstCost = printCost;
			}
		}

		std::cout << "\tComputing score for " << worstBox
				  << " " << worstBox->dims
				  << ":" << std::endl;

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
				*worstBox, sourceBoxes, 
				grid, gridCells);
			scores[direction] = score;
		}

		Direction bestDirection = getBestDirection(scores);
		if(scores[bestDirection] > 0.0)
		{
			std::cout << "\tBest Score: " << toText(bestDirection)
					  << " = " << scores[bestDirection] << std::endl;
			//#pragma omp critical
			grow(bestDirection, gridCells, *worstBox);
		}
		else
			std::cout << "\tNo possible directions detected." << std::endl;

		// Recompute meshes
		// (Not sure if necessary, but for sanity at this stage)
		#pragma omp parallel for
		for(auto& sourceBox: sourceBoxes)
			clipFromMesh(grid, parent, *sourceBox);

		++iterNum;
	}*/
}
