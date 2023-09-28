#include "regiongrowth.h"
#include "objective.h"
#include "multivec.h"
#include <unordered_map>
#include <optional>

void addCellsToMeshBox(
	mv::vector3<GridCell>& gridCells, 
	MeshBox& box,
	const Vector3D& a, const Vector3D& b) 
{
	MeshBox* boxPtr = std::addressof(box);
    for(int x = a.x; x < b.x; x++) 
	{
        for(int y = a.y; y < b.y; y++) 
		{
            for(int z = a.z; z < b.z; z++) 
			{
				GridCell* cell = std::addressof(mv::get(gridCells, x, y, z));

				assert(cell);
				assert(std::find(
					cell->parents.cbegin(), cell->parents.cend(), boxPtr)
						== cell->parents.cend());

				box.children.push_back(cell);
				cell->parents.push_back(boxPtr);
            }
        }
    }
}

void grow(Direction direction, mv::vector3<GridCell>& gridCells, MeshBox& box)
{
	// Direction indicates the side being grown!
	switch(direction)
	{
		case Direction::Left:
		{
			std::cout << "Growing Left..." << std::endl;

			--box.dims.origin.x;
			++box.dims.size.x;

			const Vector3D& btmLeftOut = box.dims.origin;
			const Vector3D topLeftIn = 
				btmLeftOut + Vector3D(0, box.dims.size.y, box.dims.size.z);
			addCellsToMeshBox(gridCells, box, btmLeftOut, topLeftIn);
		}
		break;                                                      
		case Direction::Right:                                          
		{
			std::cout << "Growing Right..." << std::endl;

			++box.dims.size.x;

			const Vector3D btmRightOut = 
				box.dims.origin + Vector3D(box.dims.size.x, 0, 0);
			const Vector3D topRightIn = 
				btmRightOut + Vector3D(0, box.dims.size.y, box.dims.size.z);
			addCellsToMeshBox(gridCells, box, btmRightOut, topRightIn);
		}
		break;                                                 
		case Direction::Up:                                             
		{
			std::cout << "Growing Up..." << std::endl;

			++box.dims.size.y;

			const Vector3D topLeftOut = 
				box.dims.origin + Vector3D(0, box.dims.size.y, 0);
			const Vector3D topRightIn = 
				topLeftOut + Vector3D(box.dims.size.x, 0, box.dims.size.z);
			addCellsToMeshBox(gridCells, box, topLeftOut, topRightIn);
		}
		break;                                                      
		case Direction::Down:                                           
		{
			std::cout << "Growing Down..." << std::endl;

			--box.dims.origin.y;
			++box.dims.size.y;

			const Vector3D& btmLeftOut = box.dims.origin;
			const Vector3D btmRightIn = 
				btmLeftOut + Vector3D(box.dims.size.x, 0, box.dims.size.z);
			addCellsToMeshBox(gridCells, box, btmLeftOut, btmRightIn);
		}
		break;                                                      
		case Direction::In:                                             
		{
			std::cout << "Growing In..." << std::endl;

			++box.dims.size.z;

			const Vector3D btmLeftIn = 
				box.dims.origin + Vector3D(0, 0, box.dims.size.z);
			const Vector3D topRightIn = 
				btmLeftIn + Vector3D(box.dims.size.x, box.dims.size.y, 0);
			addCellsToMeshBox(gridCells, box, btmLeftIn, topRightIn);
		}
		break;                                                      
		case Direction::Out:                                            
		{
			std::cout << "Growing Out..." << std::endl;

			--box.dims.origin.z;
			++box.dims.size.z;

			const Vector3D& btmLeftOut = box.dims.origin;
			const Vector3D topRightOut = 
				btmLeftOut + Vector3D(box.dims.size.x, box.dims.size.y, 0);
			addCellsToMeshBox(gridCells, box, btmLeftOut, topRightOut);
		}
		break;
	}
}

void resetMeshBoxChildren(MeshBox& meshBox)
{
	GridCell* origin = meshBox.children.front();
	meshBox.children.clear();
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
			// TODO: Magic!
			if(bestScore < it.second)
			{
				bestDir = it.first;
				bestScore = it.second;
			}
		}
	}

	assert(bestDir);
	return *bestDir;
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
			if(cuboid.end().x >= grid.getWidth())
				return std::nullopt;
		}
		break;                                                 
		case Direction::Up:                                             
		{
			++cuboid.size.y;
			if(cuboid.end().y >= grid.getHeight())
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
			if(cuboid.end().z >= grid.getDepth())
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

[[nodiscard]] size_t enumerateBoundaries(
	size_t& accum, const GridCell* cell)
{
	if(cell->type == GridCell::ContentType::Boundary)
		++accum;
	return accum;
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

[[nodiscard]] double proximityScore(
	Direction direction, const MeshBox& region1, const MeshBox& region2)
{
	const Vector3D centroid1 = region1.dims.origin + Vector3D(
		region1.dims.size.x / 2, region1.dims.size.y / 2, region1.dims.size.z / 2);
	const Vector3D centroid2 = region2.dims.origin + Vector3D(
		region2.dims.size.x / 2, region2.dims.size.y / 2, region2.dims.size.z / 2);

	const Vector3D delta = centroid2 - centroid1;
	switch(direction)
	{
		case Direction::Left:
			if(delta.x >= 0)
				return 0;
			break;
		case Direction::Right:
			if(delta.x <= 0)
				return 0;
			break;
		case Direction::Up:
			if(delta.y <= 0)
				return 0;
			break;
		case Direction::Down:
			if(delta.y >= 0)
				return 0;
			break;
		case Direction::In:
			if(delta.z <= 0)
				return 0;
			break;
		case Direction::Out:
			if(delta.z >= 0)
				return 0;
			break;
	}
	
	return l2nSq(centroid1, centroid2);
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
		return 0;

	std::vector<GridCell*> samplesOld, samplesNew;
	sampleCells(gridCells, region.dims, samplesOld);
	sampleCells(gridCells, *newRegion, samplesNew);

	size_t numBoxesOld = mv::reduce(enumerateBoundaries, samplesOld);
	size_t numBoxesNew = mv::reduce(enumerateBoundaries, samplesNew);

	if(numBoxesOld == numBoxesNew)
		return 0;

	// Compute new MeshBox
	///////////////////////////////////////////////////////////////////////////
	
	MeshBox newMeshBox(*newRegion);

	clipFromMesh(grid, parent, newMeshBox);

	// Penalisation of Proximity
	///////////////////////////////////////////////////////////////////////////
	
	double proxScore = 0.0;
	for(const auto& other: regions)
		proxScore += proximityScore(direction, region, *other);

	// Application of Standard Objective
	// (Penalisation of Overhang, Incentivisation of Size)
	///////////////////////////////////////////////////////////////////////////
	
	if(!fitsVolume(config, newMeshBox.mesh))
		return 0;

	double printCost = printingCost(config, newMeshBox.mesh);
	double overhangCost = /*overhangCost()*/ 0.0;
	// TODO: What the fuck is with our overhang function parameters?!??!
	// TODO: Also: overhang area should be penalised! So, negate?

	return proxScore + printCost + overhangCost;
}

[[nodiscard]]
bool continueRegionGrowth(
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
	mv::vector3<GridCell>& gridCells)
{
	// If we can find a boundary box with no parent, we still have expansion to do
	const auto* pendingMeshCell = mv::find<GridCell>([](const GridCell& cell) {
		if(cell.type == GridCell::ContentType::Boundary)
			return cell.parents.size() > 0;

		return false;
	}, gridCells);

	return pendingMeshCell;
}

void regionGrowth(
	const Config& config,
	const Mesh& parent,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
	mv::vector3<GridCell>& gridCells,
	Grid& grid)
{
	struct MeshBoxGrow
	{
		MeshBox* meshBox;
		Direction direction;
	};

	while(continueRegionGrowth(sourceBoxes, gridCells))
	{
		// Enumerate best directions of growth
		std::vector<MeshBoxGrow> expandDirections;
		#pragma omp parallel for
		for(auto& sourceBox: sourceBoxes)
		{
			std::unordered_map<Direction,double> scores;
			
			const std::array<Direction,6> directions = {
				Direction::Left, Direction::Right,
				Direction::Up, Direction::Down,
				Direction::In, Direction::Out
			};

			for(Direction direction: directions)
			{
				scores[direction] = computeScore(
					config, parent,
					direction, 
					*sourceBox, sourceBoxes, 
					grid, gridCells);
			}

			Direction bestDirection = getBestDirection(scores);
			#pragma omp critical
			expandDirections.push_back((MeshBoxGrow){
				sourceBox.get(), bestDirection});
		}

		// Apply region growth
		//#pragma omp parallel for		<- Don't parallel yet
		for(MeshBoxGrow& expansion: expandDirections)
		{
			//resetMeshBoxChildren(expansion.first);
			grow(expansion.direction, gridCells, *expansion.meshBox);
		}

		// Recompute meshes
		// (Not sure if necessary, but for sanity at this stage)
		#pragma omp parallel for
		for(auto& sourceBox: sourceBoxes)
			clipFromMesh(grid, parent, *sourceBox);
	}
}
