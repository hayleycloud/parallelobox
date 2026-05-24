#include "fillempty.h"
#include "regiongrowth.h"


/*std::vector<const GridCell*> sampleCells(
	const mv::vector3<GridCell>& gridCells,
	const Vector3D& a, const Vector3D& b)
{
	std::vector<const GridCell*> cells;

    for(int x = a.x; x < b.x; ++x)
	{
        for(int y = a.y; y < b.y; ++y)
		{
            for(int z = a.z; z < b.z; ++z)
			{
				const GridCell* cell = std::addressof(mv::get(gridCells, x, y, z));
				assert(cell);
				cells.push_back(cell);
            }
        }
    }

	return cells;
}

[[nodiscard]] K::Vector_3 cubeInReal(const Grid& grid, const Cuboid& cube)
{
	double elementSize = grid.getElementSize();
	return {
		cube.size.x * elementSize,
		cube.size.y * elementSize,
		cube.size.z * elementSize
	};
}

[[nodiscard]] bool doesCubeFit(
	const Config& config, const Grid& grid, const Cuboid& cube)
{
	K::Vector_3 realDims = cubeInReal(grid, cube);
	return fitsVolume(config, realDims.x(), realDims.y(), realDims.z());
}

[[nodiscard]] std::vector<const GridCell*> expandCube(
	const Config& config,
	Direction direction, 
	const Grid& grid,
	const mv::vector3<GridCell>& gridCells, 
	Cuboid& cube)
{
	// Direction indicates the side being grown!
	switch(direction)
	{
		case Direction::Left:
		{
			//std::cout << "Growing Left..." << std::endl;

			if(cube.origin.x == 0)
				return {};

			Vector3D btmLeftOut = cube.origin;
			--btmLeftOut.x;

			Vector3D topLeftIn =
				btmLeftOut + Vector3D(1, cube.size.y, cube.size.z);

			--cube.origin.x;
			++cube.size.x;

			if(!doesCubeFit(config, grid, cube))
				return {};

			return sampleCells(gridCells, btmLeftOut, topLeftIn);
		}
		break;                                                      
		case Direction::Right:                                          
		{
			//std::cout << "Growing Right..." << std::endl;

			const Vector3D btmRightOut = 
				cube.origin + Vector3D(cube.size.x, 0, 0);
			const Vector3D topRightIn = 
				btmRightOut + Vector3D(1, cube.size.y, cube.size.z);

			if(topRightIn.x >= grid.getNumBoxesX())
				return {};

			++cube.size.x;

			if(!doesCubeFit(config, grid, cube))
				return {};

			return sampleCells(gridCells, btmRightOut, topRightIn);
		}
		break;
		case Direction::Down:
		{
			//std::cout << "Growing Down..." << std::endl;

			if(cube.origin.y == 0)
				return {};

			Vector3D btmLeftOut = cube.origin;
			--btmLeftOut.y;

			Vector3D btmRightIn =
				btmLeftOut + Vector3D(cube.size.x, 1, cube.size.z);

			--cube.origin.y;
			++cube.size.y;

			if(!doesCubeFit(config, grid, cube))
				return {};

			return sampleCells(gridCells, btmLeftOut, btmRightIn);
		}
		break;
		case Direction::Up:
		{
			//std::cout << "Growing Up..." << std::endl;

			const Vector3D topLeftOut = 
				cube.origin + Vector3D(0, cube.size.y, 0);
			const Vector3D topRightIn = 
				topLeftOut + Vector3D(cube.size.x, 1, cube.size.z);

			if(topLeftOut.y >= grid.getNumBoxesY())
				return {};

			++cube.size.y;

			if(!doesCubeFit(config, grid, cube))
				return {};

			return sampleCells(gridCells, topLeftOut, topRightIn);
		}
		break;
		case Direction::Out:
		{
			//std::cout << "Growing Out..." << std::endl;

			if(cube.origin.z == 0)
				return {};

			Vector3D btmLeftOut = cube.origin;
			--btmLeftOut.z;

			Vector3D topRightOut =
				btmLeftOut + Vector3D(cube.size.x, cube.size.y, 1);

			--cube.origin.z;
			++cube.size.z;

			if(!doesCubeFit(config, grid, cube))
				return {};

			return sampleCells(gridCells, btmLeftOut, topRightOut);
		}
		break;
		case Direction::In:
		{
			//std::cout << "Growing In..." << std::endl;

			const Vector3D btmLeftIn = 
				cube.origin + Vector3D(0, 0, cube.size.z);
			const Vector3D topRightIn = 
				btmLeftIn + Vector3D(cube.size.x, cube.size.y, 1);

			if(topRightIn.z >= grid.getNumBoxesZ())
				return {};

			++cube.size.z;

			if(!doesCubeFit(config, grid, cube))
				return {};

			return sampleCells(gridCells, btmLeftIn, topRightIn);
		}
		break;                                                      
	}

	return {};
}

bool isValidEmptyRegion(
	const std::set<const GridCell*>& allEmptyCells,
	const std::vector<const GridCell*>& cells)
{
	if(cells.empty())
		return false;

	bool isOnlyInternals = true;

	for(const GridCell* cell: cells)
	{
		if(cell->type == GridCell::ContentType::Boundary)
			isOnlyInternals = false;

		if((cell->type == GridCell::ContentType::Boundary || 
			cell->type == GridCell::ContentType::Internal)
			&& !cell->parents.empty())
		{
			return false;
		}

		if(allEmptyCells.contains(cell))
			return false;
	}

	return !isOnlyInternals;
}

bool expandInDirection(
	Direction direction,
	const Config& config,
	const Grid& grid, 
	const mv::vector3<GridCell>& gridCells,
	Cuboid& cube,
	std::set<const GridCell*>& allEmptyCells,
	std::vector<const GridCell*>& newEmptyCells)
{
	std::vector<const GridCell*> newCells = 
		expandCube(config, direction, grid, gridCells, cube);

	if(!isValidEmptyRegion(allEmptyCells, newCells))
		return false;

	newEmptyCells.insert(newEmptyCells.end(), newCells.begin(), newCells.end());

	return true;
}


void updateEmptyCellLog(
	std::set<const GridCell*>& allEmptyCells, 
	const std::vector<const GridCell*>& newEmptyCells)
{
	for(const GridCell* cell: newEmptyCells)
		allEmptyCells.insert(cell);
}

bool getNeighbouringEmptyCells(
	const Config& config,
	const Vector3D& position,
	const Grid& grid, 
	const mv::vector3<GridCell>& gridCells,
	std::set<const GridCell*>& emptyCells,
	std::set<const GridCell*>& allEmptyCells)
{
	const GridCell& cell = mv::get(gridCells, position.x, position.y, position.z);
	if(cell.type != GridCell::ContentType::Boundary || !cell.parents.empty())
		return false;

	if(emptyCells.insert(std::addressof(cell)).second == false)
		return false;

	Cuboid cube(position, Vector3D(1, 1, 1));

	std::set<Direction> activeDirections = {
		Direction::Left, Direction::Right,
		Direction::Up, Direction::Down,
		Direction::Out, Direction::In
	};

	while(!activeDirections.empty())
	{
		std::vector<const GridCell*> newEmptyCells;

		std::vector<Direction> invalidDirections;

		for(Direction direction: activeDirections)
		{
			Cuboid backupCube(cube);

			bool valid = expandInDirection(
				direction,
				config,
				grid, gridCells, 
				cube, 
				allEmptyCells, newEmptyCells);
			if(!valid)
			{
				cube = backupCube;
				invalidDirections.push_back(direction);
			}
		}

		for(Direction direction: invalidDirections)
			activeDirections.erase(direction);

#ifdef VERBOSE
		std::cout << cube << std::endl;
#endif

		updateEmptyCellLog(emptyCells, newEmptyCells);
	}

	return true;
}

bool calcDiscreteRegions(
	const Config& config,
	const Grid& grid, 
	mv::vector3<GridCell>& gridCells, 
	std::vector<std::vector<const GridCell*>>& regions,
	int numAvailablePrinters)
{
	std::set<const GridCell*> allEmptyCells;

#ifdef VERBOSE
	std::cout << "Finding empty regions... " << std::flush;
#endif

	int limit = numAvailablePrinters > 0 
		? numAvailablePrinters : std::numeric_limits<int>::max();
	for(int itr = 0; itr < limit; ++itr)
	{
		std::set<const GridCell*> emptyCells;

		bool found = false;
		Vector3D seedPos;
		mv::forEachIndexed<GridCell>([&](const GridCell& cell, int x, int y, int z) {
			if(!found && 
			   cell.type == GridCell::ContentType::Boundary && 
			   cell.parents.empty())
			{
				if(!allEmptyCells.contains(std::addressof(cell)))
				{
					found = true;
					seedPos = Vector3D(x, y, z);
				}
			}
		}, gridCells);

		if(!found)
		{
#ifdef VERBOSE
			std::cout << "done." << std::endl;
#endif
			return true;
		}

		getNeighbouringEmptyCells(
			config, seedPos, grid, gridCells, emptyCells, allEmptyCells);
		for(const GridCell* cell: emptyCells)
			allEmptyCells.insert(cell);
		regions.emplace_back(std::vector<const GridCell*>(emptyCells.cbegin(), emptyCells.cend()));

#ifdef VERBOSE
		std::cout << itr+1 << ", " << std::flush;
#endif
	}

	bool success = true;
	mv::forEachIndexed<GridCell>([&](const GridCell& cell, int x, int y, int z) {
		if(cell.type == GridCell::ContentType::Boundary && cell.parents.empty())
		{
			if(!allEmptyCells.contains(std::addressof(cell)))
				success = false;
			
		}
	}, gridCells);

#ifdef VERBOSE
	if(!success)
		std::cout << "failed - too many!" << std::endl;
#endif

	return success;
}

bool getDiscreteEmptyRegions(
	const Config& config,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	int numAvailablePrinters,
	std::vector<Cuboid>& emptyRegions)
{
	std::vector<std::vector<const GridCell*>> regionCells;
	bool valid = calcDiscreteRegions(
		config, grid, gridCells, regionCells, numAvailablePrinters);

	for(const auto& region: regionCells)
		emptyRegions.emplace_back(getDimsFrom(region));

	return valid;
}

int getDiscreteEmptyRegions(
	const Config& config,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	std::vector<Cuboid>& emptyRegions)
{
	std::vector<std::vector<const GridCell*>> regionCells;
	bool valid = calcDiscreteRegions(config, grid, gridCells, regionCells, 0);

	for(const auto& region: regionCells)
		emptyRegions.emplace_back(getDimsFrom(region));

	return valid ? regionCells.size() : -1;
}*/


[[nodiscard]] double computeEmptyFillCost(
	const Config& config,
	const Mesh& parent,
	double currentPrintCost,
	double currentParallelCost,
	Direction direction, 
	const MeshBox& region,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache)
{
	// REMEMBER: Lower is better, negative not allowed
	
	// Grow and Test for Constraint Violations
	///////////////////////////////////////////////////////////////////////////
	
	std::vector<GridCell*> tempVec;
	std::optional<Cuboid> newRegion = 
		safeExpand(direction, region.dims, grid, gridCells, tempVec);
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
	double printCost = printingCost(config, grid, newMeshBox, printCostCache);
	if(printCost > currentParallelCost)
		return -1.0;
	double dPrintCost = printCost - currentPrintCost;

	// Application of standard objective
	///////////////////////////////////////////////////////////////////////////

	double score = dPrintCost;
#ifdef VERBOSE
	std::cout << "\t\tScore for " << toText(direction)
			  << " = " << dPrintCost  << std::endl;
#endif

	return score;
}

bool growBoxIfPossible(
	const Config& config,
	const Mesh& parent,
	MeshBox& targetBox,
	double currentPrintCost,
	double currentParallelCost,
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

	#pragma omp parallel for default(none) shared(directions, costs, config, parent, currentPrintCost, currentParallelCost, targetBox, grid, gridCells, printCostCache)
	for(Direction direction: directions)
	{
		costs[direction] = computeEmptyFillCost(
			config, parent,
			currentPrintCost,
			currentParallelCost,
			direction,
			targetBox, 
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

std::optional<Vector3D> findEmptyBoundarySpace(mv::vector3<GridCell>& gridCells)
{
	std::optional<Vector3D> seedPos = std::nullopt;
	mv::forEachIndexed<GridCell>([&](const GridCell& cell, int x, int y, int z) {
		if(!seedPos && cell.type == GridCell::ContentType::Boundary && cell.parents.empty())
			seedPos = Vector3D(x, y, z);
	}, gridCells);
	return seedPos;
}

std::unique_ptr<MeshBox> fillEmptyBoundarySpace(
	const Config& config,
	const Mesh& mesh,
	const Vector3D& from,
	double currentParallelCost,
	const Grid& grid, 
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache)
{
	GridCell& source = mv::get(gridCells, from.x, from.y, from.z);
	std::unique_ptr<MeshBox> meshBox = std::make_unique<MeshBox>((MeshBox){
		source.mesh,
		Cuboid(from, Vector3D(1, 1, 1)),
		{std::addressof(source)}});
	source.parents.clear();
	source.parents.push_back(meshBox.get());

	double currentPrintCost = printingCost(config, grid, *meshBox, printCostCache);
	while(growBoxIfPossible(
		config, 
		mesh, 
		*meshBox, 
		currentPrintCost, currentParallelCost,
		gridCells, grid, 
		printCostCache))
	{
		currentPrintCost = printingCost(config, grid, *meshBox, printCostCache);
	}

	return std::move(meshBox);
}

bool fillEmptyBoundarySpaces(
	const Config& config,
	const Mesh& mesh,
	std::vector<std::unique_ptr<MeshBox>>& outMeshBoxes,
	double currentParallelCost,
	const Grid& grid, 
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache)
{
	//std::cout << "Filling empty regions... " << std::endl;

	std::optional<Vector3D> found = findEmptyBoundarySpace(gridCells);
	while(found)
	{
		outMeshBoxes.emplace_back(fillEmptyBoundarySpace(
			config, mesh, 
			*found, 
			currentParallelCost, 
			grid, gridCells, 
			printCostCache));
		found = findEmptyBoundarySpace(gridCells);
	}

	return true;
}
