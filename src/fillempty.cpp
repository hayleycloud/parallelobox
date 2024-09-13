#include "fillempty.h"


std::vector<const GridCell*> sampleCells(
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

[[nodiscard]] std::vector<const GridCell*> expandCube(
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
	const Grid& grid, 
	const mv::vector3<GridCell>& gridCells,
	Cuboid& cube,
	std::set<const GridCell*>& allEmptyCells,
	std::vector<const GridCell*>& newEmptyCells)
{
	std::vector<const GridCell*> newCells = 
		expandCube(direction, grid, gridCells, cube);

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
	const Grid& grid, 
	mv::vector3<GridCell>& gridCells, 
	int numAvailablePrinters,
	std::vector<std::vector<const GridCell*>>& regions)
{
	std::set<const GridCell*> allEmptyCells;

#ifdef VERBOSE
	std::cout << "Finding empty regions... " << std::flush;
#endif

	for(int itr = 0; itr < numAvailablePrinters; ++itr)
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
			seedPos, grid, gridCells, emptyCells, allEmptyCells);
		for(const GridCell* cell: emptyCells)
			allEmptyCells.insert(cell);
		regions.emplace_back(std::vector<const GridCell*>(emptyCells.cbegin(), emptyCells.cend()));

#ifdef VERBOSE
		std::cout << itr+1 << ", " << std::flush;
#endif
	}

#ifdef VERBOSE
	std::cout << "failed - too many!" << std::endl;
#endif

	return false;
}

bool getDiscreteEmptyRegions(
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	int numAvailablePrinters,
	std::vector<Cuboid>& emptyRegions)
{

	std::vector<std::vector<const GridCell*>> regionCells;
	if(!calcDiscreteRegions(grid, gridCells, numAvailablePrinters, regionCells))
		return false;

	for(const auto& region: regionCells)
		emptyRegions.emplace_back(getDimsFrom(region));

	return true;
}

