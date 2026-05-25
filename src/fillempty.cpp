#include "fillempty.h"
#include "regiongrowth.h"

struct CostData
{
	std::unique_ptr<MeshBox> newRegion;
	std::vector<GridCell*> newCells;
};

[[nodiscard]] double computeEmptyFillCost(
	const Config& config,
	const Mesh& parent,
	double currentPrintCost,
	double currentParallelCost,
	Direction direction, 
	const MeshBox& region,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache,
	CostData& returnData)
{
	// REMEMBER: Lower is better, negative not allowed
	
	// Grow and Test for Constraint Violations
	///////////////////////////////////////////////////////////////////////////
	
	std::optional<Cuboid> newRegion = 
		safeExpand(direction, region.dims, grid, gridCells, returnData.newCells);
	if(!newRegion)
		return -1.0;

	// Compute new MeshBox
	///////////////////////////////////////////////////////////////////////////
	
	std::unique_ptr<MeshBox> newMeshBox = std::make_unique<MeshBox>(*newRegion);

	if(INVALID(clipFromMesh(grid, parent, *newMeshBox)))
		return -1.0;
	
	// Printability Constraint
	///////////////////////////////////////////////////////////////////////////
	
	if(!fitsVolume(config, newMeshBox->mesh))
	{
#ifdef VERBOSE
		std::cout << "\tGrowth " << toText(direction) << " does not fit volume" << std::endl;
#endif
		return -1.0;
	}

	// Standard Objective
	///////////////////////////////////////////////////////////////////////////
	// This accounts for core printing cost + minimal support structure print costs
	double printCost = printingCost(config, grid, *newMeshBox, printCostCache);
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

	returnData.newRegion = std::move(newMeshBox);

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
	std::unordered_map<Direction,CostData> costData;
	for(Direction direction: directions)
	{
		costs[direction] = -1.0;
		costData[direction] = (CostData) { nullptr, {} };
	}

	#pragma omp parallel for default(none) shared(directions, costs, costData, config, parent, currentPrintCost, currentParallelCost, targetBox, grid, gridCells, printCostCache)
	for(Direction direction: directions)
	{
		costs[direction] = computeEmptyFillCost(
			config, parent,
			currentPrintCost,
			currentParallelCost,
			direction,
			targetBox, 
			grid, gridCells,
			printCostCache,
			costData[direction]);
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

	MeshBox& newMeshBox = *costData[*bestDirection].newRegion;
	targetBox.mesh = newMeshBox.mesh;
	targetBox.dims = newMeshBox.dims;
	addCellsToMeshBox(targetBox, costData[*bestDirection].newCells);
	//grow(*bestDirection, parent, grid, gridCells, targetBox);

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
