#include "blockmerge.h"
#include "objective.h"
#include "multivec.h"
#include <omp.h>

typedef std::vector<MeshBox*> MeshBoxPtrList;
typedef std::vector<std::pair<MeshBox*,double>> MeshBoxPtrCostList;


enum class Direction {
	Left, Right, Up, Down, In, Out
};

Direction invert(Direction direction)
{
	switch(direction)
	{
		case Direction::Left:
			return Direction::Right;
		case Direction::Right:
			return Direction::Left;
		case Direction::Up:
			return Direction::Down;
		case Direction::Down:
			return Direction::Up;
		case Direction::In:
			return Direction::Out;
		case Direction::Out:
			return Direction::In;
	}
	return Direction::Out;
}

struct GridPathed
{
	mv::vector3<GridCell> cells;
	unsigned int sideIndex;
};

bool getAdjacentBoxIfExists(
	GridPathed& gridCells, 
	int x, int y, int z,
	std::unordered_set<MeshBox*>& meshboxes)
{
	if(x < 0 || y < 0 || z < 0 || 
		x >= gridCells.cells[0][0].size() ||
		y >= gridCells.cells[0].size() ||
		z >= gridCells.cells.size())
	{
		return false;
	}

	GridCell& cell = mv::get(gridCells.cells, x, y, z);
	if(cell.type == GridCell::ContentType::Boundary)
	{
		assert(cell.parent);
		std::cout << "If Exists: " << cell.sideParents[gridCells.sideIndex] << std::endl;
		meshboxes.insert(cell.sideParents[gridCells.sideIndex]);
		return true;
	}
	else
		return false;
}

void getAdjacentBox(
	GridPathed& gridCells, 
	int x, int y, int z,
	Direction direction,
	std::unordered_set<MeshBox*>& adjacentBoxes)
{
	switch(direction)
	{
		case Direction::Left:
			getAdjacentBoxIfExists(gridCells, x - 1, y, z, adjacentBoxes);
			break;                                                      
		case Direction::Right:                                          
			getAdjacentBoxIfExists(gridCells, x + 1, y, z, adjacentBoxes);
			break;                                                      
		case Direction::Up:                                             
			getAdjacentBoxIfExists(gridCells, x, y + 1, z, adjacentBoxes);
			break;                                                      
		case Direction::Down:                                           
			getAdjacentBoxIfExists(gridCells, x, y - 1, z, adjacentBoxes);
			break;                                                      
		case Direction::In:                                             
			getAdjacentBoxIfExists(gridCells, x, y, z + 1, adjacentBoxes);
			break;                                                      
		case Direction::Out:                                            
			getAdjacentBoxIfExists(gridCells, x, y, z - 1, adjacentBoxes);
			break;
	}
}

std::unordered_set<MeshBox*> getAdjacentBoxes(
	GridPathed& gridCells,
	MeshBox& box,
	Direction direction)
{
	std::unordered_set<MeshBox*> adjacentBoxes;

	for(GridCell* child: box.children)
	{
		assert(child);
		const Vector3D& position = child->position;
		getAdjacentBox(
			gridCells, 
			position.x, position.y, position.z, 
			direction, 
			adjacentBoxes);
	}

	adjacentBoxes.erase(std::addressof(box));
	
	return adjacentBoxes;
}

struct AdjacencyBranch
{
	MeshBox* parent;
	MeshBox* child;
	Direction direction;

	enum class Type { Expand, Shrink } type;
};

Direction determineOptimumShrinkDirection(
	//MeshBox& predator, MeshBox& prey,
	Direction expandDirection)
{
	// TODO: Optimisation logic
	
	return invert(expandDirection);
}

AdjacencyBranch::Type toggleType(AdjacencyBranch::Type type)
{
	if(type == AdjacencyBranch::Type::Expand)
		return AdjacencyBranch::Type::Shrink;
	//else if(type == AdjacencyBranch::Type::Shrink)
	return AdjacencyBranch::Type::Expand;
}

bool nodeAlreadyVisited(
	MeshBox* node,
	const std::vector<AdjacencyBranch>& adjacencyBranches)
{
	for(const AdjacencyBranch& branch: adjacencyBranches)
	{
		if(branch.parent == node)
			return true;
	}
	return false;
}

void exploreBranches(
	GridPathed& gridCells,
	std::vector<AdjacencyBranch>& adjacencyBranches,
	MeshBox& box, 
	Direction direction,
	AdjacencyBranch::Type type = AdjacencyBranch::Type::Expand)
{
	std::cout << "Exploring branch..." << std::endl;

	AdjacencyBranch::Type nextType = toggleType(type);

	std::vector<MeshBox*> removals;
	auto adjBoxes = getAdjacentBoxes(gridCells, box, direction);
	for(MeshBox* adjBox: adjBoxes)
	{
		if(nodeAlreadyVisited(adjBox, adjacencyBranches)) {
			std::cout << "Node already visited." << std::endl;
			removals.push_back(adjBox);
		}
	}

	for(MeshBox* removal: removals)
		adjBoxes.erase(removal);

	if(adjBoxes.size() == 0)
	{
		std::cout << "No adjacent boxes." << std::endl;
		adjacencyBranches.emplace_back((AdjacencyBranch){
			std::addressof(box), nullptr, 
			determineOptimumShrinkDirection(direction),
			nextType
		});
		return;
	}

	std::vector<Direction> directions;
	for(MeshBox* adjBox: adjBoxes)
	{
		std::cout << "Adjacent Box found" << std::endl;
		adjacencyBranches.emplace_back((AdjacencyBranch){
			std::addressof(box), adjBox, 
			determineOptimumShrinkDirection(direction),
			nextType
		});
	}

	for(MeshBox* adjBox: adjBoxes)
	{
		std::cout << "AdjBox = " << adjBox << std::endl;
		exploreBranches(
			gridCells, adjacencyBranches, *adjBox, direction, nextType);
	}
}

void removeMeshBoxCells(
	int sideIndex, 
	MeshBox& box, 
	std::function<bool(const Cuboid&,const Vector3D&)> test)
{
//	std::vector<GridCell*> removal;
	for(GridCell* cell: box.children)
	{
		if(test(box.dims, cell->position))
		{
			//removal.push_back(cell);
			box.sideChanged[sideIndex] = true;
			cell->sideParents[sideIndex] = nullptr;
		}
	}

//	for(auto cell: removal)
//		box.children.remove(cell);
}

bool shrink(GridPathed& gridCells, MeshBox& box, Direction direction)
{
	int sideIndex = gridCells.sideIndex;

	// Direction indicates the side being shrunk!
	switch(direction)
	{
		case Direction::Left:
		{
			std::cout << "Shrinking Left..." << std::endl;

			removeMeshBoxCells(sideIndex, box, 
				[&](const Cuboid& a, const Vector3D& pos) {
					return a.origin.x == pos.x;
			});

			--box.dims.size.x;
			if(box.dims.size.x == 0)
				return false;

			++box.dims.origin.x;
		}
		break;                                                      
		case Direction::Right:                                          
		{
			std::cout << "Shrinking Right..." << std::endl;

			removeMeshBoxCells(sideIndex, box, 
				[&](const Cuboid& a, const Vector3D& pos) {
					return (a.origin.x + (a.size.x - 1)) == pos.x;
			});

			--box.dims.size.x;
			if(box.dims.size.x == 0)
				return false;
		}
		break;                                                 
		case Direction::Up:                                             
		{
			std::cout << "Shrinking Up..." << std::endl;

			removeMeshBoxCells(sideIndex, box, 
				[&](const Cuboid& a, const Vector3D& pos) {
					return (a.origin.y + (a.size.y - 1)) == pos.y;
			});

			--box.dims.size.y;
			if(box.dims.size.y == 0)
				return false;
		}
		break;                                                      
		case Direction::Down:                                           
		{
			std::cout << "Shrinking Down..." << std::endl;

			removeMeshBoxCells(sideIndex, box, 
				[&](const Cuboid& a, const Vector3D& pos) {
					return a.origin.y == pos.y;
			});

			--box.dims.size.y;
			if(box.dims.size.y == 0)
				return false;

			++box.dims.origin.y;
		}
		break;                                                      
		case Direction::In:                                             
		{
			std::cout << "Shrinking In..." << std::endl;

			removeMeshBoxCells(sideIndex, box, 
				[&](const Cuboid& a, const Vector3D& pos) {
					return (a.origin.z + (a.size.z - 1)) == pos.z;
			});

			--box.dims.size.z;
			if(box.dims.size.z == 0)
				return false;
		}
		break;                                                      
		case Direction::Out:                                            
		{
			std::cout << "Shrinking Out..." << std::endl;

			removeMeshBoxCells(sideIndex, box, 
				[&](const Cuboid& a, const Vector3D& pos) {
					return a.origin.z == pos.z;
			});

			--box.dims.size.z;
			if(box.dims.size.z == 0)
				return false;

			++box.dims.origin.z;
		}
		break;
	}

	return true;
}

void addCellsToMeshBox(
	GridPathed& gridCells, 
	MeshBox& box,
	const Vector3D& a, const Vector3D& b) 
{
	box.sideChanged[gridCells.sideIndex] = true;

    for(int x = a.x; x < b.x; x++) 
	{
        for(int y = a.y; y < b.y; y++) 
		{
            for(int z = a.z; z < b.z; z++) 
			{
				GridCell* cell = std::addressof(mv::get(gridCells.cells, x, y, z));
				assert(cell->sideParents[gridCells.sideIndex] == nullptr);

				cell->sideParents[gridCells.sideIndex] = std::addressof(box);
                //box.children.push_back(cell);
            }
        }
    }
}

void grow(GridPathed& gridCells, MeshBox& box, Direction direction)
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

void applyBranchShrinks(
	GridPathed& gridCells, 
	const std::vector<AdjacencyBranch>& branches)
{
	for(const AdjacencyBranch& branch: branches)
	{
		if(branch.type == AdjacencyBranch::Type::Shrink)
			shrink(gridCells, *branch.parent, branch.direction);
	}
}

void applyBranchGrows(
	GridPathed& gridCells, 
	const std::vector<AdjacencyBranch>& branches)
{
	for(const AdjacencyBranch& branch: branches)
	{
		if(branch.type == AdjacencyBranch::Type::Expand)
			grow(gridCells, *branch.parent, branch.direction);
	}
}

void feed(
	GridPathed& gridCells,
	MeshBox& prey,
	Direction direction)
{
	std::vector<AdjacencyBranch> adjacencyBranches;
	exploreBranches(gridCells, adjacencyBranches, prey, direction);

	applyBranchShrinks(gridCells, adjacencyBranches);
	applyBranchGrows(gridCells, adjacencyBranches);

	printParents(gridCells.cells, gridCells.sideIndex);
}

bool shouldIterateAgain(
	const Config& config, const std::list<MeshBox>& meshBoxes)
{
	// Hard require that the models can be printed in the available printers
	if(meshBoxes.size() > config.numPrinters)
		return true;

	// We can iterate to combine models further if the cost disparity is high
	// We should stop if it becomes evident that the algorithm has stagnated
	
	return false;
}

// The Dialogue
// Okay, right, wtf are we doing here
// Okay, the gist is, we feed the smallest box to the neighbours in the best way
// 
// Get the adjacent neighbour of each box of the prey in the given direction?
// But this means that *all* of those adjacent that way need expanding into it
// And expanding those might cause shrinkage requirements on other models!
// ERRRRR
// Okay, OOPS, this needs thinking through
//
// Okay, so, yes, let's try this:
// - Copy of mesh state per initial feed direction?
// - Apply sequences of feeding and expansion until fully resolved
// - Compute costs for each initial feed direction
// - Select and forward best one
// - Grid cells might need 6 extra parent ptrs for each direction
// - - Set from current parent, change as needed

struct SideScore
{
	unsigned int index;
	double score;
};

SideScore bestSideScore(
	const std::unordered_map<unsigned int,double>& sideIndexScores)
{
	unsigned int bestScoreIndex;
	double bestScore = std::numeric_limits<double>::max();

	for(unsigned int sideIndex = 0; sideIndex < NUM_SIDES; ++sideIndex)
	{
		int score = sideIndexScores.at(sideIndex);
		std::cout << "Index " << sideIndex << ": " << score << std::endl;
		if(score < bestScore)
		{
			bestScoreIndex = sideIndex;
			bestScore = score;
		}
	}

	return (SideScore){ bestScoreIndex, bestScore };
}



/**
 * TODO: Recompute mesh box list after iteration for next iteration.
 */
void mergeIterate(
	const Config& config, 
	const Mesh& parent,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells, 
	std::list<MeshBox>& meshBoxes)
{
	printParents(gridCells);

	int iteration = 1;
	while(shouldIterateAgain(config, meshBoxes))
	{
		std::cout << "Iteration " << iteration << std::endl;

		MeshBoxPtrCostList meshboxCosts; 
		for(auto itr = meshBoxes.begin(); itr != meshBoxes.end(); ++itr)
		{
			auto& meshBox = *itr;
			if(!meshBox.mesh.is_valid())
				std::cerr << "Mesh is not valid!" << std::endl;

			meshboxCosts.push_back(std::make_pair(
				std::addressof(meshBox), fitness(config, meshBox.mesh)));
		}

		// Sort the meshboxes such that highest cost is first
		// TODO: Lowest cost ya dummy
		std::sort(meshboxCosts.begin(), meshboxCosts.end(), [](auto& a, auto& b) {
			return a.second < b.second;
		});

		//for(const auto& box: meshboxCosts)
		//	std::cout << box.second << std::endl;

		MeshBox* prey = meshboxCosts.front().first;
		std::cout << "Target: " << prey << std::endl;

		// TODO: Recalculate only altered mesh boxes?
		std::array<std::vector<MeshBox>,NUM_SIDES> sideInstances;
		std::array<Direction,NUM_SIDES> sideDirections = {
			Direction::Left, Direction::Right,
			Direction::Up, Direction::Down,
			Direction::In, Direction::Out
		};

		std::unordered_map<unsigned int,double> sidePathScores;

		//#pragma omp parallel for
		for(unsigned int sideIndex = 0; sideIndex < NUM_SIDES; ++sideIndex)
		{
			std::cout << "Processing sideIndex = " << sideIndex << std::endl;

			GridPathed gridPath = {gridCells, sideIndex};
			feed(gridPath, *prey, sideDirections[sideIndex]);

			extractUniqueMeshBoxes(
				grid, gridCells, parent, sideInstances[sideIndex], sideIndex);

			// TODO: Um....? What about the side instances? We need to keep track
			// of the best cost path available!

			MeshBoxPtrCostList mbSideCosts; 
			for(MeshBox& meshBox: sideInstances[sideIndex])
			{
				//std::cout << "side instance" << std::endl;

				if(meshBox.mesh.is_empty())
					std::cerr << "Mesh is not valid!" << std::endl;

				//std::cout << "meshBox: " << meshBox << std::endl;
				auto meshBoxCost = std::make_pair(
					std::addressof(meshBox), fitness(config, meshBox.mesh));
				if(meshBoxCost.second > 0.0)
					mbSideCosts.push_back(meshBoxCost);
				//std::cout << "fitness" << std::endl;
			}

			// Sort the meshboxes such that highest cost is first
			// TODO: Lowest cost ya dummy
			std::sort(mbSideCosts.begin(), mbSideCosts.end(), [](auto& a, auto& b) {
				return a.second < b.second;
			});

			//#pragma omp critical
			sidePathScores[sideIndex] = mbSideCosts.front().second;
		}

		SideScore best = bestSideScore(sidePathScores);
		assignParents(gridCells, best.index);

		std::cout << "Best path: " << best.index << " (" << best.score << ")" << std::endl;

		printParents(gridCells);

		meshBoxes.clear();
		std::vector<MeshBox>& bestInst = sideInstances[best.index];
		std::copy(bestInst.begin(), bestInst.end(), std::back_inserter(meshBoxes));

		//for(MeshBox& box: meshBoxes) 
		//{
		//	clearMeshBoxChildren(box);
		//}
		//std::cout << "hi1" << std::endl;
		//assignParents(gridCells, best.index);
		//std::cout << "hi2" << std::endl;
		//meshBoxes.remove_if([](const MeshBox& box) {
		//	return box.children.empty();
		//});
		//std::cout << "hi3" << std::endl;

		//for(MeshBox& box: meshBoxes)
		//{
		//	if(box.sideChanged[best.index])
		//		clipFromMesh(grid, parent, box);
		//}

		int x;
		std::cin >> x;
		++iteration;
	}
}
