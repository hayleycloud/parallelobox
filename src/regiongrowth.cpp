#include "regiongrowth.h"
#include "objective.h"
#include "multivec.h"
#include <unordered_map>
#include <optional>

[[nodiscard]]
bool continueRegionGrowth(
	std::vector<MeshBox*>& sourceBoxes, mv::vector3<GridCell>& gridCells)
{
	// If we can find a boundary box with no parent, we still have expansion to do
	const auto* pendingMeshCell = mv::find<GridCell>([](const GridCell& cell) {
		if(cell.type == GridCell::ContentType::Boundary)
			return cell.parents.size() > 0;

		return false;
	}, gridCells);

	return pendingMeshCell;
}

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
					cell->parents.cbegin(), cell->parents.cend(), boxPtr));

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

void regionGrowth(
	const Mesh& parent,
	std::vector<MeshBox*>& sourceBoxes, 
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
		for(MeshBox* sourceBox: sourceBoxes)
		{
			std::unordered_map<Direction,double> scores;

			Direction bestDirection = getBestDirection(scores);
			#pragma omp critical
			expandDirections.push_back((MeshBoxGrow){sourceBox, bestDirection});
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
		for(MeshBox* sourceBox: sourceBoxes)
			clipFromMesh(grid, parent, *sourceBox);
	}
}
