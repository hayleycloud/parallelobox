#include "resolve.h"

int enumerateConflicts(const mv::vector3<GridCell>& gridCells)
{
	size_t conflicts = 0;
	mv::reduce<const GridCell,size_t>([](size_t& acc, const GridCell& cell) {
		if(cell.parents.size() > 1)
			++conflicts;
	}, gridCells, conflicts);

	return conflicts;
}

// Input -> Mesh boxes and empty regions
// Output -> Mesh boxes

struct EmptyFillOp
{
	int boxIndex;
	Direction direction;
	int quantity;
}

Direction inverse(Direction direction)
{
	switch(direction)
	{
		case Direction::Right:
			return Direction::Left;
		case Direction::Left:
			return Direction::Right;
		case Direction::Up:
			return Direction::Down;
		case Direction::Down:
			return Direction::Up;
		case Direction::In:
			return Direction::Out;
		case Direction::Out:
			return Direction::In;
	}

	return Direction::Right;
}

bool isOverlapping(const Cuboid& cube1, const Cuboid& cube2)
{
	const Cuboid& cube1 = regionExpanded;
	const Cuboid& cube2 = meshbox.dims;

	Vector3D cube1Max(
		cube1.origin.x + cube1.size.x,
		cube1.origin.y + cube1.size.y,
		cube1.origin.z + cube1.size.z
	);
	Vector3D cube2Max(
		cube2.origin.x + cube2.size.x,
		cube2.origin.y + cube2.size.y,
		cube2.origin.z + cube2.size.z
	);

	Vector3D overlapOrigin(
		std::max(cube1.origin.x, cube2.origin.x),
		std::max(cube1.origin.y, cube2.origin.y),
		std::max(cube1.origin.z, cube2.origin.z));
	Vector3D overlapSize(
		std::min(cube1Max.x, cube2Max.x) - overlapOrigin.x,
		std::min(cube1Max.y, cube2Max.y) - overlapOrigin.y,
		std::min(cube1Max.z, cube2Max.z) - overlapOrigin.z);

	return overlapSize.x > 0 && overlapSize.y > 0 && overlapSize.z > 0;
}

void extractFillOps(
	const Cuboid& region, 
	Direction direction, 
	const std::vector<Cuboid>& boxDims,
	std::vector<EmptyFillOp>& ops)
{
	Cuboid regionExpanded = Cuboid(region);
	switch(direction)
	{
		case Direction::Right:
			regionExpanded.x += 1;
			break;
		case Direction::Left:
			regionExpanded.x -= 1;
			break;
		case Direction::Up:
			regionExpanded.y += 1;
			break;
		case Direction::Down:
			regionExpanded.y -= 1;
			break;
		case Direction::In:
			regionExpanded.z += 1;
			break;
		case Direction::Out:
			regionExpanded.z -= 1;
			break;
	}

	for(int i = 0; i < boxDims.size(); ++i)
	{
		const Cuboid& dim = boxDims.at(i);

		if(isOverlapping(dim, regionExpanded)
		{
			Direction invDir;
			size_t amount;
			switch(direction)
			{
				case Direction::Right:
					invDir = Direction::Left;
					amount = region.size.x;
					break;
				case Direction::Left:
					invDir = Direction::Right;
					amount = region.size.x;
					break;
				case Direction::Up:
					invDir = Direction::Down;
					amount = region.size.y;
					break;
				case Direction::Down:
					invDir = Direction::Up;
					amount = region.size.y;
					break;
				case Direction::In:
					invDir = Direction::Out;
					amount = region.size.z;
					break;
				case Direction::Out:
					invDir = Direction::In;
					amount = region.size.z;
					break;
			}

			ops.emplace_back((EmptyFillOp) { i, invDir, amount });
		}
	}
}

void applyExpansion(
	Direction direction, 
	const std::vector<EmptyFillOp>& ops,
	std::vector<Cuboid>& boxDims)
{
	for(EmptyFillOp& op: ops)
	{
		Cuboid& dim = boxDims.at(op.boxIndex);

		switch(direction)
		{
			case Direction::Right:
				dim.origin.x -= op.quantity;
			case Direction::Left:
				dim.size.x += op.quantity;
				break;
			case Direction::Up:
				dim.origin.y -= op.quantity;
			case Direction::Down:
				dim.size.y += op.quantity;
				break;
			case Direction::In:
				dim.origin.z -= op.quantity;
			case Direction::Out:
				dim.size.z += op.quantity;
				break;
		}
	}
}

void resolveGaps(
	const Mesh& parent,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes,
	std::vector<Cuboid>& emptyRegions,
	mv::vector3<GridCell>& gridCells,
	Grid& grid)
{
	// For each empty
	// Enlarge empty region cuboid horizontally and vertically
	// (Independently! We don't want the corners! So three iterations here)
	// (Or six - and enumerate the growths)
	// Test for source boxes that overlap
	// Grow into the void
	// Enumerate conflicts
	// Attempt fixes
	
	std::vector<Cuboid> boxRegions;
	for(auto& box: sourceBoxes)
		boxRegions.push_back(box.dims);

	std::unordered_map<Direction,std::vector<Cuboid>> grownRegions;
	grownRegions[Direction::Right] = boxRegions;
	grownRegions[Direction::Left] = boxRegions;
	grownRegions[Direction::Up] = boxRegions;
	grownRegions[Direction::Down] = boxRegions;
	grownRegions[Direction::In] = boxRegions;
	grownRegions[Direction::Out] = boxRegions;

	extractFillOps();

	applyExpansion(Direction::Right, ops, grownRegions[Direction::Right]);
	applyExpansion(Direction::Left, ops, grownRegions[Direction::Left]);
	applyExpansion(Direction::Up, ops, grownRegions[Direction::Up]);
	applyExpansion(Direction::Down, ops, grownRegions[Direction::Down]);
	applyExpansion(Direction::In, ops, grownRegions[Direction::In]);
	applyExpansion(Direction::Out, ops, grownRegions[Direction::Out]);

	// Rank these regions, eliminating any conflicts (for now!)
}
