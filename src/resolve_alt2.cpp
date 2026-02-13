#include "resolve.h"

void forEachDirection(std::function<void(Direction)> func)
{
	func(Direction::Right);
	func(Direction::Left);
	func(Direction::Up);
	func(Direction::Down);
	func(Direction::In);
	func(Direction::Out);
}

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

void getOverlapOf(const Cuboid& cube1, const Cuboid& cube2, Cuboid& overlap)
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

	overlap = Cuboid(overlapOrigin, overlapSize);
}

int volumeOf(const Cuboid& cube)
{
	return cube.size.x * cube.size.y * cube.size.z;
}

bool hasVolume(const Cuboid& cube)
{
	return volumeOf(cube) > 0;
}

bool isOverlapping(const Cuboid& cube1, const Cuboid& cube2)
{
	Cuboid overlap;
	getOverlapOf(cube1, cube2, overlap);

	return hasVolume(overlap);
}

bool extractFillOps(
	const Cuboid& region, 
	Direction direction, 
	const std::vector<Cuboid>& boxDims,
	std::vector<EmptyFillOp>& ops)
{
	Cuboid regionExpanded = Cuboid(region);
	switch(direction)
	{
		case Direction::Left:
			regionExpanded.origin.x -= 1;
			regionExpanded.size.x = 1;
			break;
		case Direction::Right:
			regionExpanded.origin.x += regionExpanded.size.x;
			regionExpanded.size.x = 1;
			break;
		case Direction::Down:
			regionExpanded.origin.y -= 1;
			regionExpanded.size.y = 1;
			break;
		case Direction::Up:
			regionExpanded.origin.y += regionExpanded.size.y;
			regionExpanded.size.y = 1;
			break;
		case Direction::Out:
			regionExpanded.origin.z -= 1;
			regionExpanded.size.z = 1;
			break;
		case Direction::In:
			regionExpanded.origin.z += regionExpanded.size.z;
			regionExpanded.size.z = 1;
			break;
	}

	int volumeRemaining = volumeOf(regionExpanded);

	for(int i = 0; i < boxDims.size(); ++i)
	{
		const Cuboid& dim = boxDims.at(i);

		Cuboid overlapVolume;
		getOverlapOf(dim, regionExpanded, overlapVolume);

		int overlapVolumeSize = volumeOf(overlapVolume);

		// Test overlapping
		if(overlapVolumeSize > 0)
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

			volumeRemaining -= overlapVolumeSize;
		}
	}

	return volumeRemaining == 0;
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

struct Conflict
{
	int expandedIndex, conflictedIndex;
	Cuboid region;
};

void enumerateConflicts(
	const std::vector<EmptyFillOp>& ops,
	const std::vector<Cuboid>& regions,
	std::vector<Conflict>& conflicts)
{
	std::vector<int> expandedIndices;
	for(const EmptyFillOp& op: ops)
		expandedIndices.push_back(op.boxIndex);

	std::vector<int> otherIndices;
	for(int i = 0; i < regions.size(); ++i)
	{
		if(std::find(expandedIndices.begin(), expandedIndices.end(), i)
			== std::end(expandedIndices))
		{
			otherIndices.push_back(i);
		}
	}

	for(int i: expandedIndices)
	{
		for(int j: otherIndices)
		{
			Cuboid conflictVolume;
			getOverlapOf(regions.at(i), regions.at(j), conflictVolume);

			if(hasVolume(conflictVolume))
			{
				conflicts.emplace_back((Conflict) { i, j, conflictVolume });
			}
		}
	}
}

struct ResolutionOp
{
	Direction direction;
	int quantity;
};

void getResolutionDirections(
	const Conflict& conflict,
	const std::vector<Cuboid>& boxes,
	std::vector<ResolutionOp>& directions)
{
	const Cuboid& box = boxes.at(conflict.conflictedIndex);

	const Vector3D confRegionEnd = conflict.region.last();
	const Vector3D boxEnd = box.region.last();

	if(conflict.region.origin.x == box.origin.x)
		directions.emplace_back((ResolutionOp) {
			Direction::Right, conflict.region.size.x });

	if(confRegionEnd.x == boxEnd.x)
		directions.emplace_back((ResolutionOp) {
			Direction::Left, conflict.region.size.x });

	if(conflict.region.origin.y == box.origin.y)
		directions.emplace_back((ResolutionOp) {
			Direction::Up, conflict.region.size.y });

	if(confRegionEnd.y == boxEnd.y)
		directions.emplace_back((ResolutionOp) {
			Direction::Down, conflict.region.size.y });

	if(conflict.region.origin.z == box.origin.z)
		directions.emplace_back((ResolutionOp) {
			Direction::In, conflict.region.size.z });

	if(confRegionEnd.z == boxEnd.z)
		directions.emplace_back((ResolutionOp) {
			Direction::Out, conflict.region.size.z });
}

// Enumerate any extra space orphaned by the shrink

void getShrinkRegion(
	const Cuboid& targetBox,
	const ResolutionOp& op,
	Cuboid& region)
{
	switch(op.direction)
	{
		case Direction::Right:
		{
			region.origin.x = targetBox.origin.x;
			region.size.x = op.quantity;
			return;
		}
		case Direction::Left:
		{
			int remX = targetBox.size.x - op.quantity;
			region.origin.x = targetBox.origin.x + remX;
			region.size.x = op.quantity;
			return;
		}
		case Direction::Up:
		{
			region.origin.y = targetBox.origin.y;
			region.size.y = op.quantity;
			return;
		}
		case Direction::Down:
		{
			int remY = targetBox.size.y - op.quantity;
			region.origin.y = targetBox.origin.y + remY;
			region.size.y = op.quantity;
			return;
		}
		case Direction::In:
		{
			region.origin.z = targetBox.origin.z;
			region.size.z = op.quantity;
			return;
		}
		case Direction::Out:
		{
			int remZ = targetBox.size.z - op.quantity;
			region.origin.z = targetBox.origin.z + remZ;
			region.size.z = op.quantity;
			return;
		}
	}
}

bool isValidResolution(
	const std::vector<Cuboid>& grownRegions,
	const Cuboid& targetBox, 
	const ResolutionOp& op)
{
	Cuboid shrinkRegion;
	getShrinkRegion(targetBox, op, shrinkRegion);

	// Intersect shrink region with grown regions
	// Then determine if meta-shrink regions add into shrink region with no redundancy
	for(const Cuboid& grownRegion: grownRegions)
	{

	}
}

void attemptResolveConflicts(
	const std::vector<Conflict>& conflicts,
	std::vector<Cuboid>& regions)
{
	for(const Conflict& conflict: conflicts)
	{
		const Cuboid& grownCube = regions.at(conflict.expandedIndex);
		Cuboid& collidedCube = regions[conflict.conflictedIndex];

		
	}
}

void resolveGap(
	const Mesh& parent,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes,
	const Cuboid& emptyRegion,
	mv::vector3<GridCell>& gridCells,
	Grid& grid)
{
	std::vector<Cuboid> boxRegions;
	for(auto& box: sourceBoxes)
		boxRegions.push_back(box.dims);

	std::unordered_map<Direction,std::vector<Cuboid>> grownRegions;
	forEachDirection([=] (Direction direction) {
		grownRegions[direction] = boxRegions;
	});

	std::unordered_map<Direction,bool> validExpansion;
	std::unordered_map<Direction,std::vector<EmptyFillOp>> ops;
	forEachDirection([=] (Direction direction) {
		validExpansion[direction] = extractFillOps(
			emptyRegion, direction, boxRegions, ops[direction]);
	});

	forEachDirection([=] (Direction direction) {
		if(validExpansion[direction])
			applyExpansion(direction, ops[direction], grownRegions[direction]);
	});

	std::unordered_map<Direction,std::vector<Conflict>> conflicts;
	forEachDirection([=] (Direction direction) {
		if(validExpansion[direction])
			enumerateConflicts(
				ops[direction], grownRegions[direction], conflicts[direction]);
	});

	// Rank these regions, eliminating any conflicts (for now!)
	std::unordered_map<Direction,double> scores;
	for(
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
	
	for(const Cuboid& emptyRegion: emptyRegions)
	{
		resolveGap(parent, sourceBoxes, emptyRegion, gridCells, grid);
	}

}
 