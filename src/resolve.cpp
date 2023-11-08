#include "resolve.h"
#include "multivec.h"
#include <iostream>

struct ShrinkOp
{
	Direction direction;		// Side to shrink
	unsigned int quantity;		// Amount of blocks to shrink

	ShrinkOp(Direction direction, unsigned int quantity)
	: direction(direction), quantity(quantity) {}
};

struct OperationAction
{
	MeshBox& target;
	ShrinkOp action;
	Cuboid region;
};

typedef std::vector<OperationAction> OperationActions;

class ConflictGraph
{
public:
	//struct Branch;

	struct Node
	{
		MeshBox& meshBox;
	};

	struct Branch
	{
		const Node& node1;
		const Node& node2;

		Cuboid region;
	};

	ConflictGraph(
		std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
		mv::vector3<GridCell>& gridCells);

	[[nodiscard]] std::list<OperationActions> getResolutionOperations(
		mv::vector3<GridCell>& gridCells);

	friend std::ostream& operator<<(std::ostream& strm, const ConflictGraph& graph);

private:
	std::vector<Node> m_Nodes;
	std::vector<Branch> m_Branches;

	void constructNodesFrom(std::vector<std::unique_ptr<MeshBox>>& boxes);

	void enumerateBranches();

	[[nodiscard]] bool alreadyExists(
		const std::vector<Branch>& branches,
		const Node& node1, 
		const Node& node2) 
		const;

	[[nodiscard]] std::optional<Cuboid> getOverlapRegion(
		const Node& node1, const Node& node2) const;

	[[nodiscard]] OperationActions getResolutionsFor(Branch& branch);

	[[nodiscard]] std::vector<ShrinkOp> getResolutionsFor(
		const Node& node, Cuboid region) const;

	[[nodiscard]] Cuboid getResolutionRegion(
		const MeshBox& meshbox, const ShrinkOp& op) const;

	// TODO: Assign nodes and subnodes from mesh boxes
	// TODO: Enumerate overlaps from mesh box cuboid data
	//
	// TODO: Conflict resolve
};

ConflictGraph::ConflictGraph(
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
	mv::vector3<GridCell>& gridCells)
{
	constructNodesFrom(sourceBoxes);
	enumerateBranches();
}

void ConflictGraph::constructNodesFrom(
	std::vector<std::unique_ptr<MeshBox>>& boxes)
{
	for(auto& box: boxes)
	{
		m_Nodes.emplace_back((Node){ *box });
	}
}

void ConflictGraph::enumerateBranches()
{
	for(int i = 0; i < m_Nodes.size(); ++i)
	{
		Node& node = m_Nodes[i];
		for(int j = 0; j < m_Nodes.size(); ++j)
		{
			if(i == j)
				continue;

			Node& other = m_Nodes[j];

			if(alreadyExists(m_Branches, node, other))
				continue;

			std::optional<Cuboid> overlapRegion = getOverlapRegion(node, other);
			if(overlapRegion)
			{
				m_Branches.push_back((Branch) {
					node, other, *overlapRegion }
				);
			}
		}
	}
}

bool ConflictGraph::alreadyExists(
	const std::vector<Branch>& branches,
	const Node& node1, 
	const Node& node2) 
	const
{
	for(const auto& branch: branches)
	{
		const Node* node1Ptr = std::addressof(node1);
		const Node* node2Ptr = std::addressof(node2);
		const Node* branchNode1Ptr = std::addressof(branch.node1);
		const Node* branchNode2Ptr = std::addressof(branch.node2);

		if((branchNode1Ptr == node1Ptr && branchNode2Ptr == node2Ptr) ||
		   (branchNode2Ptr == node1Ptr && branchNode1Ptr == node2Ptr))
		{
			return true;
		}
	}

	return false;
}

std::optional<Cuboid> ConflictGraph::getOverlapRegion(
	const Node& node1, const Node& node2) const
{
	const Cuboid& cube1 = node1.meshBox.dims;
	const Cuboid& cube2 = node2.meshBox.dims;

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

	bool overlapX = overlapSize.x > 0;
	bool overlapY = overlapSize.y > 0;
	bool overlapZ = overlapSize.z > 0;

	if(overlapSize.x > 0 && overlapSize.y > 0 && overlapSize.z > 0)
		return Cuboid(overlapOrigin, overlapSize);
	else
		return std::nullopt;
}

std::list<OperationActions> ConflictGraph::getResolutionOperations(
	mv::vector3<GridCell>& gridCells)
{
	std::list<OperationActions> resolutionOps;

	for(Branch& branch: m_Branches)
		resolutionOps.emplace_back(getResolutionsFor(branch));

	return resolutionOps;
}

OperationActions ConflictGraph::getResolutionsFor(Branch& branch)
{
	OperationActions operations;

	std::vector<ShrinkOp> node1Ops = getResolutionsFor(branch.node1, branch.region);
	for(const ShrinkOp& op: node1Ops)
		operations.emplace_back((OperationAction) {
			branch.node1.meshBox, op, getResolutionRegion(
					branch.node1.meshBox, op)
		});

	std::vector<ShrinkOp> node2Ops = getResolutionsFor(branch.node2, branch.region);
	for(const ShrinkOp& op: node2Ops)
		operations.emplace_back((OperationAction) {
			branch.node2.meshBox, op, getResolutionRegion(
				branch.node2.meshBox, op)
		});

	return operations;
}

std::vector<ShrinkOp>
ConflictGraph::getResolutionsFor(const Node& node, Cuboid region) const
{
	std::vector<ShrinkOp> resolutions;

	if(region.origin.x > node.meshBox.dims.origin.x && region.size.x < node.meshBox.dims.size.x)
		resolutions.emplace_back(Direction::Left, region.size.x);
	if(region.origin.y > node.meshBox.dims.origin.y && region.size.y < node.meshBox.dims.size.y)
		resolutions.emplace_back(Direction::Down, region.size.y);
	if(region.origin.z > node.meshBox.dims.origin.z && region.size.z < node.meshBox.dims.size.z)
		resolutions.emplace_back(Direction::In, region.size.z);

	if(region.end().x == node.meshBox.dims.end().x && region.size.x < node.meshBox.dims.size.x)
		resolutions.emplace_back(Direction::Right, region.size.x);
	if(region.end().y == node.meshBox.dims.end().y && region.size.y < node.meshBox.dims.size.y)
		resolutions.emplace_back(Direction::Up, region.size.y);
	if(region.end().z == node.meshBox.dims.end().z && region.size.z < node.meshBox.dims.size.z)
		resolutions.emplace_back(Direction::Out, region.size.z);

	return resolutions;
}

[[nodiscard]] Cuboid ConflictGraph::getResolutionRegion(
	const MeshBox& meshbox, const ShrinkOp& op) const
{
	const Cuboid& dims = meshbox.dims;
	auto quantity = static_cast<int>(op.quantity);

	switch(op.direction)
	{
		case Direction::Left:
		{
			const Vector3D end = dims.origin + dims.size;
			const Vector3D size = Vector3D(quantity, dims.size.y, dims.size.z);
			return { end - size, size };
		}
		case Direction::Right:
		{
			return {
				dims.origin, Vector3D(quantity, dims.size.y, dims.size.z)
			};
		}
		case Direction::Down:
		{
			const Vector3D end = dims.origin + dims.size;
			const Vector3D size = Vector3D(dims.size.x, quantity, dims.size.z);
			return { end - size, size };
		}
		case Direction::Up:
		{
			return {
				dims.origin, Vector3D(dims.size.x, quantity, dims.size.z)
			};
		}
		case Direction::Out:
		{
			const Vector3D end = dims.origin + dims.size;
			const Vector3D size = Vector3D(dims.size.x, dims.size.y, quantity);
			return { end - size, size };
		}
		case Direction::In:
		{
			return {
				dims.origin, Vector3D(dims.size.x, dims.size.y, quantity)
			};
		}
	}

	return {{}, {}};    // Should never get here!
}

std::ostream& operator<<(std::ostream& strm, const ConflictGraph& graph)
{
	std::unordered_map<const ConflictGraph::Node*,int> idMap;
	for(int i = 0; i < graph.m_Nodes.size(); ++i)
		idMap[std::addressof(graph.m_Nodes.at(i))] = i;

	for(int i = 0; i < graph.m_Branches.size(); ++i)
	{
		const ConflictGraph::Branch& branch = graph.m_Branches.at(i);
		strm << "Branch " << i << " (";
		strm << "Node " << idMap.at(std::addressof(branch.node1)) << " to ";
		strm << "Node " << idMap.at(std::addressof(branch.node2)) << "):" << std::endl;

		strm << "\tRegion: "
			<< branch.region.corners() << ", [size: " << branch.region.size << ']' << std::endl
			<< "\tNode " << idMap.at(std::addressof(branch.node1)) << ": "
			<< branch.node1.meshBox.dims.corners() << ", [size: " << branch.node1.meshBox.dims.size << ']' << std::endl
			<< "\tNode " << idMap.at(std::addressof(branch.node2)) << ": "
			<< branch.node2.meshBox.dims.corners() << ", [size: " << branch.node2.meshBox.dims.size << ']' << std::endl;
	}

	return strm;
}

void printOperations(
	const std::list<OperationActions>& operations,
	const std::vector<std::unique_ptr<MeshBox>>& sourceBoxes)
{
	for(const OperationActions& actionSet: operations)
	{
		for(const OperationAction& action: actionSet)
		{
			unsigned int index = 0;
			for(const auto& meshbox: sourceBoxes)
			{
				if(meshbox.get() == std::addressof(action.target))
					break;
				++index;
			}

			std::cout << "Shrink Mesh Box " << index << " ";
			std::cout << toText(action.action.direction) << " by ";
			std::cout << action.action.quantity << std::endl;
		}
	}
}

std::vector<GridCell*> sampleCells(
	mv::vector3<GridCell>& gridCells, const Cuboid& cuboid)
{
	const Vector3D end = cuboid.end();
	std::vector<GridCell*> cells;

	//std::cout << "Sampling " << cuboid.origin << " to " << end << " (excl) "
	//		  << cuboid.size << std::endl;
	for(int x = cuboid.origin.x; x < end.x; ++x)
	{
		for(int y = cuboid.origin.y; y < end.y; ++y)
		{
			for(int z = cuboid.origin.z; z < end.z; ++z)
			{
				GridCell* cell = std::addressof(mv::get(gridCells, x, y, z));
				assert(cell);
				cells.push_back(cell);
			}
		}
	}

	return cells;
}

void recomputeAABB(MeshBox& box)
{
	Vector3D min = box.children.front()->position;
	Vector3D max = box.children.front()->position;
	for(GridCell* child: box.children)
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
	}
	Vector3D size = (max - min);
	box.dims = Cuboid(min, (max - min));
}

/*void execute(
	mv::vector3<GridCell>& gridCells,
	OperationAction& action,
	const std::vector<std::unique_ptr<MeshBox>>& sourceBoxes)
{
	unsigned int index = 0;
	for(const auto& box: sourceBoxes)
	{
		if(box.get() == std::addressof(action.target))
			break;
		++index;
	}

	Cuboid oldDims = action.target.dims;
	std::cout << "Shrinking " << toText(action.action.direction)
			  << " of Mesh Box " << index
			  << " by " << action.action.quantity
			  << " (region: " << action.region << ")" << std::endl;

	std::vector<GridCell*> cells = sampleCells(gridCells, action.region);

	for(GridCell* cell: cells)
	{
		action.target.children.remove(cell);
		cell->parents.remove(std::addressof(action.target));
	}

	recomputeAABB(action.target);

	std::cout << "\t=> Then: " << oldDims << ", Now: " << action.target.dims << std::endl;
}*/

void applyShrinkOp(mv::vector3<GridCell>& gridCells, const ShrinkOp& op, MeshBox& box)
{
	// Recompute AABB from shrink op direction (this shrinks the directed side)
	switch(op.direction)
	{
		case Direction::Left:
		{
			box.dims.origin.x += static_cast<int>(op.quantity);
			box.dims.size.x -= static_cast<int>(op.quantity);
		}
		break;
		case Direction::Right:
		{
			box.dims.size.x -= static_cast<int>(op.quantity);
		}
		break;
		case Direction::Up:
		{
			box.dims.size.y -= static_cast<int>(op.quantity);
		}
		break;
		case Direction::Down:
		{
			box.dims.origin.y += static_cast<int>(op.quantity);
			box.dims.size.y -= static_cast<int>(op.quantity);
		}
		break;
		case Direction::In:
		{
			box.dims.origin.z += static_cast<int>(op.quantity);
			box.dims.size.z -= static_cast<int>(op.quantity);
		}
		break;
		case Direction::Out:
		{
			box.dims.size.z -= static_cast<int>(op.quantity);
		}
		break;
	}

	// Enumerate the cells which are now no longer part of the box
	std::vector<GridCell*> newBoxCells = sampleCells(gridCells, box.dims);
	std::vector<GridCell*> exclBoxCells;
	for(GridCell* cell: box.children)
	{
		if(std::find(newBoxCells.begin(), newBoxCells.end(), cell) == std::end(newBoxCells))
			exclBoxCells.push_back(cell);
	}

	// ... and fully remove them
	for(GridCell* cell: exclBoxCells)
	{
		box.children.remove(cell);
		cell->parents.remove(std::addressof(box));
	}
}

void execute(
	mv::vector3<GridCell>& gridCells,
	OperationAction& action,
	const std::vector<std::unique_ptr<MeshBox>>& sourceBoxes)
{
	unsigned int index = 0;
	for(const auto& box: sourceBoxes)
	{
		if(box.get() == std::addressof(action.target))
			break;
		++index;
	}

	Cuboid oldDims = action.target.dims;
	std::cout << "Shrinking " << toText(action.action.direction)
	          << " of Mesh Box " << index
	          << " by " << action.action.quantity << std::endl;

	applyShrinkOp(gridCells, action.action, action.target);
}

/*OperationAction getBestOperation(OperationActions& actions)
{

}*/

//void applyBestOperations(

bool bestOperationSort(const OperationActions& a, const OperationActions& b)
{
	return true;
}

void resolveConflicts(
	const Mesh& parent,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
	mv::vector3<GridCell>& gridCells,
	Grid& grid)
{
	size_t empties = 0;
	empties = mv::reduce<GridCell,size_t>([](size_t& acc, GridCell& cell) {
		if(cell.type == GridCell::ContentType::Boundary && cell.parents.empty())
			++acc;
	}, gridCells, empties);

	std::cout << "Pre-Orphaned Cells: " << empties << std::endl;

	std::cout << "Resolving conflicts..." << std::endl;

	ConflictGraph graph(sourceBoxes, gridCells);

	std::cout << graph << std::endl;

	std::list<OperationActions> operations = graph.getResolutionOperations(
		gridCells);
	//printOperations(operations, sourceBoxes);
	//operations.sort(bestOperationSort);

	std::set<MeshBox*> meshBoxesDone;
	for(OperationActions& actions: operations)
	{
		if(!actions.empty())
		{
			if(meshBoxesDone.contains(std::addressof(actions[0].target)))
				continue;

			execute(gridCells, actions[0], sourceBoxes);

			meshBoxesDone.insert(std::addressof(actions[0].target));
		}
	}

	std::cout << "After resolution... " << std::endl;

	ConflictGraph graph2(sourceBoxes, gridCells);
	std::cout << graph2 << std::endl;
	//printOperations(graph2.getResolutionOperations(gridCells), sourceBoxes);

	empties = 0;
	empties = mv::reduce<GridCell,size_t>([](size_t& acc, GridCell& cell) {
		if(cell.type == GridCell::ContentType::Boundary && cell.parents.empty())
			++acc;
	}, gridCells, empties);

	std::cout << "Orphaned Cells: " << empties << std::endl;
}

