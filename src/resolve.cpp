#include "resolve.h"

struct ShrinkOp
{
	Direction direction;
	unsigned int quantity;
};

struct OperationAction
{
	MeshBox& target;
	ShrinkOp action;
};

typedef std::vector<OperationAction> Operation;

class ConflictGraph
{
public:
	//struct Branch;

	struct Node
	{
		const MeshBox& meshBox;
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

	[[nodiscard]] std::list<Operation> getResolutionOperations();

	friend ostream& operator<<(ostream& stream, const ConflictGraph& graph);

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

	[[nodiscard]] Operation getResolutionFor(Branch& branch);

	const ostream& operator<<(const ostream& stream) const;

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
	for(Node& node: m_Nodes)
	{
		for(Node& other: m_Nodes)
		{
			if(std::addressof(other) == std::addressof(node))
				continue;
 
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

	bool overlapX = overlapSize.x > 0 && overlapSize.x < cube1.size.x && overlapSize.x < cube2.size.x;
	bool overlapY = overlapSize.y > 0 && overlapSize.y < cube1.size.y && overlapSize.y < cube2.size.y;
	bool overlapZ = overlapSize.z > 0 && overlapSize.z < cube1.size.z && overlapSize.z < cube2.size.z;

	if(overlapX && overlapY && overlapZ)
		return Cuboid(overlapOrigin, overlapSize);
	else
		return std::nullopt;
}

std::list<Operation> ConflictGraph::getResolutionOperations()
{
	std::list<Operation> resolutionOps;

	for(Branch& branch: m_Branches)
	{
		std::vector<OperationAction> actions = getResolutionFor(branch);
	}

	return resolutionOps;
}

Operation ConflictGraph::getResolutionFor(Branch& branch)
{
	if(branch.node
}

std::vector<ShrinkOp>
ConflictGraph::getResolutionFor(Node& node, Cuboid region)
{
	if(region.origin.x > node.meshBox.dims.x)
		

}

ostream& operator<<(ostream& stream, const ConflictGraph& graph)
{
	std::unordered_map<const ConflictGraph::Node*,int> idMap;
	for(int i = 0; i < graph.m_Nodes.size(); ++i)
		idMap[graph.m_Nodes.at(i)] = i;

	for(int i = 0; i < graph.m_Branches.size(); ++i)
	{
		const ConflictGraph::Branch& branch = graph.m_Branches.at(i);
		stream << "Branch " << i << " (" << std::endl;
		stream << "Node " << idMap.at(branch.node1) << " to ";
		stream << "Node " << idMap.at(branch.node2) << "):" << std::endl;

		stream << "\tRegion: " 
			<< "[ origin: (" 
			<< branch.region.origin.x << "," 
			<< branch.region.origin.y << ","
			<< branch.region.origin.z << "), size: ("
			<< branch.region.size.x << ","
			<< branch.region.size.y << ","
			<< branch.region.size.z << ")]" << std::endl;
	}

	return stream;
}

void resolveConflicts(
	const Mesh& parent,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
	mv::vector3<GridCell>& gridCells,
	Grid& grid)
{
	std::list<Operation> operations;


}

