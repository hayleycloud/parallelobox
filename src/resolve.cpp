#include "resolve.h"

struct OperationAction
{
	MeshBox& target;
	Direction direction;
	unsigned int quantity;
};

typedef std::vector<OperationAction> Operation;

class ConflictGraph
{
public:
	struct Node;
	struct Branch;

	struct SubNode
	{
		const Node* parent;
		std::vector<Branch*> branches;

		SubNode(const Node* parent) : parent(parent) {}
	};

	struct Node
	{
		const MeshBox& meshBox;
		std::array<SubNode,6> directions;
	};

	struct Branch
	{
		const SubNode& node1;
		const SubNode& node2;

		Cuboid region;
	};

	ConflictGraph(
		std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
		mv::vector3<GridCell>& gridCells);

	std::list<Operation> getResolutionOperations();

private:
	std::vector<Node> m_Nodes;
	std::vector<std::unique_ptr<Branch>> m_Branches;

	std::array<SubNode,6> createSubNodesFor(const Node& node) const;

	void constructNodesFrom(std::vector<std::unique_ptr<MeshBox>>& boxes);

	void enumerateBranches();

	[[nodiscard]] bool alreadyExists(
		const std::vector<std::unique_ptr<Branch>>& branches,
		const SubNode& node1, 
		const SubNode& node2) 
		const;

	[[nodiscard]] std::optional<Cuboid> getOverlapRegion(
		const Node& node1, const Node& node2) const;

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

std::array<ConflictGraph::SubNode,6> 
ConflictGraph::createSubNodesFor(const Node& node) const
{
	return {
		SubNode(&node),
		SubNode(&node),
		SubNode(&node),
		SubNode(&node),
		SubNode(&node),
		SubNode(&node)
	};
}

void ConflictGraph::constructNodesFrom(
	std::vector<std::unique_ptr<MeshBox>>& boxes)
{
	for(auto& box: boxes)
	{
		m_Nodes.emplace_back((Node){ *box, {} });
		m_Nodes.back().directions = createSubNodesFor(m_Nodes.back());
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

				m_Branches.push_back(std::make_unique<Branch>((Branch){
					node, other, *overlapRegion }));
			}
		}
	}
}

bool ConflictGraph::alreadyExists(
	const std::vector<std::unique_ptr<Branch>>& branches,
	const SubNode& node1, 
	const SubNode& node2) 
	const
{
	for(const auto& branch: branches)
	{
		const SubNode* node1Ptr = std::addressof(node1);
		const SubNode* node2Ptr = std::addressof(node2);
		const SubNode* branchNode1Ptr = std::addressof(branch->node1);
		const SubNode* branchNode2Ptr = std::addressof(branch->node2);

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

}


void resolveConflicts(
	const Mesh& parent,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
	mv::vector3<GridCell>& gridCells,
	Grid& grid)
{
	std::list<Operation> operations;


}

