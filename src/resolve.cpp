#include "resolve.h"
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

	[[nodiscard]] std::list<OperationActions> getResolutionOperations();

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

std::list<OperationActions> ConflictGraph::getResolutionOperations()
{
	std::list<OperationActions> resolutionOps;

	for(Branch& branch: m_Branches)
	{
		resolutionOps.emplace_back(getResolutionsFor(branch));
	}

	return resolutionOps;
}

OperationActions ConflictGraph::getResolutionsFor(Branch& branch)
{
	OperationActions operations;

	std::vector<ShrinkOp> node1Ops = getResolutionsFor(branch.node1, branch.region);
	for(const ShrinkOp& op: node1Ops)
		operations.emplace_back((OperationAction) {
			branch.node1.meshBox, op
		});

	std::vector<ShrinkOp> node2Ops = getResolutionsFor(branch.node2, branch.region);
	for(const ShrinkOp& op: node2Ops)
		operations.emplace_back((OperationAction) {
			branch.node2.meshBox, op
		});

	return operations;
}

std::vector<ShrinkOp>
ConflictGraph::getResolutionsFor(const Node& node, Cuboid region) const
{
	std::vector<ShrinkOp> resolutions;

	if(region.origin.x > node.meshBox.dims.origin.x)
		resolutions.push_back(ShrinkOp(Direction::Left, region.size.x));
	if(region.origin.y > node.meshBox.dims.origin.y)
		resolutions.push_back(ShrinkOp(Direction::Down, region.size.y));
	if(region.origin.z > node.meshBox.dims.origin.z)
		resolutions.push_back(ShrinkOp(Direction::In, region.size.z));

	if(region.end().x == node.meshBox.dims.end().x)
		resolutions.push_back(ShrinkOp(Direction::Right, region.size.x));
	if(region.end().y == node.meshBox.dims.end().y)
		resolutions.push_back(ShrinkOp(Direction::Up, region.size.y));
	if(region.end().z == node.meshBox.dims.end().z)
		resolutions.push_back(ShrinkOp(Direction::Out, region.size.z));

	return resolutions;
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
			<< branch.node1.meshBox.dims.corners() << std::endl
			<< "\tNode " << idMap.at(std::addressof(branch.node2)) << ": "
			<< branch.node2.meshBox.dims.corners() << std::endl;
	}

	return strm;
}

/*void removeMeshBoxCells(
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
}*/

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
	std::cout << "Resolving conflicts..." << std::endl;

	ConflictGraph graph(sourceBoxes, gridCells);

	std::cout << graph << std::endl;

	std::list<OperationActions> operations = graph.getResolutionOperations();
	operations.sort(bestOperationSort);


}

