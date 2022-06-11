#pragma once

#include "NavigationSystemTypes.h"

#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

struct FNNAreaGeneratorData;
struct FNNPolygonMesh;

struct FNNNode
{
	FNNNode(const FVector& InPosition) : Position(InPosition) {}

	FVector Position;
	TMap<int32, float> Neighbours;
	/** To which polygons this node belongs */
	TArray<int32> PolygonIndexes;
};

struct FNNGraph
{
	TArray<FNNNode> Nodes;
};

class FNNPathfinding
{
public:
	FNNPathfinding(FNNAreaGeneratorData& InAreaGeneratorData, const FNNOpenHeightField& InOpenHeightField)
		: AreaGeneratorData(&InAreaGeneratorData), OpenHeightField(InOpenHeightField) {}

	/** Created a Graph for pathfinding with the given PolygonMesh */
	void CreateGraph(const FNNPolygonMesh& PolygonMesh, FNNGraph& OutGraph) const;

	/** Uses A* to find a path between the StartLocation and EndLocation of the given Query */
	FNavPathSharedPtr FindPath(const FNNGraph& Graph, const FNNPolygonMesh& PolygonMesh, const FNavAgentProperties& AgentProperties, const
	                           FPathFindingQuery& Query) const;

protected:
	/** Adds a new neighbours to the Node */
	void AddNodeNeighbour(FNNNode& Node, const FVector& Neighbour, int32 NeighbourIndex) const;

	/** Calculates the distance squared between Lhs and Rhs. Used for the A* algorithm */
	static float CalculateHeuristic(const FVector& Lhs, const FVector& Rhs);

	/** Transforms the Position in OpenHeightField space into world space */
	FVector TransformToWorldSpace(const FVector& Position) const;

private:
	mutable FNNAreaGeneratorData* AreaGeneratorData;

	const FNNOpenHeightField& OpenHeightField;
};
