#pragma once

#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

struct FNNAreaGeneratorData;
struct FNNPolygonMesh;

struct FNNNode
{
	FNNNode(const FVector& InPosition) : Position(InPosition) {}

	FVector Position;
	TMap<int32, float> Neighbours;
};

struct FNNGraph
{
	TArray<FNNNode> Nodes;
};

class FNNPathfinding
{
public:
	FNNPathfinding(FNNAreaGeneratorData& InAreaGeneratorData) : AreaGeneratorData(&InAreaGeneratorData) {}

	/** Created a Graph for pathfinding with the given PolygonMesh */
	void CreateGraph(const FNNOpenHeightField& OpenHeightField, const FNNPolygonMesh& PolygonMesh, FNNGraph& OutGraph) const;

protected:
	/** Adds a new neighbours to the Node */
	void AddNodeNeighbour(FNNNode& Node, const FVector& Neighbour, int32 NeighbourIndex) const;

private:
	mutable FNNAreaGeneratorData* AreaGeneratorData;
};
