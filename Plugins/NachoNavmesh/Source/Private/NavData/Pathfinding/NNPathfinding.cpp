#include "NavData/Pathfinding/NNPathfinding.h"

// NN Includes
#include "NavData/ConvexPolygon/NNPolyMeshBuilder.h"
#include "NavData/NNAreaGenerator.h"

#define NN_DEBUG_PATHFINDING_GRAPH 0

void FNNPathfinding::CreateGraph(const FNNOpenHeightField& OpenHeightField, const FNNPolygonMesh& PolygonMesh, FNNGraph& OutGraph) const
{
	OutGraph.Nodes.Reserve(PolygonMesh.Vertexes.Num());
	for (int32 VertexIndex = 0; VertexIndex < PolygonMesh.Vertexes.Num(); ++VertexIndex)
	{
		const FVector& Vertex = PolygonMesh.Vertexes[VertexIndex];
		FNNNode Node (Vertex);
		for (const FNNPolygon& Polygon : PolygonMesh.PolygonIndexes)
		{
			const int32 PolygonIndex = Polygon.Indexes.Find(VertexIndex);
			if (PolygonIndex == INDEX_NONE)
			{
				continue;
			}

			const int32 PolygonIndexNum = Polygon.Indexes.Num();
			// Add the neighbours to the Node with the minimum cost
			int32 PolygonIndexNeighbour = Polygon.Indexes[(PolygonIndex + 1) % PolygonIndexNum];
			AddNodeNeighbour(Node, PolygonMesh.Vertexes[PolygonIndexNeighbour], PolygonIndexNeighbour);
			PolygonIndexNeighbour = Polygon.Indexes[(PolygonIndex + PolygonIndexNum - 1) % PolygonIndexNum];
			AddNodeNeighbour(Node, PolygonMesh.Vertexes[PolygonIndexNeighbour], PolygonIndexNeighbour);
		}
		OutGraph.Nodes.Add(MoveTemp(Node));
	}

#if NN_DEBUG_PATHFINDING_GRAPH
	for (const FNNNode& Node : OutGraph.Nodes)
	{
		const FVector& Start = OpenHeightField.TransformVectorToWorldPosition(Node.Position);
		AreaGeneratorData->AddDebugPoint(Start);
		for (const TTuple<int, float>& Neighbour : Node.Neighbours)
		{
			const FVector& End = OpenHeightField.TransformVectorToWorldPosition(OutGraph.Nodes[Neighbour.Key].Position);
			AreaGeneratorData->AddDebugArrow(Start, End, FColor::Red);
		}
	}
#endif //NN_DEBUG_PATHFINDING_GRAPH

}

void FNNPathfinding::AddNodeNeighbour(FNNNode& Node, const FVector& Neighbour, int32 NeighbourIndex) const
{
	const float NewCost = FVector::Distance(Node.Position, Neighbour);
	if (const float* Cost = Node.Neighbours.Find(NeighbourIndex))
	{
		// TODO (ignacio) we might want to delete this. I think that the costs will always be the same
		Node.Neighbours.Add(NeighbourIndex, FMath::Min(NewCost, *Cost));
	}
	else
	{
		Node.Neighbours.Add(NeighbourIndex, NewCost);
	}
}
