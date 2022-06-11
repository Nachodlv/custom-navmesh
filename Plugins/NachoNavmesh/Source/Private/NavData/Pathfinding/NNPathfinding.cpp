#include "NavData/Pathfinding/NNPathfinding.h"

// UE Includes
#include "Algo/ForEach.h"
#include "NavigationPath.h"
#include "NavigationSystem.h"

// NN Includes
#include "NavData/ConvexPolygon/NNPolyMeshBuilder.h"
#include "NavData/NNAreaGenerator.h"
#include "NavData/NNNavMesh.h"
#include "NavData/Pathfinding/NNPriorityQueue.h"

#define NN_DEBUG_PATHFINDING_GRAPH 0

#define PATH_START_INDEX -1
#define PATH_GOAL_INDEX - 2

void FNNPathfinding::CreateGraph(const FNNPolygonMesh& PolygonMesh, FNNGraph& OutGraph) const
{
	OutGraph.Nodes.Reserve(PolygonMesh.Vertexes.Num());
	for (int32 VertexIndex = 0; VertexIndex < PolygonMesh.Vertexes.Num(); ++VertexIndex)
	{
		const FVector& Vertex = PolygonMesh.Vertexes[VertexIndex];
		FNNNode Node (Vertex);
		for (int32 i = 0; i < PolygonMesh.PolygonIndexes.Num(); ++i)
		{
			const FNNPolygon& Polygon = PolygonMesh.PolygonIndexes[i];
			const int32 PolygonIndex = Polygon.Indexes.Find(VertexIndex);
			if (PolygonIndex == INDEX_NONE)
			{
				continue;
			}

			Node.PolygonIndexes.AddUnique(i);
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
	for (int32 i = 0; i < OutGraph.Nodes.Num(); ++i)
	{
		const FNNNode& Node = OutGraph.Nodes[i];
		const FVector& Start = OpenHeightField.TransformVectorToWorldPosition(Node.Position);
		AreaGeneratorData->AddDebugPoint(Start);
		for (const TTuple<int, float>& Neighbour : Node.Neighbours)
		{
			const FVector& End = OpenHeightField.TransformVectorToWorldPosition(OutGraph.Nodes[Neighbour.Key].Position);
			AreaGeneratorData->AddDebugArrow(Start, End, FColor::Red);
		}
		AreaGeneratorData->AddDebugText(Start, FString::FromInt(i));
	}
	for (int32 i = 0; i < PolygonMesh.PolygonIndexes.Num(); ++i)
	{
		const FNNPolygon& Polygon = PolygonMesh.PolygonIndexes[i];
		FVector PolygonPosition;
		Algo::ForEach(Polygon.Indexes, [&PolygonPosition, &PolygonMesh](const int32& Index)
		{
			PolygonPosition += PolygonMesh.Vertexes[Index];
		});
		PolygonPosition /= Polygon.Indexes.Num();
		PolygonPosition = OpenHeightField.TransformVectorToWorldPosition(PolygonPosition);
		AreaGeneratorData->AddDebugText(PolygonPosition, FString::FromInt(i));
	}
#endif //NN_DEBUG_PATHFINDING_GRAPH

}

FNavPathSharedPtr FNNPathfinding::FindPath(const FNNGraph& Graph, const FNNPolygonMesh& PolygonMesh,
	const FNavAgentProperties& AgentProperties, const FPathFindingQuery& Query) const
{
	TMap<int32, int32> CameFrom;
	TMap<int32, float> CostSoFar;
	FNNPriorityQueue<int32> Frontier;

	UNavigationSystemV1* NavSys = UNavigationSystemV1::GetCurrent(Query.Owner->GetWorld());
	FNavLocation NavGoal;
	FNavLocation NavStart;
	bool bProjected = NavSys->ProjectPointToNavigation(Query.EndLocation, NavGoal);
	bProjected &= NavSys->ProjectPointToNavigation(Query.StartLocation, NavStart);
	if (!bProjected)
	{
		return nullptr;
	}
	
	ANNNavMesh* NavMesh = Cast<ANNNavMesh>(NavSys->GetMainNavData());
	FNNPolygon PolyGoal;
	FNNPolygon PolyStart;
	bool bRetrieved = NavMesh->GetPolygonFromNavLocation(NavGoal, PolyGoal);
	bRetrieved &= NavMesh->GetPolygonFromNavLocation(NavStart, PolyStart);
	if (!bRetrieved)
	{
		return nullptr;
	}

	FVector Goal = OpenHeightField.TransformToHeightFieldPosition(NavGoal.Location);
	FVector Start = OpenHeightField.TransformToHeightFieldPosition(NavStart.Location);

	if (PolyGoal == PolyStart)
	{
		// On the same polygon we can move directly to our goal
		TArray<FVector> Path = {NavStart, NavGoal};
		FNavPathSharedPtr NavigationPath = MakeShared<FNavigationPath, ESPMode::ThreadSafe>(MoveTemp(Path));
		return NavigationPath;
	}

	TMap<int32, float> GoalNeighbours;
	TMap<int32, float> StartNeighbours;

	// We have an end and a start for every vertex on the the polygon in our current graph
	for (int32 i = 0; i < Graph.Nodes.Num(); ++i)
	{
		const FNNNode& Node = Graph.Nodes[i];
		for (int32 PolygonIndex : Node.PolygonIndexes)
		{
			const FNNPolygon& Polygon = PolygonMesh.PolygonIndexes[PolygonIndex];
			if (Polygon == PolyGoal)
			{
				GoalNeighbours.Add(i, CalculateHeuristic(Node.Position, Goal));
				break;
			}
			if (Polygon == PolyStart)
			{
				StartNeighbours.Add(i, CalculateHeuristic(Node.Position, Start));
				break;
			}
		}
	}

	Frontier.Push(PATH_START_INDEX, 0.0f);
	CameFrom.Add(PATH_START_INDEX, PATH_START_INDEX);
	CostSoFar.Add(PATH_START_INDEX, 0.0f);

	TMap<int32, float> WorkingNeighbours;
	
	while (!Frontier.IsEmpty())
	{
		int32 CurrentIndex = Frontier.Pop();

		if (CurrentIndex == PATH_GOAL_INDEX)
		{
			break;
		}
		if (CurrentIndex == PATH_START_INDEX)
		{
			WorkingNeighbours = MoveTemp(StartNeighbours);
		}
		else
		{
			const FNNNode& Current = Graph.Nodes[CurrentIndex];
			WorkingNeighbours = Current.Neighbours;
			if (float* Cost = GoalNeighbours.Find(CurrentIndex))
			{
				WorkingNeighbours.Add(PATH_GOAL_INDEX, *Cost);
			}
		}
		
		for (const auto& Neighbour : WorkingNeighbours)
		{
			const float NewCost = CostSoFar[CurrentIndex] + Neighbour.Value;
			const float* NeighbourCost = CostSoFar.Find(Neighbour.Key);
			if (!NeighbourCost || NewCost < *NeighbourCost)
			{
				CostSoFar.Add(Neighbour.Key, NewCost);
				float Priority = NewCost;
				Priority += Neighbour.Key != PATH_GOAL_INDEX
					? CalculateHeuristic(Graph.Nodes[Neighbour.Key].Position, Goal)
					:  0.0f;
				Frontier.Push(Neighbour.Key, Priority);
				CameFrom.Add(Neighbour.Key, CurrentIndex); 
			}
		}
		
		WorkingNeighbours.Reset();
	}
	if (Frontier.IsEmpty())
	{
		// TODO (ignacio) remove later
		return nullptr;
	}
	TArray<FVector> Path = {NavGoal};
	int32 Current = CameFrom[PATH_GOAL_INDEX];
	while (Current != PATH_START_INDEX)
	{
		Path.Add(TransformToWorldSpace(Graph.Nodes[Current].Position));
		Current = CameFrom[Current];
	}
	Path.Add(NavStart);
	Algo::Reverse(Path);
	FNavPathSharedPtr NavigationPath = MakeShared<FNavigationPath, ESPMode::ThreadSafe>(MoveTemp(Path));
	return NavigationPath;
}

void FNNPathfinding::AddNodeNeighbour(FNNNode& Node, const FVector& Neighbour, int32 NeighbourIndex) const
{
	const float NewCost = CalculateHeuristic(Node.Position, Neighbour);
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

float FNNPathfinding::CalculateHeuristic(const FVector& Lhs, const FVector& Rhs)
{
	return FVector::DistSquared(Lhs, Rhs);
}

FVector FNNPathfinding::TransformToWorldSpace(const FVector& Position) const
{
	return OpenHeightField.TransformVectorToWorldPosition(Position);
}

#undef PATH_START_INDEX
#undef PATH_GOAL_INDEX
