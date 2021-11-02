#include "NavData/NNNavMesh.h"

ANNNavMesh::ANNNavMesh()
{
	FindPathImplementation = FindPath;
}

FPathFindingResult ANNNavMesh::FindPath(const FNavAgentProperties& AgentProperties, const FPathFindingQuery& Query)
{
	UE_LOG(LogTemp, Warning, TEXT("%s: looking for a path!"), ANSI_TO_TCHAR(__FUNCTION__));
	return FPathFindingResult();
}

void ANNNavMesh::ConditionalConstructGenerator()
{
	UE_LOG(LogTemp, Warning, TEXT("%s"), ANSI_TO_TCHAR(__FUNCTION__));
}

UPrimitiveComponent* ANNNavMesh::ConstructRenderingComponent()
{
	UE_LOG(LogTemp, Warning, TEXT("%s"), ANSI_TO_TCHAR(__FUNCTION__));
	return nullptr;
}
