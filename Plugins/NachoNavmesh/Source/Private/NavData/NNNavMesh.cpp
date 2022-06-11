#include "NavData/NNNavMesh.h"

// UE Includes
#include "AbstractNavData.h"
#include "NavigationSystem.h"

// NN Includes
#include "NavData/NNNavMeshGenerator.h"
#include "NavData/NNNavMeshRenderingComp.h"

ANNNavMesh::ANNNavMesh()
{
	FindPathImplementation = FindPath;
	DefaultQueryFilter->SetFilterImplementation(new FAbstractQueryFilter());
}

FPathFindingResult ANNNavMesh::FindPath(const FNavAgentProperties& AgentProperties, const FPathFindingQuery& Query)
{
	const ANNNavMesh* Self = Cast<ANNNavMesh>(Query.NavData.Get());
	const FNNNavMeshGenerator* Generator = static_cast<FNNNavMeshGenerator*>(Self->NavDataGenerator.Get());
	FPathFindingResult Result = Generator->FindPath(AgentProperties, Query);
	return Result;
}

bool ANNNavMesh::ProjectPoint(const FVector& Point, FNavLocation& OutLocation, const FVector& Extent,
	FSharedConstNavQueryFilter Filter, const UObject* Querier) const
{
	const FNNNavMeshGenerator* Generator = static_cast<FNNNavMeshGenerator*>(NavDataGenerator.Get());
	return Generator->ProjectPoint(Point, OutLocation, Extent, Filter, Querier);
}

bool ANNNavMesh::GetPolygonFromNavLocation(const FNavLocation& NavLocation, FNNPolygon& OutPolygon) const
{
	const FNNNavMeshGenerator* Generator = static_cast<FNNNavMeshGenerator*>(NavDataGenerator.Get());
	return Generator->GetPolygonFromNavLocation(NavLocation, OutPolygon);
}

void ANNNavMesh::ConditionalConstructGenerator()
{
	UE_LOG(LogTemp, Warning, TEXT("%s"), ANSI_TO_TCHAR(__FUNCTION__));
	if (!GetWorld()->IsGameWorld())
	{
		NavDataGenerator = MakeShareable(new FNNNavMeshGenerator(*this));
	}
}

UPrimitiveComponent* ANNNavMesh::ConstructRenderingComponent()
{
	return NewObject<UNNNavMeshRenderingComp>(this, TEXT("NavRenderingComp"), RF_Transient);
}

const TSet<FNavigationBounds>& ANNNavMesh::GetRegisteredBounds() const
{
	const UNavigationSystemV1* NavigationSystem = UNavigationSystemV1::GetNavigationSystem(GetWorld());
	return NavigationSystem->GetNavigationBounds();
}

void ANNNavMesh::GrabDebuggingInfo(FNNNavMeshDebuggingInfo& DebuggingInfo) const
{
	const FNNNavMeshGenerator* Generator = static_cast<FNNNavMeshGenerator*>(NavDataGenerator.Get());
	Generator->GrabDebuggingInfo(DebuggingInfo);
}

FBox ANNNavMesh::GetNavMeshBounds() const
{
	const TSet<FNavigationBounds> Bounds = GetRegisteredBounds();
	FBox TotalBounds;
	for (const FNavigationBounds& Bound : Bounds)
	{
		TotalBounds += Bound.AreaBox;
	}
	return TotalBounds;
}

void ANNNavMesh::PostEditChangeChainProperty(FPropertyChangedChainEvent& PropertyChangedEvent)
{
	Super::PostEditChangeChainProperty(PropertyChangedEvent);
	RebuildAll();
}
