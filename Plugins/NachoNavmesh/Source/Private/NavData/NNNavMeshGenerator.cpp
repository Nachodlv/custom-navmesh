#include "NavData/NNNavMeshGenerator.h"

// NN Includes
#include "NavData/NNNavMeshRenderingComp.h"


FNNNavMeshGenerator::FNNNavMeshGenerator(ANNNavMesh& InNavMesh)
	: NavMesh(&InNavMesh), NavBounds(InNavMesh.GetRegisteredBounds())
{}

void FNNNavMeshGenerator::TickAsyncBuild(float DeltaSeconds)
{
	ProcessDirtyAreas();
}

bool FNNNavMeshGenerator::RebuildAll()
{
	for (const FNavigationBounds& NavBound : NavBounds)
	{
		DirtyAreas.AddUnique(NavBound.UniqueID);
	}
	DirtyAreas.Empty();
	return true;
}

void FNNNavMeshGenerator::RebuildDirtyAreas(const TArray<FNavigationDirtyArea>& NavigationDirtyAreas)
{
	// TODO (ignacio) I probably want to rebuild only the dirty areas
	RebuildAll();
}

FNNAreaGenerator* FNNNavMeshGenerator::CreateAreaGenerator(const FNavigationBounds& DirtyArea)
{
	return new FNNAreaGenerator(this, DirtyArea);
}

void FNNNavMeshGenerator::ProcessDirtyAreas()
{
	if (DirtyAreas.Num() == 0)
	{
		return;
	}

	for (int32 i = DirtyAreas.Num() - 1; i >= 0; --i)
	{
		FNavigationBounds DirtyAreaSearch;
		DirtyAreaSearch.UniqueID = DirtyAreas[i];
		const FNavigationBounds* DirtyArea = NavBounds.Find(DirtyAreaSearch);
		check(DirtyArea);
		FNNAreaGenerator* AreaGenerator = CreateAreaGenerator(*DirtyArea);
		AreaGenerator->DoWork();
		GeneratorsData.Add(DirtyAreas[i], AreaGenerator->GetAreaGeneratorData());
		delete AreaGenerator;
	}
	Cast<UNNNavMeshRenderingComp>(NavMesh->RenderingComp)->ForceUpdate();
}

FBox FNNNavMeshGenerator::GrowBoundingBox(const FBox& BBox, bool bUseAgentHeight) const
{
	FVector BBoxGrowOffsetMin = FVector(0.0f);
	if (bUseAgentHeight && GetOwner().IsValid())
	{
		BBoxGrowOffsetMin += FVector(0.0f, 0.0f, GetOwner()->AgentHeight);
	}

	return FBox(BBox.Min - BBoxGrowth - BBoxGrowOffsetMin, BBox.Max + BBoxGrowth);
}

void FNNNavMeshGenerator::GrabDebuggingInfo(FNNNavMeshDebuggingInfo& DebuggingInfo) const
{
	for (const auto& Result : GeneratorsData)
	{
		DebuggingInfo.RawGeometryToDraw.Append(Result.Value.RawGeometry);
	}
}
