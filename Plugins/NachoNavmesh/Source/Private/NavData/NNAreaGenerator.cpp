#include "NavData/NNAreaGenerator.h"

// UE Includes
#include "NavigationSystem.h"

// NN Includes
#include "NavData/Contour/NNContourGeneration.h"
#include "NavData/NNNavMeshGenerator.h"
#include "NavData/Pathfinding/NNPathfinding.h"
#include "NavData/Regions/NNRegionGenerator.h"
#include "NavData/Voxelization/HeightFieldGenerator.h"
#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

FVector FNNRawGeometryElement::GetGeometryPosition(int32 Index) const
{
	return FVector(-GeomCoords[Index * 3], -GeomCoords[Index * 3 + 2], GeomCoords[Index * 3 + 1]);
}

FNNGeometryCache::FNNGeometryCache(const uint8* Memory)
{
	Header = *((FHeader*)Memory);
	Verts = (float*)(Memory + sizeof(FNNGeometryCache));
	Indices = (int32*)(Memory + sizeof(FNNGeometryCache) + (sizeof(float) * Header.NumVerts * 3));
}

void FNNAreaGeneratorData::AddDebugPoint(const FVector& Point, float Radius)
{
	FBoxSphereBounds PointToDebug = FBoxSphereBounds(FSphere(Point, Radius));
	TemporaryBoxSpheres.Add(MoveTemp(PointToDebug));
}

void FNNAreaGeneratorData::AddDebugText(const FVector& Location, const FString& Text)
{
	TemporaryTexts.Emplace(Location, Text);
}

void FNNAreaGeneratorData::AddDebugLine(const FVector& Start, const FVector& End)
{
	TemporaryLines.Emplace(Start, End, FColor::Blue, 2.0f);
}

void FNNAreaGeneratorData::AddDebugArrow(const FVector& Start, const FVector& End, const FColor& Color)
{
	TemporaryArrows.Emplace(Start, End, Color);
}

FNNAreaGenerator::FNNAreaGenerator(const FNNNavMeshGenerator* InParentGenerator, const FNavigationBounds& Bounds)
	: AreaBounds(Bounds), ParentGenerator(InParentGenerator)
{
}

void FNNAreaGenerator::DoWork()
{
	check(ParentGenerator);

	AreaGeneratorData = MakeUnique<FNNAreaGeneratorData>();

	GatherGeometry(true);

	if (!ensure(AreaGeneratorData->RawGeometry.Num() > 0))
	{
		return;
	}

	const TWeakObjectPtr<ANNNavMesh> NavMesh = ParentGenerator->GetOwner();
	const float HeightFieldHeight = NavMesh->CellHeight; // Z Axis
	const float HeightFieldSize = NavMesh->CellSize; // X and Y Axis

	const FVector& MinimumPoint = AreaBounds.AreaBox.Min;
	const FVector& MaximumPoint = AreaBounds.AreaBox.Max;

	// Create Solid HeightField
	const FHeightFieldGenerator HeightFieldGenerator (*AreaGeneratorData);
	HeightFieldGenerator.InitializeHeightField(AreaGeneratorData->HeightField ,
		AreaGeneratorData->RawGeometry, MinimumPoint, MaximumPoint, HeightFieldSize,
		HeightFieldHeight, NavMesh->WalkableSlopeDegrees, NavMesh->AgentHeight, NavMesh->MaxLedgeHeight);


	// Create Open HeightField
	const FOpenHeightFieldGenerator OpenHeightFieldGenerator (*AreaGeneratorData);
	OpenHeightFieldGenerator.GenerateOpenHeightField(AreaGeneratorData->OpenHeightField, AreaGeneratorData->HeightField, NavMesh->MaxLedgeHeight, NavMesh->AgentHeight);

	// Generate Regions for the Open HeightField
	constexpr FNNRegionGenerator RegionGenerator;
	const int32 MinTraversableSize = FMath::CeilToInt(NavMesh->AgentRadius / NavMesh->CellSize);
	RegionGenerator.CreateRegions(AreaGeneratorData->OpenHeightField, NavMesh->MinRegionSize, MinTraversableSize);

	// Generate Contour
	FNNContourGeneration ContourGeneration (*AreaGeneratorData, NavMesh->ContourDeviationThreshold, NavMesh->MaxEdgeLength);
	ContourGeneration.CalculateContour(AreaGeneratorData->OpenHeightField, AreaGeneratorData->Contours);

	// Triangulate Contour
	FNNPolyMeshBuilder MeshBuilder;
	MeshBuilder.GenerateConvexPolygon(AreaGeneratorData->Contours, AreaGeneratorData->PolygonMesh);

	// Pathfinding graph
	const FNNPathfinding Pathfinding (*AreaGeneratorData, AreaGeneratorData->OpenHeightField);
	Pathfinding.CreateGraph(AreaGeneratorData->PolygonMesh, AreaGeneratorData->PathfindingGraph);

	// Gives each polygon an unique ID
	for (int32 i = 0; i < AreaGeneratorData->PolygonMesh.PolygonIndexes.Num(); ++i)
	{
		FNNPolygon& Polygon = AreaGeneratorData->PolygonMesh.PolygonIndexes[i];
		Polygon.NodeRef = ParentGenerator->GeneratePolygonNodeRef(AreaBounds.UniqueID, i);
	}
}

void FNNAreaGenerator::GatherGeometry(bool bGeometryChanged)
{
	check(ParentGenerator);
	QUICK_SCOPE_CYCLE_COUNTER(STAT_RecastNavMeshGenerator_GatherGeometry);

	UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(ParentGenerator->GetWorld());
	FNavigationOctree* NavigationOctree = NavSys ? NavSys->GetMutableNavOctree() : nullptr;
	if (NavigationOctree == nullptr)
	{
		return;
	}
	const FNavDataConfig& OwnerNavDataConfig = ParentGenerator->GetOwner()->GetConfig();

	NavigationOctree->FindElementsWithBoundsTest(ParentGenerator->GrowBoundingBox(AreaBounds.AreaBox, /*bIncludeAgentHeight*/ false), [&OwnerNavDataConfig, &NavigationOctree, this, NavSys, bGeometryChanged](const FNavigationOctreeElement& Element)
	{
		const bool bShouldUse = Element.ShouldUseGeometry(OwnerNavDataConfig);
		if (bShouldUse)
		{
			GatherNavigationDataGeometry(Element.Data, *NavSys, OwnerNavDataConfig, bGeometryChanged);
		}
	});
}

void FNNAreaGenerator::GatherNavigationDataGeometry(
	const TSharedRef<FNavigationRelevantData, ESPMode::ThreadSafe>& ElementData, UNavigationSystemV1& NavSys,
	const FNavDataConfig& OwnerNavDataConfig, bool bGeometryChanged)
{
	if (ElementData->IsPendingLazyGeometryGathering() || ElementData->NeedAnyPendingLazyModifiersGathering())
	{
		QUICK_SCOPE_CYCLE_COUNTER(STAT_RecastNavMeshGenerator_LazyGeometryExport);
		NavSys.DemandLazyDataGathering(*ElementData);
	}

	// TODO (ignacio) Support for Geometry slices?

	const FCompositeNavModifier ModifierInstance = ElementData->GetModifierForAgent(&OwnerNavDataConfig);

	const bool bExportGeometry = bGeometryChanged && ElementData->HasGeometry();
	if (bExportGeometry)
	{
		// TODO (ignacio) Support for vocel cache?

		const FNavigationRelevantData& DataRef = ElementData.Get();
		if (DataRef.IsCollisionDataValid())
		{
			AppendGeometry(DataRef, ModifierInstance, DataRef.NavDataPerInstanceTransformDelegate);
		}
	}

	// TODO (ignacio) Here we should take into account modifiers
	// if (ModifierInstance.IsEmpty() == false)
	// {
	// 	AppendModifier(ModifierInstance, ElementData->NavDataPerInstanceTransformDelegate);
	// }
}

void FNNAreaGenerator::AppendGeometry(const FNavigationRelevantData& DataRef, const FCompositeNavModifier& InModifier,
	const FNavDataPerInstanceTransformDelegate& InTransformsDelegate)
{
	const TNavStatArray<uint8>& RawCollisionCache = DataRef.CollisionData;
	if (RawCollisionCache.Num() == 0)
	{
		return;
	}

	FNNRawGeometryElement GeometryElement;

	// TODO (ignacio) we might want to set rasterization flags here

	const FNNGeometryCache CollisionCache(RawCollisionCache.GetData());

	// Gather per instance transforms
	// if (InTransformsDelegate.IsBound())
	// {
	// 	InTransformsDelegate.Execute(TileBBExpandedForAgent, GeometryElement.PerInstanceTransform);
	// 	if (GeometryElement.PerInstanceTransform.Num() == 0)
	// 	{
	// 		return;
	// 	}
	// }

	const int32 NumCoords = CollisionCache.Header.NumVerts * 3;
	const int32 NumIndices = CollisionCache.Header.NumFaces * 3;
	if (NumIndices > 0)
	{
		UE_LOG(LogNavigationDataBuild, VeryVerbose, TEXT("%s adding %i vertices from %s."), ANSI_TO_TCHAR(__FUNCTION__), CollisionCache.Header.NumVerts, *GetFullNameSafe(DataRef.GetOwner()));

		GeometryElement.GeomCoords.SetNumUninitialized(NumCoords);
		GeometryElement.GeomIndices.SetNumUninitialized(NumIndices);

		FMemory::Memcpy(GeometryElement.GeomCoords.GetData(), CollisionCache.Verts, sizeof(float) * NumCoords);
		FMemory::Memcpy(GeometryElement.GeomIndices.GetData(), CollisionCache.Indices, sizeof(int32) * NumIndices);

		AreaGeneratorData->RawGeometry.Add(MoveTemp(GeometryElement));
	}
}
