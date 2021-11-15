#include "NavData/NNAreaGenerator.h"

// UE Includes
#include "NavigationSystem.h"

// NN Includes
#include "NavData/HeightFieldGenerator.h"
#include "NavData/NNNavMeshGenerator.h"

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

FString Span::ToString() const
{
	return FString::Printf(TEXT("(%d, %d, %s)%s"),
		MinSpanHeight,
		MaxSpanHeight,
		bWalkable ? TEXT("w") : TEXT("nw"),
		NextSpan ? *FString::Printf(TEXT("->%s"), *NextSpan->ToString()) : *FString());
}

FNNHeightField::FNNHeightField(int32 InUnitsWidth, int32 InUnitsHeight, int32 InUnitsDepth)
	: UnitsWidth(InUnitsWidth), UnitsHeight(InUnitsHeight), UnitsDepth(InUnitsDepth)
{
	const int32 SpansLength = InUnitsWidth * InUnitsDepth;
	Spans.reserve(SpansLength);
	for (int32 i = 0; i < SpansLength; ++i)
	{
		Spans.push_back(nullptr);
	}
}

FNNAreaGeneratorData::~FNNAreaGeneratorData()
{
	delete HeightField;
	HeightField = nullptr;
	TemporaryBoxSpheres.Reset();
	RawGeometry.Reset();
	TemporaryLines.Reset();
	TemporaryTexts.Reset();
}

FNNAreaGenerator::FNNAreaGenerator(FNNNavMeshGenerator* InParentGenerator, const FNavigationBounds& Bounds)
	: AreaBounds(Bounds), ParentGenerator(InParentGenerator)
{
}

void FNNAreaGenerator::DoWork()
{
	check(ParentGenerator);

	if (AreaGeneratorData)
	{
		delete AreaGeneratorData;
	}
	AreaGeneratorData = new FNNAreaGeneratorData();

	GatherGeometry(true);

	const TWeakObjectPtr<ANNNavMesh> NavMesh = ParentGenerator->GetOwner();
	const float HeightFieldHeight = NavMesh->CellHeight; // Z Axis
	const float HeightFieldSize = NavMesh->CellSize; // X and Y Axis

	const FVector& MinimumPoint = AreaBounds.AreaBox.Min;
	const FVector& MaximumPoint = AreaBounds.AreaBox.Max;

	const FHeightFieldGenerator Generator (*AreaGeneratorData);
	AreaGeneratorData->HeightField = Generator.InitializeHeightField(AreaGeneratorData->RawGeometry, MinimumPoint, MaximumPoint, HeightFieldSize, HeightFieldHeight, NavMesh->WalkableSlopeDegrees, NavMesh->AgentHeight, NavMesh->MaxLedgeHeight);
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
