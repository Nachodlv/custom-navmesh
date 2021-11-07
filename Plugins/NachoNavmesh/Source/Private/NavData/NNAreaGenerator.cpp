#include "NavData/NNAreaGenerator.h"

// UE Includes
#include "NavigationSystem.h"

// NN Includes
#include "NavData/NNNavMeshGenerator.h"

FNNGeometryCache::FNNGeometryCache(const uint8* Memory)
{
	Header = *((FHeader*)Memory);
	Verts = (float*)(Memory + sizeof(FNNGeometryCache));
	Indices = (int32*)(Memory + sizeof(FNNGeometryCache) + (sizeof(float) * Header.NumVerts * 3));
}


FNNAreaGenerator::FNNAreaGenerator(FNNNavMeshGenerator* InParentGenerator, const FNavigationBounds& Bounds)
	: AreaBounds(Bounds), ParentGenerator(InParentGenerator)
{
}

void FNNAreaGenerator::DoWork()
{
	check(ParentGenerator);
	GatherGeometry(true);
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

		AreaGeneratorData.RawGeometry.Add(MoveTemp(GeometryElement));
	}
}
