#include "NavData/NNAreaGenerator.h"

// UE Includes
#include "NavigationSystem.h"

// NN Includes
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

Span::~Span()
{
	if (NextSpan)
	{
		delete NextSpan;
	}
}

HeightField::HeightField(int32 InUnitsWidth, int32 InUnitsHeight, int32 InUnitsDepth)
	: UnitsWidth(InUnitsWidth), UnitsHeight(InUnitsHeight), UnitsDepth(InUnitsDepth)
{
	Spans.Init(nullptr, InUnitsWidth * InUnitsDepth);
}

HeightField::~HeightField()
{
	Spans.Reset();
}

FNNAreaGeneratorData::~FNNAreaGeneratorData()
{
	delete HeightField;
}

FNNAreaGenerator::FNNAreaGenerator(FNNNavMeshGenerator* InParentGenerator, const FNavigationBounds& Bounds)
	: AreaBounds(Bounds), ParentGenerator(InParentGenerator)
{
}

void FNNAreaGenerator::DoWork()
{
	check(ParentGenerator);

	if (!AreaGeneratorData)
	{
		AreaGeneratorData = new FNNAreaGeneratorData();
	}

	GatherGeometry(true);

	constexpr float HeightFieldHeight = 200.0f; // Z Axis
	constexpr float HeightFieldSize = 200.0f; // X and Y Axis

	const FVector& MinimumPoint = AreaBounds.AreaBox.Min;
	const FVector& MaximumPoint = AreaBounds.AreaBox.Max;

	AreaGeneratorData->HeightField = InitializeHeightField(MinimumPoint, MaximumPoint, HeightFieldSize, HeightFieldHeight);
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

HeightField* FNNAreaGenerator::InitializeHeightField(const FVector& MinPoint, const FVector& MaxPoint, float CellSize, float CellHeight) const
{
	// https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
	// Clip polygons in heightfields

	const int32 XHeightFieldNum = FMath::CeilToInt((MaxPoint.X - MinPoint.X) / CellSize);
	const int32 YHeightFieldNum = FMath::CeilToInt((MaxPoint.Y - MinPoint.Y) / CellSize);
	const int32 ZHeightFieldNum = FMath::CeilToInt((MaxPoint.Z - MinPoint.Z) / CellHeight);

	HeightField* Field = new HeightField(XHeightFieldNum, ZHeightFieldNum, YHeightFieldNum);
	Field->CellHeight = CellHeight;
	Field->CellSize = CellSize;
	Field->MaxPoint = MaxPoint;
	Field->MinPoint = MinPoint;

	for (int32 i = 0; i < ZHeightFieldNum; ++i)
	{
		for (int32 j = 0; j < YHeightFieldNum; ++j)
		{
			for (int32 k = 0; k < XHeightFieldNum; ++k)
			{
				// We should only create span if intersects with a polygon
				Span* NewSpan = new Span();
				Span* BelowSpawn = Field->Spans[k + j * XHeightFieldNum];
				UE_LOG(LogTemp, Warning, TEXT("Index: %d"), k + j * XHeightFieldNum);
				if (!BelowSpawn)
				{
					 Field->Spans[k + j * XHeightFieldNum] = NewSpan;
				}
				else
				{
					while (BelowSpawn->NextSpan)
					{
						BelowSpawn = BelowSpawn->NextSpan;
					}
					BelowSpawn->NextSpan = NewSpan;
				}
			}
		}
	}
	return Field;
}
