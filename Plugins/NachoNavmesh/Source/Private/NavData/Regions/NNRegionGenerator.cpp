#include "NavData/Regions/NNRegionGenerator.h"

#include "NavData/Regions/CleanNullRegionBorders.h"
#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

namespace
{
	void GetNeighboursToFlood(FNNOpenSpan* Span, int32 CurrentWaterLevel, TArray<FNNOpenSpan*>& SpansToFlood)
	{
		for (FNNOpenSpan* Neighbour : Span->Neighbours)
		{
			if (Neighbour && Neighbour->RegionID == INDEX_NONE && Neighbour->EdgeDistance == CurrentWaterLevel)
			{
				SpansToFlood.Add(Neighbour);
			}
		}
	}
}

void FNNRegionGenerator::CreateRegions(FNNOpenHeightField& OpenHeightField, float MinRegionSize) const
{
	const int32 MinSpansForRegions = FMath::CeilToInt(MinRegionSize / OpenHeightField.CellSize);

	TArray<FNNRegion>& Regions = OpenHeightField.Regions;
	for (int32 WaterLevel = OpenHeightField.SpanMaxEdgeDistance; WaterLevel > 0; --WaterLevel)
	{
		// Grow regions
		GrowRegions(OpenHeightField, WaterLevel);

		// Create new regions
		for (int32 i = 0; i < OpenHeightField.Spans.Num(); ++i)
		{
			FNNOpenSpan* OpenSpan = OpenHeightField.Spans[i].Get();
			while (OpenSpan)
			{
				if (OpenSpan->EdgeDistance == WaterLevel && OpenSpan->RegionID == INDEX_NONE)
				{
					FNNRegion& NewRegion = Regions.Emplace_GetRef(FNNRegion::GenerateNewRegion());
					FloodRegion(OpenSpan, NewRegion, WaterLevel);
				}
				OpenSpan = OpenSpan->NextOpenSpan.Get();
			}
		}
	}

	FilterSmallRegions(Regions, MinSpansForRegions);
	FNNCleanNullRegionBorders CleanNullRegionBorders (OpenHeightField);
	CleanNullRegionBorders.CleanNullRegionBorders();
}

void FNNRegionGenerator::GrowRegions(FNNOpenHeightField& OpenHeightField, int32 CurrentWaterLevel) const
{
	TArray<FNNRegion>& Regions = OpenHeightField.Regions;
	TMap<int32, TArray<FNNOpenSpan*>> SpansToFloodByRegion;

	for (int32 i = 0; i < Regions.Num(); ++i)
	{
		FNNRegion& Region = Regions[i];
		TArray<FNNOpenSpan*> SpansToFlood;
		for (FNNOpenSpan* Span : Region.Spans)
		{
			// Was the Span just flooded?
			if (Span->EdgeDistance == CurrentWaterLevel + 1)
			{
				TArray<FNNOpenSpan*> NeighboursToFlood;
				GetNeighboursToFlood(Span, CurrentWaterLevel, NeighboursToFlood);
				SpansToFlood.Append(NeighboursToFlood);
			}
		}
		SpansToFloodByRegion.Add(Region.ID, SpansToFlood);
	}

	while (SpansToFloodByRegion.Num() > 0)
	{
		// We fill the Regions equally
		for (FNNRegion& Region : Regions)
		{
			TArray<FNNOpenSpan*>* SpansToFloodPointer = SpansToFloodByRegion.Find(Region.ID);
			if (!SpansToFloodPointer)
			{
				continue;
			}

			TArray<FNNOpenSpan*> NextSpansToFlood;
			for (FNNOpenSpan* Span : *SpansToFloodPointer)
			{
				if (Span->RegionID == INDEX_NONE)
				{
					Span->RegionID = Region.ID;
					Region.Spans.Add(Span);
					TArray<FNNOpenSpan*> NeighboursToFlood;
					GetNeighboursToFlood(Span, CurrentWaterLevel, NeighboursToFlood);
					NextSpansToFlood.Append(NeighboursToFlood);
				}
			}
			if (NextSpansToFlood.Num() == 0)
			{
				SpansToFloodByRegion.Remove(Region.ID);
			}
			else
			{
				SpansToFloodByRegion.Add(Region.ID, NextSpansToFlood);
			}
		}
	}
}

void FNNRegionGenerator::FilterSmallRegions(TArray<FNNRegion>& Regions, int32 MinSpansForRegions) const
{
	// Seems faster than saving the index in the spans and then remapping them
	TMap<int32, int32> RegionIndexByRegionID;
	for (int32 i = 0; i < Regions.Num(); ++i)
	{
		RegionIndexByRegionID.Add(Regions[i].ID, i);
	}

	for (int32 i = Regions.Num() -1; i >= 0; --i)
	{
		FNNRegion& CurrentRegion = Regions[i];
		// TODO (ignacio) if we are going to access the region neighbours in another place we might want to cache them in the region
		if (CurrentRegion.Spans.Num() > MinSpansForRegions)
		{
			continue;
		}
		// We will need to merge it with another region or eliminate it
		int32 SmallestRegionIndex = INDEX_NONE;
		int32 SmallestRegionSpansQuantity = TNumericLimits<int32>::Max();
		for (FNNOpenSpan* Span : CurrentRegion.Spans)
		{
			for (FNNOpenSpan* Neighbour : Span->Neighbours)
			{
				if (Neighbour && Neighbour->RegionID != CurrentRegion.ID)
				{
					const int32 RegionIndex = *RegionIndexByRegionID.Find(Neighbour->RegionID);
					FNNRegion& NeighbourRegion = Regions[RegionIndex];
					if (NeighbourRegion.Spans.Num() < SmallestRegionSpansQuantity)
					{
						SmallestRegionSpansQuantity = NeighbourRegion.Spans.Num();
						SmallestRegionIndex = RegionIndex;
					}
				}
			}
		}

		for (FNNOpenSpan* Span : CurrentRegion.Spans)
		{
			if (SmallestRegionIndex == INDEX_NONE)
			{
				Span->RegionID = INDEX_NONE;
			}
			else
			{
				FNNRegion& NewRegion = Regions[SmallestRegionIndex];
				Span->RegionID = NewRegion.ID;
				NewRegion.Spans.Add(Span);
			}
		}
		RegionIndexByRegionID.Remove(Regions[i].ID);
		Regions.RemoveAt(i);

		// Remap RegionIndexByRegionID
		for (int32 RemapIndex = i; RemapIndex < Regions.Num(); ++RemapIndex)
		{
			RegionIndexByRegionID[Regions[RemapIndex].ID] = RemapIndex;
		}
	}
}

void FNNRegionGenerator::FloodRegion(FNNOpenSpan* CurrentSpan, FNNRegion& Region, int32 CurrentWaterLevel) const
{
	Region.Spans.Add(CurrentSpan);
	CurrentSpan->RegionID = Region.ID;
	for (FNNOpenSpan* Neighbour : CurrentSpan->Neighbours)
	{
		if (Neighbour && Neighbour->RegionID == INDEX_NONE && Neighbour->EdgeDistance == CurrentWaterLevel)
		{
			FloodRegion(Neighbour, Region, CurrentWaterLevel);
		}
	}
}
