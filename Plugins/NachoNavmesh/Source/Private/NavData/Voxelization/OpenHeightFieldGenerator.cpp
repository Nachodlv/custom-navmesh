#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

// NN Includes
#include "NavData/Voxelization/HeightFieldGenerator.h"

#define DEBUG_OPENHEIGHTFIELD 0

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

FNNOpenHeightField::FNNOpenHeightField(int32 InUnitsWidth, int32 InUnitsDepth, int32 InUnitsHeight)
	: UnitsWidth(InUnitsWidth), UnitsDepth(InUnitsDepth), UnitsHeight(InUnitsHeight)
{
	const int32 Num = UnitsWidth * UnitsDepth;
	Spans.Reserve(Num);
	for (int32 i = 0; i < Num; ++i)
	{
		Spans.Add(nullptr);
	}
}

void FOpenHeightFieldGenerator::GenerateOpenHeightField(FNNOpenHeightField& OutOpenHeightField, const FNNHeightField& SolidHeightField, float MaxLedgeHeight, float AgentHeight, float MinRegionSize) const
{
	OutOpenHeightField = FNNOpenHeightField(SolidHeightField.UnitsWidth, SolidHeightField.UnitsDepth, SolidHeightField.UnitsHeight);
	OutOpenHeightField.CellHeight = SolidHeightField.CellHeight;
	OutOpenHeightField.CellSize = SolidHeightField.CellSize;
	OutOpenHeightField.Bounds = FBox(SolidHeightField.MinPoint, SolidHeightField.MaxPoint);

	// Create the open spans
	for (int32 i = 0; i < SolidHeightField.Spans.Num(); ++i)
	{
		const int32 X = (i % SolidHeightField.UnitsWidth);
		const int32 Y = (i / SolidHeightField.UnitsWidth);
		const int32 Index = X + Y * SolidHeightField.UnitsWidth;
		FNNOpenSpan* LastOpenSpan = nullptr;
		Span* Span = SolidHeightField.Spans[i].Get();
		while (Span)
		{
			if (Span->bWalkable)
			{
				const int32 MinHeight = Span->MaxSpanHeight;
				const int32 MaxHeight = Span->NextSpan ? Span->NextSpan->MinSpanHeight : TNumericLimits<int32>::Max();
				if (LastOpenSpan)
				{
					LastOpenSpan->NextOpenSpan = MakeUnique<FNNOpenSpan>(MinHeight, MaxHeight, X, Y);
					LastOpenSpan = LastOpenSpan->NextOpenSpan.Get();
				}
				else
				{
					OutOpenHeightField.Spans[Index] = MakeUnique<FNNOpenSpan>(MinHeight, MaxHeight, X, Y);
					LastOpenSpan = OutOpenHeightField.Spans[Index].Get();
				}
			}
			Span = Span->NextSpan.Get();
		}
	}

	// Add linked neighbours to the open spans
	const TArray<FVector2D> PossibleNeighbours = {FVector2D(-1, 0), FVector2D(0, 1), FVector2D(1, 0), FVector2D(0, -1)};
	for (int32 i = 0; i < OutOpenHeightField.Spans.Num(); ++i)
	{
		FNNOpenSpan* OpenSpan = OutOpenHeightField.Spans[i].Get();
		while (OpenSpan)
		{
			SetOpenSpanNeighbours(OutOpenHeightField, PossibleNeighbours, OpenSpan, MaxLedgeHeight, AgentHeight);
			OpenSpan = OpenSpan->NextOpenSpan.Get();
		}
	}

	int32 MaxEdgeDistance = INDEX_NONE;
	// Calculate distance to nearest edge
	for (int32 i = 0; i < OutOpenHeightField.Spans.Num(); ++i)
	{
		FNNOpenSpan* OpenSpan = OutOpenHeightField.Spans[i].Get();
		while (OpenSpan)
		{
			TArray<const FNNOpenSpan*> VisitedSpans;
			OpenSpan->EdgeDistance = GetOpenSpanEdgeDistance(OpenSpan);
			MaxEdgeDistance = FMath::Max(OpenSpan->EdgeDistance, MaxEdgeDistance);

#if DEBUG_OPENHEIGHTFIELD
			// Debug distances
			AreaGeneratorData.AddDebugText(GetOpenSpanWorldPosition(OpenSpan, OutOpenHeightField), FString::FromInt(OpenSpan->EdgeDistance));
#endif // DEBUG_OPENHEIGHTFIELD_DISTANCE
			OpenSpan = OpenSpan->NextOpenSpan.Get();
		}
	}
	OutOpenHeightField.SpanMaxEdgeDistance = MaxEdgeDistance;


	const int32 MinSpansQuantity = FMath::CeilToInt(MinRegionSize / OutOpenHeightField.CellSize);
	CreateRegions(OutOpenHeightField, MinSpansQuantity);
}

void FOpenHeightFieldGenerator::SetOpenSpanNeighbours(FNNOpenHeightField& OutOpenHeightField, const TArray<FVector2D>& PossibleNeighbours, FNNOpenSpan* OpenSpan, float MaxLedgeHeight, float AgentHeight) const
{
	for (int32 NeighbourIndex = 0; NeighbourIndex < PossibleNeighbours.Num(); ++NeighbourIndex)
	{
		const FVector2D& Neighbour = PossibleNeighbours[NeighbourIndex];
		const int32 X = OpenSpan->X + Neighbour.X;
		const int32 Y = OpenSpan->Y + Neighbour.Y;
		FNNOpenSpan* NeighbourSpan = OutOpenHeightField.Spans[X + Y * OutOpenHeightField.UnitsWidth].Get();
		FNNOpenSpan* NearestNeighbourSpan = nullptr;
		int32 NearestDistance = INDEX_NONE;
		while (NeighbourSpan)
		{
			// Head Bonk Test
			bool bWillHeadBonk = false;
			const int32 SpaceBetween = FMath::Abs(NeighbourSpan->MaxHeight - OpenSpan->MinHeight);
			if (SpaceBetween * OutOpenHeightField.CellHeight < AgentHeight)
			{
				bWillHeadBonk = true;
			}

			if (!bWillHeadBonk)
			{
				const int32 Distance = FMath::Abs(NeighbourSpan->MinHeight - OpenSpan->MinHeight);
				if (Distance * OutOpenHeightField.CellHeight < MaxLedgeHeight)
				{
					if ((NearestDistance == INDEX_NONE || Distance < NearestDistance))
					{
						NearestDistance = Distance;
						NearestNeighbourSpan = NeighbourSpan;
					}
					else
					{
						// The distance will only continue to increase
						break;
					}
				}
			}

			NeighbourSpan = NeighbourSpan->NextOpenSpan.Get();
		}
		OpenSpan->Neighbours[NeighbourIndex] = NearestNeighbourSpan;

#if DEBUG_OPENHEIGHTFIELD
		// Debug neighbours
		if (NearestNeighbourSpan)
		{
			FVector CurrentSpanLocation = GetOpenSpanWorldPosition(OpenSpan, OutOpenHeightField);
			FVector NeighbourLocation = GetOpenSpanWorldPosition(NearestNeighbourSpan, OutOpenHeightField);
			AreaGeneratorData.AddDebugLine(CurrentSpanLocation, NeighbourLocation);
		}
#endif // DEBUG_OPENHEIGHTFIELD_DISTANCE
	}
}

void FOpenHeightFieldGenerator::CreateRegions(FNNOpenHeightField& OpenHeightField, int32 MinSpansForRegions) const
{
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
					FNNRegion& NewRegion = Regions.Emplace_GetRef(FNNRegion(Regions.Num() + 1));
					FloodRegion(OpenSpan, NewRegion, WaterLevel);
				}
				OpenSpan = OpenSpan->NextOpenSpan.Get();
			}
		}
	}

	FilterSmallRegions(Regions, MinSpansForRegions);
}

void FOpenHeightFieldGenerator::GrowRegions(FNNOpenHeightField& OpenHeightField, int32 CurrentWaterLevel) const
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

void FOpenHeightFieldGenerator::FilterSmallRegions(TArray<FNNRegion>& Regions, int32 MinSpansForRegions) const
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

		// TODO (ignacio) fix me
	}
}

void FOpenHeightFieldGenerator::FloodRegion(FNNOpenSpan* CurrentSpan, FNNRegion& Region, int32 CurrentWaterLevel) const
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

int32 FOpenHeightFieldGenerator::GetOpenSpanEdgeDistance(const FNNOpenSpan* OpenSpan) const
{
	TArray<const FNNOpenSpan*> VisitedSpans;
	TArray<const FNNOpenSpan*> OpenSpans;
	TMap<const FNNOpenSpan*, int32> Distances;
	int32 NearestEdge = TNumericLimits<int32>::Max();
	Distances.Add(OpenSpan, 0);
	OpenSpans.Add(OpenSpan);
	while (OpenSpans.Num() > 0)
	{
		const FNNOpenSpan* CurrentSpan = OpenSpans[0];
		OpenSpans.RemoveAt(0);
		VisitedSpans.Add(CurrentSpan);
		const int32 CurrentDistance = Distances[CurrentSpan];

		// The result for this span has already been calculated
		if (CurrentSpan->EdgeDistance != INDEX_NONE)
		{
			NearestEdge = FMath::Min(CurrentDistance + CurrentSpan->EdgeDistance, NearestEdge);
			continue;
		}

		const int32 NewDistance = CurrentDistance + 1;
		const bool bSearchNeighbours = NewDistance < NearestEdge;
		for (const FNNOpenSpan* Neighbour : CurrentSpan->Neighbours)
		{
			// It's an edge!
			// TODO (ignacio) should i add the edge into the open spans?
			if (!Neighbour)
			{
				NearestEdge = FMath::Min(NewDistance, NearestEdge);
			}
			else if (bSearchNeighbours && !VisitedSpans.Contains(Neighbour))
			{
				Distances.Add(Neighbour, NewDistance);
				OpenSpans.Add(Neighbour);
			}
		}
	}

	return NearestEdge;

}

FVector FOpenHeightFieldGenerator::GetOpenSpanWorldPosition(const FNNOpenSpan* OpenSpan, const FNNOpenHeightField& OpenHeightField)
{
	const float X = OpenSpan->X * OpenHeightField.CellSize + OpenHeightField.CellSize / 2;
	const float Y = OpenSpan->Y * OpenHeightField.CellSize + OpenHeightField.CellSize / 2;
	const float Z = OpenSpan->MinHeight * OpenHeightField.CellHeight;
	return OpenHeightField.Bounds.Min + FVector(X, Y, Z);
}


