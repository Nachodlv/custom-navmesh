#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

// NN Includes
#include "NavData/Voxelization/HeightFieldGenerator.h"

#define DEBUG_OPENHEIGHTFIELD 0


TArray<FNNOpenSpan*> FNNOpenSpan::GetDetailedNeighbours() const
{
	TArray<FNNOpenSpan*> DetailedNeighbours;
	DetailedNeighbours.Init(nullptr, 8);
	for (int32 i = 0; i < Neighbours.Num(); ++i)
	{
		FNNOpenSpan* Neighbour = Neighbours[i];
		if (Neighbour)
		{
			DetailedNeighbours[i] = Neighbour;
			DetailedNeighbours[i + 4] = Neighbour->Neighbours[(i + 1) % 4];
			DetailedNeighbours[((i + 3) % 4) + 4] = Neighbour->Neighbours[(i + 3)  % 4];
		}
	}
	return DetailedNeighbours;
}

FVector FNNOpenSpan::GetOpenSpanWorldPosition(const FNNOpenHeightField& OpenHeightField) const
{
	const float WorldX = X * OpenHeightField.CellSize + OpenHeightField.CellSize / 2;
	const float WorldY = Y * OpenHeightField.CellSize + OpenHeightField.CellSize / 2;
	const float WorldZ = MinHeight * OpenHeightField.CellHeight;
	return OpenHeightField.Bounds.Min + FVector(WorldX, WorldY, WorldZ);
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

int32 FNNOpenHeightField::GetRegionIndexByID(int32 ID) const
{
	return Regions.IndexOfByPredicate([ID](const FNNRegion& Region) { return Region.ID == ID; });
}

FNNRegion FNNRegion::GenerateNewRegion()
{
	static int32 UniqueIdentifier = 0;
	FNNRegion Region (UniqueIdentifier);
	++UniqueIdentifier;
	return Region;
}

void FOpenHeightFieldGenerator::GenerateOpenHeightField(FNNOpenHeightField& OutOpenHeightField, const FNNHeightField& SolidHeightField, float MaxLedgeHeight, float AgentHeight) const
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
				++OutOpenHeightField.AmountOfSpans;
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
			const FVector OpenSpanPosition = OpenSpan->GetOpenSpanWorldPosition(OutOpenHeightField);
			AreaGeneratorData.AddDebugText(OpenSpanPosition, FString::FromInt(OpenSpan->EdgeDistance));
#endif // DEBUG_OPENHEIGHTFIELD_DISTANCE
			OpenSpan = OpenSpan->NextOpenSpan.Get();
		}
	}
	OutOpenHeightField.SpanMaxEdgeDistance = MaxEdgeDistance;
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
			int32 SpaceBetween = FMath::Abs(NeighbourSpan->MaxHeight - OpenSpan->MinHeight);
			// We need to check both ways
			SpaceBetween = FMath::Min(SpaceBetween, FMath::Abs(OpenSpan->MaxHeight - NeighbourSpan->MinHeight));
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
			FVector CurrentSpanLocation = OpenSpan->GetOpenSpanWorldPosition(OutOpenHeightField);
			FVector NeighbourLocation =NearestNeighbourSpan-> GetOpenSpanWorldPosition(OutOpenHeightField);
			AreaGeneratorData.AddDebugLine(CurrentSpanLocation, NeighbourLocation);
		}
#endif // DEBUG_OPENHEIGHTFIELD_DISTANCE
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



