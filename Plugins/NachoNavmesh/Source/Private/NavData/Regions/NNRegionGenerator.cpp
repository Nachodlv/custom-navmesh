#include "NavData/Regions/NNRegionGenerator.h"

#include "NavData/Regions/CleanNullRegionBorders.h"
#include "NavData/Voxelization/OpenHeightFieldGenerator.h"


void FNNRegionGenerator::CreateRegions(FNNOpenHeightField& OpenHeightField, float MinRegionSize, int32 TraversableAreaBorderSize) const
{
	const int32 MinDist = TraversableAreaBorderSize + OpenHeightField.GetSpanMinEdgeDistance();
	const int32 ExpandIterations = 4 + (TraversableAreaBorderSize * 2); // ???

	// The current distance from the border that are we currently searching
	// The distance starts at the maximum distance and moves towards 0
	// It should always be divisible by 2
	int32 Dist = (OpenHeightField.GetSpanMaxEdgeDistance()) & ~1;

	// These spans are flooded and ready to be processed
	TArray<FNNOpenSpan*> FloodedSpans;
	FloodedSpans.Reserve(1024);

	TArray<FNNOpenSpan*> WorkingStack;
	WorkingStack.Reserve(1024);

	FNNRegion NewRegion = FNNRegion::GenerateNewRegion();
	TMap<int32, FNNRegion> RegionsByID;
	RegionsByID.Add(NewRegion.ID, NewRegion);

	// Iterates until the distance reached the minimum allowed distance
	while (Dist > MinDist)
	{
		FloodedSpans.Reset();
		// Finds all the spans that are below the current "water level" and don't have a region assigned
		for (FNNOpenHeightFieldIterator It (OpenHeightField); It; ++It)
		{
			FNNOpenSpan* CurrentSpan = It.Get();
			if (CurrentSpan->RegionID == INDEX_NONE && CurrentSpan->EdgeDistance >= Dist)
			{
				FloodedSpans.Add(CurrentSpan);
			}
		}

		if (RegionsByID.Num() > 1)
		{
			ExpandRegions(RegionsByID, FloodedSpans, Dist > 0 ? ExpandIterations : -1);
		}

		for (FNNOpenSpan* FloodedSpan : FloodedSpans)
		{
			if (!FloodedSpan || FloodedSpan->RegionID != INDEX_NONE)
			{
				continue;
			}

			// Fill slightly more than the current "water level". This should improve the efficiency of the algorithm
			const int32 FillTo = FMath::Max(Dist - 2, MinDist);
			if (FloodNewRegion(FloodedSpan, FillTo, WorkingStack, NewRegion))
			{
				RegionsByID.Add(NewRegion.ID, MoveTemp(NewRegion));
				NewRegion = FNNRegion::GenerateNewRegion();
			}
		}

		Dist = FMath::Max(Dist - 2, 0);
	}

	// Find all the spans remaining without region
	FloodedSpans.Reset();
	for (FNNOpenHeightFieldIterator It (OpenHeightField); It; ++It)
	{
		if (It->EdgeDistance >= MinDist && It->RegionID == INDEX_NONE)
		{
			FloodedSpans.Add(*It);
		}
	}

	// Perform a final region expansion. Allow more iterations than the previous ones
	ExpandRegions(RegionsByID, FloodedSpans, MinDist > 0 ? ExpandIterations * 8 : -1);

	OpenHeightField.Regions.Reserve(RegionsByID.Num());
	 for (auto& RegionByID : RegionsByID)
	 {
		 OpenHeightField.Regions.Add(MoveTemp(RegionByID.Value));
	 }

	const int32 MinSpansForRegions = FMath::CeilToInt(MinRegionSize / OpenHeightField.CellSize);
	// TODO (ignacio) we are creating the region mapping inside this function
	FilterSmallRegions(OpenHeightField.Regions, MinSpansForRegions);
	FNNCleanNullRegionBorders CleanNullRegionBorders (OpenHeightField);
	CleanNullRegionBorders.CleanNullRegionBorders();
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
				if (Neighbour && Neighbour->RegionID != INDEX_NONE && Neighbour->RegionID != CurrentRegion.ID)
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

bool FNNRegionGenerator::FloodNewRegion(FNNOpenSpan* RootSpan, int32 FillToDistance, TArray<FNNOpenSpan*>& WorkingStack, FNNRegion& NewRegion) const
{
	WorkingStack.Reset();

	WorkingStack.Add(RootSpan);
	NewRegion.Spans.Add(RootSpan);
	RootSpan->RegionID = NewRegion.ID;
	RootSpan->DistanceToCore = 0;

	int32 RegionSize = 0;
	while (WorkingStack.Num() > 0)
	{
		FNNOpenSpan* Span = WorkingStack.Pop();

		bool bOnRegionBorder = false;
		for (int32 Dir = 0; Dir < 4; ++Dir)
		{
			FNNOpenSpan* Neighbour = Span->Neighbours[Dir];
			if (!Neighbour)
			{
				continue;
			}

			if (Neighbour->RegionID != INDEX_NONE && Neighbour->RegionID != NewRegion.ID)
			{
				bOnRegionBorder = true;
				break;
			}

			// Check the diagonal neighbour
			Neighbour = Neighbour->Neighbours[(Dir + 1) % 4];
			if (Neighbour && Neighbour->RegionID != INDEX_NONE && Neighbour->RegionID != NewRegion.ID)
			{
				bOnRegionBorder = true;
				break;
			}
		}

		if (bOnRegionBorder)
		{
			NewRegion.Spans.Remove(Span);
			Span->RegionID = INDEX_NONE;
			continue;
		}
		++RegionSize;

		// The new span is on the region. Checks if any of its neighbours should also be assigned to the new region
		for (int32 Dir = 0; Dir < 4; ++Dir)
		{
			FNNOpenSpan* Neighbour = Span->Neighbours[Dir];

			if (Neighbour && Neighbour->EdgeDistance >= FillToDistance && Neighbour->RegionID == INDEX_NONE)
			{
				Neighbour->RegionID = NewRegion.ID;
				Neighbour->DistanceToCore = 0;
				NewRegion.Spans.Add(Neighbour);
				WorkingStack.Add(Neighbour);
			}
		}
	}

	return RegionSize > 0;
}

void FNNRegionGenerator::ExpandRegions(TMap<int32, FNNRegion>& RegionsByID, TArray<FNNOpenSpan*>& Spans, int32 MaxIterations) const
{
	if (Spans.Num() == 0)
	{
		return;
	}

	int32 IterationCount = 0;
	while (true)
	{
		int32 Skipped = 0;

		for (int32 i = 0; i < Spans.Num(); ++i)
		{
			FNNOpenSpan* Span = Spans[i];
			if (!Span)
			{
				++Skipped;
				continue;
			}

			int32 SpanRegion = INDEX_NONE;
			int32 RegionCenterDistance = INT32_MAX;
			for (int32 Dir = 0; Dir < 4; ++Dir)
			{
				FNNOpenSpan* Neighbour = Span->Neighbours[Dir];
				if (!Neighbour)
				{
					continue;
				}
				if (Neighbour->RegionID != INDEX_NONE)
				{
					if (Neighbour->DistanceToCore + 2 < RegionCenterDistance)
					{
						int32 SameRegionCount = 0;
						// Check if this neighbour has at least two other neighbours in its region
						// To avoid a single width line of voxels
						for (int32 NDir = 0; NDir < 4; ++NDir)
						{
							FNNOpenSpan* NNSpan = Neighbour->Neighbours[NDir];
							if (!NNSpan)
							{
								continue;
							}

							if (NNSpan->RegionID == Neighbour->RegionID)
							{
								++SameRegionCount;
							}
						}
						if (SameRegionCount > 1)
						{
							// Choose this neighbour region
							// Sets the distance to center a slightly further away than this neighbour
							SpanRegion = Neighbour->RegionID;
							RegionCenterDistance = Neighbour->DistanceToCore + 2;
						}
					}
				}
			}
			if (SpanRegion != INDEX_NONE)
			{
				RegionsByID[SpanRegion].Spans.Add(Span);
				Span->RegionID = SpanRegion;
				Span->DistanceToCore = RegionCenterDistance;
				Spans[i] = nullptr;
			}
			else
			{
				++Skipped;
			}
		}

		// All spans have been processed
		if (Skipped == Spans.Num())
		{
			break;
		}

		++IterationCount;
		if (MaxIterations > 0 && IterationCount > MaxIterations)
		{
			// Reached the iteration limit
			break;
		}
	}
}
