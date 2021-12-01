#include "NavData/Regions/CleanNullRegionBorders.h"

// NN Includes
#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

void FNNCleanNullRegionBorders::CleanNullRegionBorders()
{
	for (TUniquePtr<FNNOpenSpan>& OpenSpan : OpenHeightField.Spans)
	{
		for (FNNOpenSpan* CurrentSpan = OpenSpan.Get(); CurrentSpan; CurrentSpan = CurrentSpan->NextOpenSpan.Get())
		{
			if (CurrentSpan->Flags & ENNOpenSpanFlags::NullRegionChecked || CurrentSpan->RegionID != INDEX_NONE)
			{
				continue;
			}
			CurrentSpan->Flags |= ENNOpenSpanFlags::NullRegionChecked;

			int32 EdgeDirection = GetNonNullBorderDirection(*CurrentSpan);
			if (EdgeDirection == INDEX_NONE)
			{
				continue;
			}
			FNNOpenSpan* WorkingSpan = CurrentSpan->Neighbours[EdgeDirection];
			EdgeDirection = (EdgeDirection + 2) % 4;

			const bool bEncompassedNullRegion = ProcessNullRegion(*WorkingSpan, EdgeDirection);
			if (bEncompassedNullRegion)
			{
				const int32 RegionIndex = OpenHeightField.GetRegionIndexByID(WorkingSpan->RegionID);
				FNNRegion& NewRegion = OpenHeightField.Regions.Emplace_GetRef(FNNRegion::GenerateNewRegion());
				PartialFloodRegion(*WorkingSpan, EdgeDirection, OpenHeightField.Regions[RegionIndex], NewRegion);
			}
		}
	}
}

int32 FNNCleanNullRegionBorders::GetNonNullBorderDirection(const FNNOpenSpan& OpenSpan)
{
	for (int32 i = 0; i < OpenSpan.Neighbours.Num(); ++i)
	{
		const FNNOpenSpan* Neighbour = OpenSpan.Neighbours[i];
		if (Neighbour && Neighbour->RegionID != INDEX_NONE)
		{
			return i;
		}
	}
	return INDEX_NONE;
}

bool FNNCleanNullRegionBorders::ProcessNullRegion(FNNOpenSpan& StartSpan, int32 StartDirection)
{
	// Travers the region contour
	const int32 BorderRegionID = StartSpan.RegionID;
	FNNOpenSpan* Span = &StartSpan;
	int32 Direction = StartDirection;

	// 90 degrees turn
	int32 AcuteCornerCount = 0;
	// 180 degrees turn
	int32 ObtuseCornerCount = 0;
	int32 StepsWithoutBorder = 0;
	bool bBorderSeenLastLoop = false;
	bool bBorder;

	bool bSingleConnection = true;

	constexpr int32 MaximumIterations = TNumericLimits<int32>::Max();
	int32 Iteration = 0;
	while (Iteration < MaximumIterations)
	{
		++Iteration;

		FNNOpenSpan* NSpan = Span->Neighbours[Direction];
		if (!NSpan)
		{
			bBorder = true;
		}
		else
		{
			NSpan->Flags |= ENNOpenSpanFlags::NullRegionChecked;
			if (NSpan->RegionID == INDEX_NONE)
			{
				bBorder = true;
			}
			else
			{
				bBorder = false;
				if (NSpan->RegionID != BorderRegionID)
				{
					// It borders another region. The contour cant represent a full encompassed null region
					bSingleConnection = false;
				}
			}
		}
		if (bBorder)
		{
			if (bBorderSeenLastLoop)
			{
				// Two detections in a row indicates we passed an acute (inner) corner
				++AcuteCornerCount;
			}
			else if (StepsWithoutBorder > 1)
			{
				// We moved at least two spans before detecting a border. This indicates a obtuse (outer) corner
				++ObtuseCornerCount;
				if (ProcessOuterCorner(*Span, Direction))
				{
					bSingleConnection = false;
				}
			}
			Direction  = (Direction + 1) % 4; // Rotate clockwise
			bBorderSeenLastLoop = true;
			StepsWithoutBorder = 0;
		}
		else
		{
			// Not a null region border.
			// Rotate the direction counter clockwise
			Span = NSpan;
			Direction = (Direction + 3) % 4;
			bBorderSeenLastLoop = false;
			++StepsWithoutBorder;
		}
		// Have we returned to the original span? The search is complete
		if (StartSpan == *Span && StartDirection == Direction)
		{
			// Is the null region inside the contour?
			return bSingleConnection && ObtuseCornerCount > AcuteCornerCount;
		}
	}
	ensureAlways(false);
	return false;
}

bool FNNCleanNullRegionBorders::ProcessOuterCorner(FNNOpenSpan& CurrentSpan, int32 BorderDirection)
{
	bool bMultiRegions = false;
	// Get the previous two spans along the border
	FNNOpenSpan* BackOne = CurrentSpan.Neighbours[(BorderDirection + 3) % 4];
	const int32 BackOneRegionID = BackOne ? BackOne->RegionID : INDEX_NONE;
	FNNOpenSpan* BackTwo = CurrentSpan.Neighbours[BorderDirection];
	const int32 BackTwoRegionID = BackTwo ? BackTwo->RegionID : INDEX_NONE;
	if (BackOneRegionID != CurrentSpan.RegionID && BackTwoRegionID == CurrentSpan.RegionID)
	{
		/*
		* Dangerous corner configuration.
		*
		*     a x
		*     b a
		*
		* Need to change to one of the following configurations:
		*
		*     b x        a x
		*     b a        b b
		*
		*/
		bMultiRegions = true;
		// Determine how many connections backTwo has to backOne's region.
		FNNOpenSpan* TestSpan = BackOne->Neighbours[(BorderDirection + 3) % 4];
		int32 BackTwoConnections = 0;
		if (TestSpan && TestSpan->RegionID == BackOne->RegionID)
		{
			++BackTwoConnections;
			TestSpan = TestSpan->Neighbours[BorderDirection];
			if (TestSpan && TestSpan->RegionID == BackOne->RegionID)
			{
				++BackTwoConnections;
			}
		}
		// Determine how many connections the current span has to backOne's region.
		int32 CurrentSpanConnections = 0;
		TestSpan = BackOne->Neighbours[(BorderDirection + 2) % 4];
		if (TestSpan && TestSpan->RegionID == BackOne->RegionID)
		{
			CurrentSpanConnections++;
			TestSpan = TestSpan->Neighbours[(BorderDirection + 2) % 4];
			if (TestSpan && TestSpan->RegionID == BackOne->RegionID)
			{
				++BackTwoConnections;
			}
		}
		// Change the region of the span that has the most connection to the target region
		if (CurrentSpanConnections > BackTwoConnections)
		{
			ChangeRegion(CurrentSpan, BackOne->RegionID);
		}
		else
		{
			ChangeRegion(*BackTwo, BackOne->RegionID);
		}
	}
	else if (BackOneRegionID == CurrentSpan.RegionID && BackTwoRegionID == CurrentSpan.RegionID)
	{
		/*
		* Dangerous configuration:
		*  b b x x
		*  b a x x <- Short wrap.
		*  b a a a
		*
		*  Fix:
		*  b b x x
		*  b b x x <- Change to this row.
		*  b a a a
		*/
		int32 SelectedRegion = SelectRegionID(*BackTwo, (BorderDirection + 1) % 4, (BorderDirection + 2) % 4);
		if (SelectedRegion == BackTwo->RegionID)
		{
			SelectedRegion = SelectRegionID(CurrentSpan, BorderDirection, (BorderDirection + 3) % 4);
			if (SelectedRegion != CurrentSpan.RegionID)
			{
				ChangeRegion(CurrentSpan, SelectedRegion);
				bMultiRegions = true;
			}
		}
		else
		{
			ChangeRegion(*BackTwo, SelectedRegion);
			bMultiRegions = true;
		}
	}
	else
	{
		bMultiRegions = true;
	}

	return bMultiRegions;
}

void FNNCleanNullRegionBorders::ChangeRegion(FNNOpenSpan& ReferenceSpan, int32 NewRegionID) const
{
	const int32 OldRegionIndex = OpenHeightField.GetRegionIndexByID(ReferenceSpan.RegionID);
	OpenHeightField.Regions[OldRegionIndex].Spans.Remove(&ReferenceSpan);
	const int32 NewRegionIndex = OpenHeightField.GetRegionIndexByID(NewRegionID);
	OpenHeightField.Regions[NewRegionIndex].Spans.Add(&ReferenceSpan);
}


void FNNCleanNullRegionBorders::PartialFloodRegion(FNNOpenSpan& StartSpan, int32 BorderDirection, FNNRegion& StartSpanRegion, FNNRegion& NewRegion)
{
	const int32 AntiBorderDirection = (BorderDirection + 2) % 4;

	StartSpan.RegionID = NewRegion.ID;
	StartSpanRegion.Spans.Remove(&StartSpan);
	NewRegion.Spans.Add(&StartSpan);
	TArray<FNNOpenSpan*> OpenSpans = {&StartSpan};
	TArray<int32> BorderDistance = {0};

	while (OpenSpans.Num() > 0)
	{
		FNNOpenSpan* CurrentSpan = OpenSpans.Pop();
		int32 Distance = BorderDistance.Pop();

		// Search all directions for neighbours
		for (int32 i = 0; i < CurrentSpan->Neighbours.Num(); ++i)
		{
			FNNOpenSpan* Neighbour = CurrentSpan->Neighbours[i];
			if (!Neighbour || Neighbour->RegionID != StartSpanRegion.ID)
			{
				continue;
			}
			int32 NeighbourDistance = Distance;
			if (i == BorderDirection)
			{
				if (Distance == 0)
				{
					// The span is at the border. Can't fo further in this direction
					continue;
				}
				--NeighbourDistance;
			}
			else if (i == AntiBorderDirection)
			{
				++NeighbourDistance;
			}
			Neighbour->RegionID = NewRegion.ID;
			NewRegion.Spans.Add(Neighbour);
			OpenSpans.Add(MoveTemp(Neighbour));
			BorderDistance.Add(NeighbourDistance);
		}
	}
}

int32 FNNCleanNullRegionBorders::SelectRegionID(const FNNOpenSpan& ReferenceSpan, int32 BorderDirection,
                                                int32 CornerDirection)
{
	TArray<FNNOpenSpan*> Neighbours = ReferenceSpan.GetDetailedNeighbours();
	/* Initial example state:
	 *
	 * a - Known region.
	 * x - Null region.
	 * u - Unknown, not checked yet.
	 *
	 *     u u u
	 *     u a x
	 *     u a a */
	FNNOpenSpan* Neighbour = Neighbours[(BorderDirection + 2) % 4];
	int32 RegionID = Neighbour ? Neighbour->RegionID : INDEX_NONE;
	if (RegionID == ReferenceSpan.RegionID || RegionID == INDEX_NONE)
	{
		/*
		  * The region away from the border is either a null region
		  * or the same region.  So we keep the current region.
		  *
		  *     u u u      u u u
		  *     a a x  or  x a x  <-- Potentially bad, but stuck with it.
		  *     u a a      u a a
		  */
		return ReferenceSpan.RegionID;
	}
	const int32 PotentialRegion = RegionID;
	Neighbour = Neighbours[(CornerDirection + 2) % 4];
	RegionID = Neighbour ? Neighbour->RegionID : INDEX_NONE;
	if (RegionID == ReferenceSpan.RegionID || RegionID == INDEX_NONE)
	{
		/*
		* The region opposite from the corner direction is
		* either a null region or the same region.  So we
		* keep the current region.
		*
		*     u a u      u x u
		*     b a x  or  b a x
		*     u a a      u a a
		*/
		return ReferenceSpan.RegionID;
	}

	// Neighbours in potential region
	int32 PotentialCount = 0;
	// Neighbours in the span's current region
	int32 CurrentCount = 0;

	for (int32 i = 0; i < 8; ++i)
	{
		Neighbour = Neighbours[i];
		const int32 NeighbourRegion = Neighbour ? Neighbour->RegionID : INDEX_NONE;
		if (NeighbourRegion == ReferenceSpan.RegionID)
		{
			++CurrentCount;
		}
		else if (NeighbourRegion == PotentialRegion)
		{
			++PotentialCount;
		}
	}

	return PotentialCount < CurrentCount ? ReferenceSpan.RegionID : PotentialRegion;
}
