#include "NavData/Contour/NNContourGeneration.h"

// NN Includes
#include "NavData/NNAreaGenerator.h"
#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

#define DEBUG_CONTOUR_GENERATION 0

void FNNContourGeneration::CalculateContour(FNNOpenHeightField& OpenHeightField)
{
	int32 DiscardedContours = 0;
	for (TUniquePtr<FNNOpenSpan>& OpenSpan : OpenHeightField.Spans)
	{
		for (FNNOpenSpan* CurrentSpan = OpenSpan.Get(); CurrentSpan; CurrentSpan = CurrentSpan->NextOpenSpan.Get())
		{
			CurrentSpan->NeighbourFlags = 0;
			if (CurrentSpan->RegionID == INDEX_NONE)
			{
				continue;
			}
			for (int32 Dir = 0; Dir < 4; ++Dir)
			{
				FNNOpenSpan* Neighbour = CurrentSpan->Neighbours[Dir];
				if (Neighbour && CurrentSpan->RegionID == Neighbour->RegionID)
				{
					// Set the neighbour bit to 1
					CurrentSpan->NeighbourFlags |= (1 << Dir);
				}
			}
			// Invert flags
			CurrentSpan->NeighbourFlags ^= 0xf;
			// Check if it's an island span. All neighbours are in other regions
			if (CurrentSpan->NeighbourFlags == 0xf)
			{
				CurrentSpan->NeighbourFlags = 0;
				++DiscardedContours;
			}
#if DEBUG_CONTOUR_GENERATION
			AreaGeneratorData.AddDebugText(CurrentSpan->GetOpenSpanWorldPosition(OpenHeightField), FString::FromInt(CurrentSpan->NeighbourFlags));
#endif
		}
	}

	TArray<FVector> Vertices;
	TArray<int32> VerticesRegions;
	for (TUniquePtr<FNNOpenSpan>& OpenSpan : OpenHeightField.Spans)
	{
		for (FNNOpenSpan* CurrentSpan = OpenSpan.Get(); CurrentSpan; CurrentSpan = CurrentSpan->NextOpenSpan.Get())
		{
			// Span already processed
			if (!CurrentSpan || CurrentSpan->RegionID == INDEX_NONE || CurrentSpan->NeighbourFlags == 0)
			{
				continue;
			}

			// Locate the direction which points to another region
			int32 StartDirection = 0;
			while ((CurrentSpan->NeighbourFlags & (1 << StartDirection)) == 0)
			{
				++StartDirection;
			}

			BuildRawContour(CurrentSpan, StartDirection, Vertices, VerticesRegions);

			OpenHeightField.Contours.Emplace(Vertices);
			Vertices.Empty();
		}
	}
}

void FNNContourGeneration::BuildRawContour(FNNOpenSpan* StartSpan, int32 StartDir, TArray<FVector>& OutContourVerts, TArray<int32>& OutVertsRegions)
{
	FNNOpenSpan* CurrentSpan = StartSpan;
	int32 Dir = StartDir;

	int32 LoopCount = 0;
	while (LoopCount < 999999)
	{
		++LoopCount;

		if ((CurrentSpan->NeighbourFlags & (1 << Dir)) != 0)
		{
			// TODO check if the X and Y are correct
			int32 EdgeX = CurrentSpan->X;
			int32 EdgeY = CurrentSpan->Y;
			const int32 EdgeZ = GetCornerHeight(*CurrentSpan, Dir);

			// This update is so the corner being represented is clockwise from the edge the direction is currently
			// pointing towards
			switch (Dir)
			{
			case 0:
				++EdgeY; break;
			case 1:
				++EdgeX; ++EdgeY; break;
			case 2:
				++EdgeX; break;
			default: break;
			}

			FNNOpenSpan* Neighbour = CurrentSpan->Neighbours[Dir];
			int32 RegionNeighbour = Neighbour ? Neighbour->RegionID : INDEX_NONE;
			OutContourVerts.Emplace(EdgeX, EdgeY, EdgeZ);
			OutVertsRegions.Add(RegionNeighbour);

			// Remove the flag to mark it as already processed
			CurrentSpan->NeighbourFlags &= ~(1 << Dir);
			Dir = (Dir + 1) % 4; // Rotate clockwise
		}
		else
		{
			// The current direction doesn't point to an edge. Points towards a neighbour of the same region
			// Move to the neighbour and rotate counterclockwise
			CurrentSpan = CurrentSpan->Neighbours[Dir];
			Dir = (Dir + 3) % 4;
		}

		if (CurrentSpan == StartSpan && Dir == StartDir)
		{
			return;
		}
	}
	ensureMsgf(false, TEXT("Something went wrong"));
}

int32 FNNContourGeneration::GetCornerHeight(FNNOpenSpan& Span, int32 Direction) const
{
	int32 MaxFloor = Span.MinHeight;
	FNNOpenSpan* DiagonalNeighbour = nullptr;

	// Rotate clockwise
	const int32 DirectionOffset = (Direction + 1) % 4;

	// Check axis neighbour in the current direction
	if (FNNOpenSpan* Neighbour = Span.Neighbours[Direction])
	{
		MaxFloor = FMath::Max(MaxFloor, Neighbour->MinHeight);
		DiagonalNeighbour = Neighbour->Neighbours[DirectionOffset];
	}

	// Check neighbour in clockwise direction
	if (FNNOpenSpan* Neighbour = Span.Neighbours[DirectionOffset])
	{
		MaxFloor = FMath::Max(MaxFloor, Neighbour->MinHeight);
		if (!DiagonalNeighbour)
		{
			// No diagonal neighbour has been found yet. Check counter clockwise from this neighbour
			DiagonalNeighbour = Neighbour->Neighbours[Direction];
		}
	}

	if (DiagonalNeighbour)
	{
		MaxFloor = FMath::Max(MaxFloor, DiagonalNeighbour->MinHeight);
	}

	return MaxFloor;
}



