#include "NavData/Contour/NNContourGeneration.h"

// NN Includes
#include "NavData/NNAreaGenerator.h"
#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

#define DEBUG_CONTOUR_GENERATION 0

void FNNContourGeneration::CalculateContour(FNNOpenHeightField& OpenHeightField, TArray<FNNContour>& OutContours)
{
	for (FNNOpenHeightFieldIterator It (OpenHeightField); It; ++It)
	{
		FNNOpenSpan* CurrentSpan = It.Get();
		CurrentSpan->NeighbourFlags = 0;
		if (CurrentSpan->RegionID == INDEX_NONE)
		{
			continue;
		}
		for (int32 Dir = 0; Dir < 4; ++Dir)
		{
			const FNNOpenSpan* Neighbour = CurrentSpan->Neighbours[Dir];
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
		}
#if DEBUG_CONTOUR_GENERATION
		AreaGeneratorData.AddDebugText(CurrentSpan->GetOpenSpanWorldPosition(OpenHeightField), FString::FromInt(CurrentSpan->NeighbourFlags));
#endif
	}

	TArray<FVector> Vertices;
	TArray<int32> VerticesRegions;
	TArray<FVector> SimplifiedVertices;
	TArray<int32> SimplifiedVerticesIndexes;

	for (FNNOpenHeightFieldIterator It (OpenHeightField); It; ++It)
	{
		FNNOpenSpan* CurrentSpan = It.Get();
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
		GenerateSimplifiedContour(Vertices, VerticesRegions, SimplifiedVertices, SimplifiedVerticesIndexes);
		MatchNullRegionEdges(Vertices, VerticesRegions, SimplifiedVertices, SimplifiedVerticesIndexes);
		NullRegionMaxEdge(Vertices, VerticesRegions, SimplifiedVertices, SimplifiedVerticesIndexes);

		if (SimplifiedVertices.Num() > 2)
		{
			OutContours.Emplace(CurrentSpan->RegionID, Vertices, SimplifiedVertices);
		}

		Vertices.Reset();
		VerticesRegions.Reset();
		SimplifiedVertices.Reset();
		SimplifiedVerticesIndexes.Reset();
	}
	
	for (const FNNRegion& Region : OpenHeightField.Regions)
	{
		const int32 RegionID = Region.ID;
		int32 RegionMatches = 0;
		for (const FNNContour& Contour : OutContours)
		{
			if (Contour.RegionID == RegionID)
			{
				++RegionMatches;
				ensureMsgf (RegionMatches <= 1, TEXT("More that one contour with the same region ID"));
			}
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

			const FNNOpenSpan* Neighbour = CurrentSpan->Neighbours[Dir];
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

void FNNContourGeneration::GenerateSimplifiedContour(const TArray<FVector>& SourceVertexes,
	const TArray<int32>& SourceRegions, TArray<FVector>& OutSimplifiedVertexes,
	TArray<int32>& OutSimplifiedVertexesIndexes) const
{
	bool bNoConnections = true;
	for (const int32 Region : SourceRegions)
	{
		if (Region != INDEX_NONE)
		{
			bNoConnections = false;
			break;
		}
	}

	// Add the mandatory edges to the simplified contour
	if (bNoConnections)
	{
		int32 LowerLeftIndex = 0;
		int32 UpperRightIndex = 0;
		for (int32 i = 0; i < SourceVertexes.Num(); ++i)
		{
			const FVector& Vertex = SourceVertexes[i];

			const FVector& LowerLeft = SourceVertexes[LowerLeftIndex];
			if (Vertex.X < LowerLeft.X || (Vertex.X == LowerLeft.X && Vertex.Y < LowerLeft.Y))
			{
				LowerLeftIndex = i;
			}

			const FVector& UpperRight = SourceVertexes[UpperRightIndex];
			// TODO (Ignacio) I don't know if I should check (Vertex.X > UpperRight.X) or (Vertex.X >= UpperRight.X)
			if (Vertex.X > UpperRight.X || (Vertex.X == UpperRight.X && Vertex.Y > UpperRight.Y))
			{
				UpperRightIndex = i;
			}
		}

		OutSimplifiedVertexes.Add(SourceVertexes[LowerLeftIndex]);
		OutSimplifiedVertexesIndexes.Add(LowerLeftIndex);
		OutSimplifiedVertexes.Add(SourceVertexes[UpperRightIndex]);
		OutSimplifiedVertexesIndexes.Add(UpperRightIndex);
	}
	else
	{
		// The contour shares edges with other non null regions
		// Add the vertexes where the region connected changes to the simplified contour
		for (int32 i = 0; i < SourceVertexes.Num(); ++i)
		{
			const int32 CurrentRegion = SourceRegions[i];
			const int32 NextRegion = SourceRegions[(i + 1) % SourceRegions.Num()];
			if (CurrentRegion != NextRegion)
			{
				OutSimplifiedVertexes.Add(SourceVertexes[i]);
				OutSimplifiedVertexesIndexes.Add(i);
			}
		}
	}

	RemoveVerticalSegments(OutSimplifiedVertexes, OutSimplifiedVertexesIndexes);
	RemoveIntersectionSegments(OutSimplifiedVertexes, OutSimplifiedVertexesIndexes, SourceRegions);
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

void FNNContourGeneration::MatchNullRegionEdges(const TArray<FVector>& SourceVertexes, const TArray<int32>& SourceRegions,
	TArray<FVector>& SimplifiedVertexes, TArray<int32>& SimplifiedVertexIndexes) const
{
	if (SourceVertexes.Num() == 0)
	{
		return;
	}

	int32 IndexVertexA = 0;

	while (IndexVertexA < SimplifiedVertexes.Num())
	{
		const int32 IndexVertexB = (IndexVertexA + 1) % SimplifiedVertexes.Num();

		// The begging of the segment
		const FVector& VertexA = SimplifiedVertexes[IndexVertexA];
		const int32 SourceVertexAIndex = SimplifiedVertexIndexes[IndexVertexA];

		// The end of the segment
		const FVector& VertexB = SimplifiedVertexes[IndexVertexB];
		const int32 SourceVertexBIndex = SimplifiedVertexIndexes[IndexVertexB];

		// The vertex just after the current vertex in the source vertex list
		int32 TestVertexIndex = (SourceVertexAIndex + 1) % SourceVertexes.Num();
		float MaxDeviation = 0.0f;

		int32 VertexIndexToAdd = INDEX_NONE;
		// The TestVertex is part of a null region edge
		if (SourceRegions[TestVertexIndex] == INDEX_NONE)
		{
			while (TestVertexIndex != SourceVertexBIndex)
			{
				const float Deviation = FMath::PointDistToSegmentSquared(SourceVertexes[TestVertexIndex], VertexA, VertexB);
				if (Deviation > MaxDeviation)
				{
					MaxDeviation = Deviation;
					VertexIndexToAdd = TestVertexIndex;
				}
				TestVertexIndex = (TestVertexIndex + 1) % SourceVertexes.Num();
			}
		}

		if (VertexIndexToAdd != INDEX_NONE && MaxDeviation > (ContourDeviationThreshold * ContourDeviationThreshold))
		{
			const FVector& VectorToAdd = SourceVertexes[VertexIndexToAdd];
			SimplifiedVertexes.Insert(VectorToAdd, IndexVertexA + 1);
			SimplifiedVertexIndexes.Insert(VertexIndexToAdd, IndexVertexA + 1);
		}
		else
		{
			++IndexVertexA;
		}
	}
}

void FNNContourGeneration::NullRegionMaxEdge(const TArray<FVector>& SourceVertexes, const TArray<int32>& SourceRegions,
	TArray<FVector>& SimplifiedVertexes, TArray<int32>& SimplifiedVertexesIndexes) const
{
	const float MaxEdgeLengthSqr = MaxEdgeLength * MaxEdgeLength;
	if (MaxEdgeLength <= 0 || SourceVertexes.Num() == 0)
	{
		return;
	}

	const int32 SourceVertexesNum = SourceVertexes.Num();

	int32 VertexAIndex = 0;
	// Inserts vertexes into null region edges that are too long
	// Insets the vertex that is closer to the middle of the edge
	while (VertexAIndex < SimplifiedVertexes.Num())
	{
		const int32 VertexBIndex = (VertexAIndex + 1) % SimplifiedVertexes.Num();

		const FVector& VectorA = SimplifiedVertexes[VertexAIndex];
		const int32 SourceVectorAIndex = SimplifiedVertexesIndexes[VertexAIndex];

		const FVector& VectorB = SimplifiedVertexes[VertexBIndex];
		const int32 SourceVectorBIndex = SimplifiedVertexesIndexes[VertexBIndex];

		int32 NewVertexIndex = INDEX_NONE;
		const int32 TestVertexIndex = (SourceVectorAIndex + 1) % SourceVertexesNum;
		if (SourceRegions[TestVertexIndex] == INDEX_NONE)
		{
			if (FVector::DistSquared(VectorA, VectorB) > MaxEdgeLengthSqr)
			{
				// The current edge is too long and needs to be split
				const int32 IndexDistance = SourceVectorBIndex < SourceVectorAIndex
					                            ? SourceVectorBIndex + (SourceVertexesNum - SourceVectorAIndex)
					                            : SourceVectorBIndex - SourceVectorAIndex;
				NewVertexIndex = (SourceVectorAIndex + IndexDistance / 2) % SourceVertexesNum;
			}
		}

		if (NewVertexIndex != INDEX_NONE && NewVertexIndex != SourceVectorAIndex && NewVertexIndex != SourceVectorBIndex)
		{
			SimplifiedVertexes.Insert(SourceVertexes[NewVertexIndex], VertexAIndex + 1);
			SimplifiedVertexesIndexes.Insert(NewVertexIndex, VertexAIndex + 1);
		}
		else
		{
			++VertexAIndex;
		}
	}
}

void FNNContourGeneration::RemoveVerticalSegments(TArray<FVector>& Vertexes, TArray<int32>& Indexes) const
{
	for (int32 i = 0; i < Vertexes.Num();)
	{
		const int32 NextVertexIndex = (i + 1) % Vertexes.Num();
		const FVector& NextVertex = Vertexes[NextVertexIndex];
		const FVector& Vertex = Vertexes[i];
		if (Vertex.X == NextVertex.X && Vertex.Y == NextVertex.Y)
		{
			Vertexes.RemoveAt(NextVertexIndex);
			Indexes.RemoveAt(NextVertexIndex);
		}
		else
		{
			++i;
		}
	}	
}

void FNNContourGeneration::RemoveIntersectionSegments(TArray<FVector>& Vertexes, TArray<int32>& SimplifiedIndexes, const TArray<int32>& SourceRegions) const
{
	for (int32 i = 0; i < Vertexes.Num(); ++i)
	{
		const int32 NextVertexIndex = (i + 1) % Vertexes.Num();
		if (SourceRegions[SimplifiedIndexes[NextVertexIndex]] != INDEX_NONE)
		{
			i += RemoveIntersectionSegments(i, NextVertexIndex, Vertexes, SimplifiedIndexes, SourceRegions);
		}
	}
}

int32 FNNContourGeneration::RemoveIntersectionSegments(int32 StartVertexIndex, int32 EndVertexIndex,
                                                       TArray<FVector>& Vertexes, TArray<int32>& SimplifiedIndexes, const TArray<int32>& SourceRegions) const
{
	if (Vertexes.Num() < 4)
	{
		return 0;
	}

	int32 Offset = 0;
	int32 VertexIndex = (EndVertexIndex + 2) % Vertexes.Num();
	int32 VertexIndexMinus = (EndVertexIndex + 1) % Vertexes.Num();
	while (VertexIndex != StartVertexIndex)
	{
		const FVector& StartVertex = Vertexes[StartVertexIndex];
		const FVector& EndVertex = Vertexes[EndVertexIndex];
		FVector IntersectionPoint;
		const int32 VertexRegion = SourceRegions[SimplifiedIndexes[VertexIndex]];
		const int32 NextVertexRegion = SourceRegions[SimplifiedIndexes[(VertexIndex + 1) % Vertexes.Num()]];
		// Only remove the vertex if both edges it belongs connect to the null region
		// And it belongs to a segment that intersect the segment being tested against
		if (VertexRegion == INDEX_NONE
			&& NextVertexRegion == INDEX_NONE
			&& FMath::SegmentIntersection2D(StartVertex, EndVertex, Vertexes[VertexIndexMinus], Vertexes[VertexIndex], IntersectionPoint))
		{
			// Remove the null region segment
			Vertexes.RemoveAt(VertexIndex);
			SimplifiedIndexes.RemoveAt(VertexIndex);
			if (VertexIndex < StartVertexIndex || VertexIndex < EndVertexIndex)
			{
				--StartVertexIndex;
				--EndVertexIndex;
				--Offset;
			}
			if (VertexIndex < VertexIndexMinus)
			{
				--VertexIndexMinus;
			}
			VertexIndex = VertexIndex % Vertexes.Num();
		}
		else
		{
			// Move to the next segment
			VertexIndexMinus = VertexIndex;
			VertexIndex = (VertexIndex + 1) % Vertexes.Num();
		}
	}

	return Offset;
}




