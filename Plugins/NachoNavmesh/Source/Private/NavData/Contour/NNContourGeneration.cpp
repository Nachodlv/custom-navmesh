#include "NavData/Contour/NNContourGeneration.h"

// NN Includes
#include "NavData/NNAreaGenerator.h"
#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

#define DEBUG_CONTOUR_GENERATION 0

// TODO (Ignacio) move this to a navmesh settings
namespace NNContourGenerationStaticVariables
{
	/** The maximum distance the edge may deviate from the geometry */
	constexpr float Threshold = 0.5f;

	/** The maximum length of polygon edges that represent the border of the navmesh */
	constexpr float MaxEdgeLength = 50.0f;
}

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
	TArray<FVector> SimplifiedVertices;
	TArray<int32> SimplifiedVerticesIndexes;
	TArray<int32> SimplifiedRegions;
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
			GenerateSimplifiedContour(Vertices, VerticesRegions, SimplifiedVertices, SimplifiedVerticesIndexes);
			MatchNullRegionEdges(Vertices, VerticesRegions, SimplifiedVertices, SimplifiedVerticesIndexes);
			NullRegionMaxEdge(Vertices, VerticesRegions, SimplifiedVertices, SimplifiedVerticesIndexes);

			if (ensureMsgf(SimplifiedVertices.Num() > 2, TEXT("Fix for this case is not yet impemented")))
			{
				OpenHeightField.Contours.Emplace(CurrentSpan->RegionID, Vertices, SimplifiedVertices);
			}

			Vertices.Reset();
			VerticesRegions.Reset();
			SimplifiedVertices.Reset();
			SimplifiedVerticesIndexes.Reset();
			SimplifiedRegions.Reset();
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
	TArray<FVector>& SimplifiedVertexes, TArray<int32>& SimplifiedVertexIndexes)
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

		constexpr float Threshold = NNContourGenerationStaticVariables::Threshold;
		if (VertexIndexToAdd != INDEX_NONE && MaxDeviation > (Threshold * Threshold))
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
	TArray<FVector>& SimplifiedVertexes, TArray<int32>& SimplifiedVertexesIndexes)
{
	constexpr float MaxEdgeLength = NNContourGenerationStaticVariables::MaxEdgeLength;
	constexpr float MaxEdgeLengthSqr = MaxEdgeLength * MaxEdgeLength;
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




