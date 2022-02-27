#include "NavData/ConvexPolygon/NNPolyMeshBuilder.h"

// UE Includes
#include "CompGeom/PolygonTriangulation.h"

// NN Includes
#include "NavData/Contour/NNContourGeneration.h"

namespace NNPolyMeshBuilderVariables
{
	constexpr int32 MaxVertexesPerPoly = 5;
	constexpr int32 TriangulationFlag = 0x80000000;
	constexpr int32 TriangulationDeFlag = 0x0fffffff;
}

namespace
{
	int32 GetNextIndex(int32 Index, int32 Num)
	{
		return Index + 1 < Num ? Index + 1 : 0;
	}

	int32 GetPreviousIndex(int32 Index, int32 Num)
	{
		return Index - 1 >= 0 ? Index - 1 : Num - 1;
	}

	bool IsPointLeftFromLine(const FVector& Point, const FVector& LineStart, const FVector& LineEnd)
	{
		const float CrossProduct = FVector::CrossProduct(Point, LineStart - LineEnd).Size();
		return CrossProduct > 0;
	}
}

int32 FNNPolyMeshBuilder::Triangulate(const TArray<FVector>& ContourVertexes, TArray<int32>& VertexesIndexes,
                                      FNNPolygonMesh& PolygonMesh)
{
	return 0;
}

void FNNPolyMeshBuilder::GenerateConvexPolygon(const TArray<FNNContour>& Contours, FNNPolygonMesh& PolygonMesh)
{
	if (Contours.Num() == 0)
	{
		return;
	}

	int32 SourceVertexesNum = 0;
	// The maximum number of polygons assuming that all will be triangles
	int32 MaxPossiblePolygons = 0;
	int32 MaxVertexesPerContour = 0;

	for (const FNNContour& Contour : Contours)
	{
		const int32 Count = Contour.SimplifiedVertexes.Num();
		SourceVertexesNum += Count;
		MaxPossiblePolygons += Count - 2;
		MaxVertexesPerContour = FMath::Max(MaxVertexesPerContour, Count);
	}

	PolygonMesh.Vertexes.Reserve(SourceVertexesNum);
	PolygonMesh.PolygonIndexes.Reserve(MaxPossiblePolygons);

	TArray<int32> ContourToPolyMeshIndices;
	ContourToPolyMeshIndices.Reserve(MaxVertexesPerContour);


	for (const FNNContour& Contour : Contours)
	{
		check(Contour.SimplifiedVertexes.Num() > 2);

		TArray<int32> WorkingIndices;
		for (int32 i = 0; i < Contour.SimplifiedVertexes.Num(); ++i)
		{
			WorkingIndices.Add(i);
		}

		// Triangulate(Contour.SimplifiedVertexes, WorkingIndices, PolygonMesh);

		TArray<FVector3<float>> Vectors3D;
		Vectors3D.Reserve(Contour.SimplifiedVertexes.Num());
		for (const FVector& SimplifiedVertex : Contour.SimplifiedVertexes)
		{
			Vectors3D.Add(SimplifiedVertex);
		}
		TArray<FIndex3i> Triangles;
		// TODO (Ignacio) This triangulation is generating some bugs because is using Ear Clipping
		// We might need to change this in the future
		PolygonTriangulation::TriangulateSimplePolygon(Vectors3D, Triangles);
		for (const FIndex3i& Triangle : Triangles)
		{
			const FVector& VertexA = Contour.SimplifiedVertexes[Triangle.A];
			const FVector& VertexB = Contour.SimplifiedVertexes[Triangle.B];
			const FVector& VertexC = Contour.SimplifiedVertexes[Triangle.C];
			const int32 AIndex = PolygonMesh.Vertexes.AddUnique(VertexA);
			const int32 BIndex = PolygonMesh.Vertexes.AddUnique(VertexB);
			const int32 CIndex = PolygonMesh.Vertexes.AddUnique(VertexC);
			FNNPolygon& Polygon = PolygonMesh.PolygonIndexes.Emplace_GetRef(NNPolyMeshBuilderVariables::MaxVertexesPerPoly);
			Polygon.RegionID = Contour.RegionID;
			Polygon.Indexes.Add(AIndex);
			Polygon.Indexes.Add(BIndex);
			Polygon.Indexes.Add(CIndex);
			PolygonMesh.TriangleIndexes.Add(Polygon);
		}

		// Merge the triangles until no polygon can be found to merge
		if (MaxVertexesPerContour > 3)
		{
			while (true)
			{
				float LongestMergeEdge = 0.0f;
				int32 BestPolyA = INDEX_NONE;
				int32 PolyAVertex = INDEX_NONE;
				int32 BestPolyB = INDEX_NONE;
				int32 PolyBVertex = INDEX_NONE;
				const int32 PolygonCount = PolygonMesh.PolygonIndexes.Num();
				for (int32 PolyAIndex = 0; PolyAIndex < PolygonCount; ++PolyAIndex)
				{
					const FNNPolygon& PolyA = PolygonMesh.PolygonIndexes[PolyAIndex];
					for (int32 PolyBIndex = PolyAIndex + 1; PolyBIndex < PolygonCount; ++PolyBIndex)
					{
						const FNNPolygon& PolyB = PolygonMesh.PolygonIndexes[PolyBIndex];
						int32 MergeVertexA;
						int32 MergeVertexB;
						float MergeVertexDistance;
						const bool bResult = GetPolyMergeInfo(PolyA, PolyB, PolygonMesh, MergeVertexA, MergeVertexB,
						                                      MergeVertexDistance);
						if (bResult && MergeVertexDistance > LongestMergeEdge)
						{
							LongestMergeEdge = MergeVertexDistance;
							BestPolyA = PolyAIndex;
							PolyAVertex = MergeVertexA;
							BestPolyB = PolyBIndex;
							PolyBVertex = MergeVertexB;
						}

					}
				}

				if (LongestMergeEdge <= 0.0f)
				{
					// No valid valid merges were found
					break;
				}

				// Perform the merge

				FNNPolygon& PolyA = PolygonMesh.PolygonIndexes[BestPolyA];
				FNNPolygon& PolyB = PolygonMesh.PolygonIndexes[BestPolyB];

				// Copy the vertexes from PolyB to PolyA
				// PolyAStartVert == PolyBEndVert && PolyAEndVert == PolyBStartVert
				TArray<int32> MergedPoly;
				MergedPoly.Reserve(PolyA.Indexes.Num() + PolyB.Indexes.Num() - 2);
				for (int32 i = 0; i < PolyA.Indexes.Num() - 1; ++i)
				{
					MergedPoly.Add(PolyA.Indexes[(PolyAVertex + 1 + i) % PolyA.Indexes.Num()]);
				}
				for (int32 i = 0; i < PolyB.Indexes.Num() - 1; ++i)
				{
					MergedPoly.Add(PolyB.Indexes[(PolyBVertex + 1 + i) % PolyB.Indexes.Num()]);
				}
				PolyA.Indexes = MoveTemp(MergedPoly);
				PolygonMesh.PolygonIndexes.RemoveAtSwap(BestPolyB);
			}
		}
	}

}

bool FNNPolyMeshBuilder::GetPolyMergeInfo(const FNNPolygon& PolyA, const FNNPolygon& PolyB,
	const FNNPolygonMesh& PolygonMesh, int32& VertexToMergeA, int32& VertexToMergeB, float& DistanceSqrOfEdge)
{
	// Default to invalid merge
	VertexToMergeA = INDEX_NONE;
	VertexToMergeB = INDEX_NONE;
	DistanceSqrOfEdge = 0.0f;

	const int32 VertCountA = PolyA.Indexes.Num();
	const int32 VertCountB = PolyB.Indexes.Num();

	// Subtracting two to take into account the effect of a merge
	if (VertCountA + VertCountB - 2 > NNPolyMeshBuilderVariables::MaxVertexesPerPoly)
	{
		return false;
	}

	// Check if the polygons share an edge
	for (int32 IndexVertA = 0; IndexVertA < VertCountA; ++IndexVertA)
	{
		const int32 VertexA = PolyA.Indexes[IndexVertA];
		const int32 VertexANext = PolyA.Indexes[GetNextIndex(IndexVertA, VertCountA)];

		for (int32 IndexVertB = 0; IndexVertB < VertCountB; ++IndexVertB)
		{
			const int32 VertexB = PolyB.Indexes[IndexVertB];
			const int32 VertexBNext = PolyB.Indexes[GetNextIndex(IndexVertB, VertCountB)];

			if (VertexA == VertexBNext && VertexANext == VertexB)
			{
				VertexToMergeA = IndexVertA;
				VertexToMergeB = IndexVertB;
			}
		}
	}

	if (VertexToMergeA == INDEX_NONE)
	{
		// No common edge
		return false;
	}

	// We need that the new polygon is convex
	// It checks if the new section forms a concave section. If so, then the merge is invalid

	int32 SharedVertexPrevious = PolyA.Indexes[GetPreviousIndex(VertexToMergeA, VertCountA)];
	int32 SharedVertexIndex = PolyA.Indexes[VertexToMergeA];
	int32 SharedVertexNext = PolyB.Indexes[(VertexToMergeB + 2) % VertCountB];

	if (!IsPointLeftFromLine(PolygonMesh.Vertexes[SharedVertexIndex],
							 PolygonMesh.Vertexes[SharedVertexPrevious],
							 PolygonMesh.Vertexes[SharedVertexNext]))
	{
		// The shared vertex is not left of the segment SharedVertexPrevious->SharedVertexNext
		// Because the polygon is wrapped clockwise this indicates the section os concave
		return false;
	}

	SharedVertexPrevious = PolyB.Indexes[GetPreviousIndex(VertexToMergeB, VertCountB)];
	SharedVertexIndex = PolyB.Indexes[VertexToMergeB];
	SharedVertexNext = PolyA.Indexes[(VertexToMergeA + 2) % VertCountA];

	if (!IsPointLeftFromLine(PolygonMesh.Vertexes[SharedVertexIndex],
							 PolygonMesh.Vertexes[SharedVertexPrevious],
							 PolygonMesh.Vertexes[SharedVertexNext]))
	{
		// The shared vertex is not left of the segment SharedVertexPrevious->SharedVertexNext
		// Because the polygon is wrapped clockwise this indicates the section os concave
		return false;
	}

	const FVector& PreviousVertex = PolygonMesh.Vertexes[PolyA.Indexes[VertexToMergeA]];
	const FVector& SharedVertex = PolygonMesh.Vertexes[PolyA.Indexes[GetNextIndex(VertexToMergeA, VertCountA)]];

	const float DeltaX = PreviousVertex.X - SharedVertex.X;
	const float DeltaY = PreviousVertex.Y - SharedVertex.Y;
	DistanceSqrOfEdge = DeltaX * DeltaX + DeltaY * DeltaY;
	return true;
}



