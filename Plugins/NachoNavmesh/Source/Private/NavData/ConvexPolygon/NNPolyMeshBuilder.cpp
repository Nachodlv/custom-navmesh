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

	// TArray<int32> ContourToPolyMeshIndices;
	// ContourToPolyMeshIndices.Reserve(MaxVertexesPerContour);
	//
	// TArray<int32> WorkingIndexes;
	// WorkingIndexes.Reserve(MaxVertexesPerContour);
	// TArray<int32> WorkingTriangles;
	// WorkingTriangles.Reserve(MaxVertexesPerContour);
	//
	// TArray<int32> WorkingPolys;
	// WorkingPolys.Reserve((MaxVertexesPerContour + 1) * NNPolyMeshBuilderVariables::MaxVertexesPerPoly);
	// TArray<int32> MergeInfo = {0, 0, 0};
	// TArray<int32> MergedPoly;
	// MergedPoly.Reserve(NNPolyMeshBuilderVariables::MaxVertexesPerPoly);

	for (const FNNContour& Contour : Contours)
	{
		check(Contour.SimplifiedVertexes.Num() > 2);

		// WorkingIndexes.Reset();
		// for (int32 i = 0; i < Contour.SimplifiedVertexes.Num(); ++i)
		// {
		// 	WorkingIndexes.Add(i);
		// }

		TArray<FVector2<float>> Vectors;
		TArray<FVector3<float>> Vectors3D;
		Vectors.Reserve(Contour.SimplifiedVertexes.Num());
		Vectors3D.Reserve(Contour.SimplifiedVertexes.Num());
		for (const FVector& SimplifiedVertex : Contour.SimplifiedVertexes)
		{
			Vectors.Add(FVector2<float>(SimplifiedVertex.X, SimplifiedVertex.Y));
			Vectors3D.Add(SimplifiedVertex);
		}
		TArray<FIndex3i> Triangles;
		// PolygonTriangulation::TriangulateSimplePolygon(Vectors, Triangles);
		PolygonTriangulation::TriangulateSimplePolygon(Vectors3D, Triangles);
		for (const FIndex3i& Triangle : Triangles)
		{
			const FVector& VertexA = Contour.SimplifiedVertexes[Triangle.A];
			const FVector& VertexB = Contour.SimplifiedVertexes[Triangle.B];
			const FVector& VertexC = Contour.SimplifiedVertexes[Triangle.C];
			const int32 AIndex = PolygonMesh.Vertexes.AddUnique(VertexA);
			const int32 BIndex = PolygonMesh.Vertexes.Add(VertexB);
			const int32 CIndex = PolygonMesh.Vertexes.Add(VertexC);
			FNNPolygon& Polygon = PolygonMesh.PolygonIndexes.Emplace_GetRef(NNPolyMeshBuilderVariables::MaxVertexesPerPoly);
			Polygon.RegionID = Contour.RegionID;
			Polygon.Indexes[0] = AIndex;
			Polygon.Indexes[1] = BIndex;
			Polygon.Indexes[2] = CIndex;
		}
	}
}

