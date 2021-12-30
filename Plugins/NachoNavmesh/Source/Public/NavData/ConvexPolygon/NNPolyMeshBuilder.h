#pragma once

struct FNNContour;

struct FNNPolygon
{
	/** Example: If IndexesNum = 6 and the the polygon has 4 vertices -> (1, 3, 4, 8, NULL_INDEX, NULL_INDEX)
	*  then (1, 3, 4, 8) defines the polygon. */
	FNNPolygon(int32 IndexesNum = 0) { Indexes.Init(INDEX_NONE, IndexesNum); }
	TArray<int32> Indexes;
	int32 RegionID = INDEX_NONE;
};

struct FNNPolygonMesh
{
	TArray<FVector> Vertexes;
	TArray<FNNPolygon> PolygonIndexes;
};

struct FNNOpenHeightField;

class FNNPolyMeshBuilder
{
public:
	void GenerateConvexPolygon(const TArray<FNNContour>& Contours, FNNPolygonMesh& PolygonMesh);
};
