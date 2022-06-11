#pragma once

struct FNNContour;

struct FNNPolygon
{
	/** Example: If IndexesNum = 6 and the the polygon has 4 vertices -> (1, 3, 4, 8, NULL_INDEX, NULL_INDEX)
	*  then (1, 3, 4, 8) defines the polygon. */
	FNNPolygon(int32 IndexesNum = 0) { Indexes.Reserve(IndexesNum); }
	TArray<int32> Indexes;
	int32 RegionID = INDEX_NONE;
	NavNodeRef NodeRef = INVALID_NAVNODEREF;
	friend bool operator==(const FNNPolygon& Lhs, const FNNPolygon& Rhs) { return Lhs.NodeRef == Rhs.NodeRef; }
};

struct FNNPolygonMesh
{
	TArray<FVector> Vertexes;
	TArray<FNNPolygon> PolygonIndexes;
	/** These indices are only used for drawing the mesh */
	TArray<FNNPolygon> TriangleIndexes;
};

struct FNNOpenHeightField;

class FNNPolyMeshBuilder
{
public:
	void GenerateConvexPolygon(const TArray<FNNContour>& Contours, FNNPolygonMesh& PolygonMesh);

protected:
	/** Gets the vertexes that need to be merged between the PolyA and PolyB.
	 * Returns whether the merge is possible */
	static bool GetPolyMergeInfo(const FNNPolygon& PolyA, const FNNPolygon& PolyB, const FNNPolygonMesh& PolygonMesh,
	                             int32& VertexToMergeA, int32& VertexToMergeB, float& DistanceSqrOfEdge);

	/** Attempts to triangulate a polygon */
	int32 Triangulate(const TArray<FVector>& ContourVertexes, TArray<int32>& VertexesIndexes, FNNPolygonMesh& PolygonMesh);
};
