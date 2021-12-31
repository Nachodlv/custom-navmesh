#pragma once

struct FNNAreaGeneratorData;
struct FNNOpenSpan;
struct FNNRegion;
struct FNNOpenHeightField;

/** Contains the vertexes of a contour */
struct FNNContour
{
	FNNContour(int32 InRegionID, const TArray<FVector>& InRawVertexes, const TArray<FVector>& InSimplifiedVertexes)
		: RegionID(InRegionID), RawVertexes(InRawVertexes), SimplifiedVertexes(InSimplifiedVertexes) {}
	int32 RegionID;
	TArray<FVector> RawVertexes;
	TArray<FVector> SimplifiedVertexes;
	TArray<FNNContour*> Neighbours;
};

struct FNNContourMesh
{
	FBox Bounds;
	TArray<TUniquePtr<FNNContour>> Contours;
};

/** Generates contours with a given OpenHeightField */
class FNNContourGeneration
{

public:
	FNNContourGeneration(FNNAreaGeneratorData& InAreaGenerator, float InDeviationThreshold, float InMaxEdgeLength)
		: AreaGeneratorData(InAreaGenerator), ContourDeviationThreshold(InDeviationThreshold), MaxEdgeLength(InMaxEdgeLength) {}

	/** Calculates the contour of the OpenHeightField and inserts the contour in the height field */
	void CalculateContour(FNNOpenHeightField& OpenHeightField, TArray<FNNContour>& OutContours);

protected:
	/** Builds a basic contour for the region of the StartSpan */
	void BuildRawContour(FNNOpenSpan* StartSpan, FPlatformTypes::int32 StartDir, TArray<FVector>&
	                      OutContourVerts, TArray<int32>& OutVertsRegions);

	/** Changes the vertexes from the region.
	 * For edges that connects non null regions it will remove all vertices except the start and the end vertex
	 * */
	void GenerateSimplifiedContour(const TArray<FVector>& SourceVertexes, const TArray<int32>& SourceRegions, TArray<FVector>& OutSimplifiedVertexes, TArray<int32>& OutSimplifiedVertexesIndexes) const;

	/** Returns the height that should be used for the parameter Span.
	 * The vertex clockwise of the specified direction */
	int32 GetCornerHeight(FNNOpenSpan& Span, int32 Direction) const;

	/** Adds vertexes to the null-region edges. All these vertexes will be closer than the given Threshold from the
	 * null region edges */
	void MatchNullRegionEdges(const TArray<FVector>& SourceVertexes, const TArray<int32>& SourceRegions, TArray<FVector>& SimplifiedVertexes, TArray<int32>& SimplifiedVertexIndexes) const;

	/** Add vertexes to a contour sucha that no null region edge segment exceeds the allowed edge length */
	void NullRegionMaxEdge(const TArray<FVector>& SourceVertexes, const TArray<int32>& SourceRegions, TArray<FVector>& SimplifiedVertexes, TArray<int32>& SimplifiedVertexesIndexes) const;

private:
	FNNAreaGeneratorData& AreaGeneratorData;

	/** The maximum distance the edge may deviate from the geometry */
	float ContourDeviationThreshold = 0.0f;

	/** The maximum length of polygon edges that represent the border of the navmesh */
	float MaxEdgeLength = 0.0f;
};
