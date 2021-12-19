#pragma once

struct FNNAreaGeneratorData;
struct FNNOpenSpan;
struct FNNRegion;
struct FNNOpenHeightField;

/** Contains the vertexes of a contour */
struct FNNContour
{
	FNNContour(const TArray<FVector>& InVertexes) : Vertexes(InVertexes) {}
	TArray<FVector> Vertexes;
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
	FNNContourGeneration(FNNAreaGeneratorData& InAreaGenerator) : AreaGeneratorData(InAreaGenerator) {}

	/** Calculates the contour of the OpenHeightField and inserts the contour in the height field */
	void CalculateContour(FNNOpenHeightField& OpenHeightField);

protected:
	/** Builds a basic contour for the region of the StartSpan */
	void BuildRawContour(FNNOpenSpan* StartSpan, FPlatformTypes::int32 StartDir, TArray<FVector>&
	                      OutContourVerts, TArray<int32>& OutVertsRegions);

	/** Returns the height that should be used for the parameter Span.
	 * The vertex clockwise of the specified direction */
	int32 GetCornerHeight(FNNOpenSpan& Span, int32 Direction) const;

private:
	FNNAreaGeneratorData& AreaGeneratorData;
};
