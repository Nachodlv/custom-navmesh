#pragma once

// NN Includes
#include "NNAreaGenerator.h"

struct FNNAreaGeneratorData;
struct FNNRawGeometryElement;
struct FNNHeightField;

/** Generates a HeightField withing given bounds */
class FHeightFieldGenerator
{
public:
	FHeightFieldGenerator(FNNAreaGeneratorData& InAreaGeneratorData) : AreaGeneratorData(InAreaGeneratorData) {}

	/** Creates a new HeightField with the given parameters */
	FNNHeightField* InitializeHeightField(TArray<FNNRawGeometryElement>& RawGeometry, const FVector& BoundMinPoint, const FVector& BoundMaxPoint, float CellSize, float CellHeight) const;

protected:
	/** Creates a 2D bounding box that contains the Polygon */
	static bool Generate2DBoundingBoxForGeometry(TArray<FVector>& Polygon, FVector& OutMinimumPoint, FVector& OutMaximumPoint, const FBox& BoundBox);

	/** Returns a span with the CurrentSpan and the NewSpan attached */
	static Span* AttachNewSpan(Span* CurrentSpan, Span* NewSpan);

	/** Combines the two Span */
	static Span* CombineSpans(Span* LowerSpan, const Span* HigherSpan);

	/** Creates a debug point in the world */
	void AddDebugPoint(const FVector& Point, float Radius = 20.0f) const;

	/** Creates a debug text in the world */
	void AddDebugText(const FVector& Location, const FString& Text) const;
private:
	/** Data from the AreaGenerator that created this class. Used to debug points and texts in the world */
	FNNAreaGeneratorData& AreaGeneratorData;
};
