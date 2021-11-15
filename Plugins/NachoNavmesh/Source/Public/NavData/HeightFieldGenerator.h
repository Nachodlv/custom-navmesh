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
	FNNHeightField* InitializeHeightField(TArray<FNNRawGeometryElement>& RawGeometry, const FVector& BoundMinPoint, const FVector& BoundMaxPoint, float CellSize, float CellHeight, float WalkableAngle, float AgentHeight, float MinLedgeHeight) const;

protected:
	/** Creates a 2D bounding box that contains the Polygon */
	static bool Generate2DBoundingBoxForGeometry(TArray<FVector>& Polygon, FVector& OutMinimumPoint, FVector& OutMaximumPoint, const FBox& BoundBox);

	/** Attaches the new span into the CurrentSpan */
	void AttachNewSpan(std::unique_ptr<Span>& CurrentSpan, std::unique_ptr<Span>& NewSpan) const;

	/** Combines the two Spans */
	std::unique_ptr<Span>& CombineSpans(std::unique_ptr<Span>& LowerSpan, const std::unique_ptr<Span>& HigherSpan) const;

	/** Creates a debug point in the world */
	void AddDebugPoint(const FVector& Point, float Radius = 20.0f) const;

	/** Creates a debug text in the world */
	void AddDebugText(const FVector& Location, const FString& Text) const;

	/** Creates a debug line in the world */
	void AddDebugLine(const FVector& Start, const FVector& End) const;

	/** Returns whether the Polygon provided is walkable */
	bool IsPolygonWalkable(const FVector& PolygonNormal, float MaxWalkableRadians) const;

	/** Returns whether the span provided is walkable */
	bool IsSpanWalkable(const FNNHeightField* HeightField, int32 XIndex, int32 YIndex, const Span* InSpan, float AgentHeight, float MinLedgeHeight) const;

	/** Returns the neighbours of the Span provided */
	TArray<Span*> GetSpanNeighbours(const FNNHeightField* HeightField, int32 XIndex, int32 YIndex, const Span* CurrentSpan) const;
private:
	/** Data from the AreaGenerator that created this class. Used to debug points and texts in the world */
	FNNAreaGeneratorData& AreaGeneratorData;
};
