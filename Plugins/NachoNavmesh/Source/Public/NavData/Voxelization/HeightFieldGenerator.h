#pragma once

// NN Includes
#include "NavData/NNAreaGenerator.h"

struct FNNAreaGeneratorData;
struct FNNRawGeometryElement;

/** Represents a cell that collides with a polygon */
struct Span
{
	Span() {}
	Span (Span& InSpan)
	{
		CopySpan(InSpan);
	}
	Span(int32 InMaxSpanHeight, int32 InMinSpanHeight, bool bInWalkable)
		: MaxSpanHeight(InMaxSpanHeight), MinSpanHeight(InMinSpanHeight), bWalkable(bInWalkable) {}
	Span (Span&& InSpan) noexcept
		: MaxSpanHeight(InSpan.MaxSpanHeight), MinSpanHeight(InSpan.MinSpanHeight), bWalkable(InSpan.bWalkable), NextSpan(MoveTemp(InSpan.NextSpan)) {}

	int32 MaxSpanHeight = INDEX_NONE;
	int32 MinSpanHeight = INDEX_NONE;
	bool bWalkable = false;
	TUniquePtr<Span> NextSpan = nullptr;

	/** Returns a readable representation of this Span */
	FString ToString() const;

	void CopySpan(Span& InSpan)
	{
		MaxSpanHeight = InSpan.MaxSpanHeight;
		MinSpanHeight = InSpan.MinSpanHeight;
		bWalkable = InSpan.bWalkable;
		if (InSpan.NextSpan)
		{
			NextSpan = MakeUnique<Span>(*InSpan.NextSpan.Release());
		}
	}
};

/** Container of spans */
struct FNNHeightField
{
	FNNHeightField() {}
	FNNHeightField(int32 InUnitsWidth, int32 InUnitsHeight, int32 InUnitsDepth);

	/** How spans are contained in every axis */
	int32 UnitsWidth = 0; // X
	int32 UnitsHeight = 0; // Z
	int32 UnitsDepth = 0; // Y

	/** Bounds */
	FVector MinPoint;
	FVector MaxPoint;

	/** The span size */
	float CellSize = 0.0f;
	float CellHeight = 0.0f;

	TArray<TUniquePtr<Span>> Spans; // 2D array, UnitsWidth * UnitsDepth
};

/** Generates a HeightField withing given bounds */
class FHeightFieldGenerator
{
public:
	FHeightFieldGenerator(FNNAreaGeneratorData& InAreaGeneratorData) : AreaGeneratorData(InAreaGeneratorData) {}

	/** Creates a new HeightField with the given parameters */
	void InitializeHeightField(FNNHeightField& OutHeightField, TArray<FNNRawGeometryElement>& RawGeometry,
		const FVector& BoundMinPoint, const  FVector& BoundMaxPoint, float CellSize, float CellHeight,
		float WalkableAngle, float AgentHeight, float MinLedgeHeight) const;

protected:
	/** Creates a 2D bounding box that contains the Polygon */
	static bool Generate2DBoundingBoxForGeometry(TArray<FVector>& Polygon, FVector& OutMinimumPoint, FVector& OutMaximumPoint, const FBox& BoundBox);

	/** Attaches the new span into the CurrentSpan */
	void AttachNewSpan(Span* CurrentSpan, Span* NewSpan) const;

	/** Combines the two Spans */
	Span* CombineSpans(Span* LowerSpan, Span* HigherSpan) const;

	/** Returns whether the Polygon provided is walkable */
	bool IsPolygonWalkable(const FVector& PolygonNormal, float MaxWalkableRadians) const;

	/** Returns whether the span provided is walkable */
	bool IsSpanWalkable(const FNNHeightField& HeightField, int32 XIndex, int32 YIndex, const Span* InSpan,
		float AgentHeight, float MinLedgeHeight) const;

	/** Returns the neighbours of the Span provided */
	TArray<Span*> GetSpanNeighbours(const FNNHeightField& HeightField, int32 XIndex, int32 YIndex,
		const Span* CurrentSpan) const;
private:
	/** Data from the AreaGenerator that created this class. Used to debug points and texts in the world */
	FNNAreaGeneratorData& AreaGeneratorData;
};
