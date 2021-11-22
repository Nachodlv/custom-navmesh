#pragma once

struct FNNAreaGeneratorData;
struct FNNHeightField;
struct FNNRegion;

/** A cell representing a open space */
struct FNNOpenSpan
{
	FNNOpenSpan(int32 InMinHeight, int32 InMaxHeight, int32 InX, int32 InY)
		: MinHeight(InMinHeight), MaxHeight(InMaxHeight), X(InX), Y(InY)
	{
		Neighbours.Init(nullptr, 4);
	}
	FNNOpenSpan(const FNNOpenSpan& InSpan) = delete;

	int32 MinHeight = INDEX_NONE;
	int32 MaxHeight = INDEX_NONE;
	TArray<FNNOpenSpan*> Neighbours; // 4 neighbours (-1, 0), (0, 1), (1, 0), (0, -1)
	/** X coordinate in the Spans array of the FNNOpenHeightField */
	int32 X = INDEX_NONE;
	/** Y coordinate in the Spans array of the FNNOpenHeightField */
	int32 Y = INDEX_NONE;
	/** The OpenSpan in top of this one */
	TUniquePtr<FNNOpenSpan> NextOpenSpan = nullptr;
	/** The distance of this span to an edge */
	int32 EdgeDistance = INDEX_NONE;
	/** The region identifier this span belongs to */
	int32 RegionID = INDEX_NONE;
};

struct FNNOpenHeightField
{
	FNNOpenHeightField() {}
	FNNOpenHeightField(int32 InUnitsWidth, int32 InUnitsDepth, int32 InUnitsHeight);
	int32 UnitsWidth = INDEX_NONE;
	int32 UnitsDepth = INDEX_NONE;
	int32 UnitsHeight = INDEX_NONE;
	FBox Bounds;
	float CellSize = 0.0f;
	float CellHeight = 0.0f;
	TArray<TUniquePtr<FNNOpenSpan>> Spans;

	/** The maximum distance from an edge a Span has from the Spans array */
	int32 SpanMaxEdgeDistance = INDEX_NONE;
	/** The regions that this OpenHeightFieldContains */
	TArray<FNNRegion> Regions;
};

struct FNNRegion
{
	FNNRegion(int32 InID) : ID(InID) {}
	int32 ID = INDEX_NONE;
	TArray<FNNOpenSpan*> Spans;
};

class FOpenHeightFieldGenerator
{
public:
	FOpenHeightFieldGenerator(FNNAreaGeneratorData& InAreaGeneratorData) : AreaGeneratorData(InAreaGeneratorData) {}

	/** Generates a HeightField with the open spaces */
	void GenerateOpenHeightField(FNNOpenHeightField& OutOpenHeightField, const FNNHeightField& SolidHeightField, float MaxLedgeHeight, float AgentHeight, float MinRegionSize) const;

protected:
	/** Get the edge from the OpenSpan to its nearest edge */
	int32 GetOpenSpanEdgeDistance(const FNNOpenSpan* OpenSpan) const;

	/** Returns the center world position from the OpenSpan */
	static FVector GetOpenSpanWorldPosition(const FNNOpenSpan* OpenSpan, const FNNOpenHeightField& OpenHeightField);

	/** Sets the OpenSpan neighbours */
	void SetOpenSpanNeighbours(FNNOpenHeightField& OutOpenHeightField, const TArray<FVector2D>& PossibleNeighbours, FNNOpenSpan* OpenSpan, float MaxLedgeHeight, float AgentHeight) const;

	/** Initializes the regions of the open spans with a watershed algorithm */
	void CreateRegions(FNNOpenHeightField& OpenHeightField, int32 MinSpansForRegions) const;

	/** Sets the region to the CurrentSpan neighbours with the same WaterLevel */
	void FloodRegion(FNNOpenSpan* CurrentSpan, FNNRegion& Region, int32 CurrentWaterLevel) const;

	/** Grows the current regions created equally */
	void GrowRegions(FNNOpenHeightField& OpenHeightField, int32 CurrentWaterLevel) const;

	/** Deletes or combines regions with less than te MinSpansForRegions */
	void FilterSmallRegions(TArray<FNNRegion>& Regions, int32 MinSpansForRegions) const;

private:
	FNNAreaGeneratorData& AreaGeneratorData;
};
