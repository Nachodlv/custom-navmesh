#pragma once

struct FNNAreaGeneratorData;
struct FNNContour;
struct FNNHeightField;
struct FNNRegion;

/** Flags used in the FNNOpenSpan */
enum ENNOpenSpanFlags
{
	None = 0,
	NullRegionChecked = 1 << 0,
};
ENUM_CLASS_FLAGS(ENNOpenSpanFlags);

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
	/** X coordinate in the Spans array of the FNNOpenHeightField. Depth */
	int32 X = INDEX_NONE;
	/** Y coordinate in the Spans array of the FNNOpenHeightField. Width */
	int32 Y = INDEX_NONE;
	/** The OpenSpan in top of this one */
	TUniquePtr<FNNOpenSpan> NextOpenSpan = nullptr;
	/** The distance of this span to an edge */
	int32 EdgeDistance = INDEX_NONE;
	/** The region identifier this span belongs to */
	int32 RegionID = INDEX_NONE;
	int32 Flags = ENNOpenSpanFlags::None;
	/** Each bit represents whether the OpenSpan is connected to another region
	* 0 represents the neighbour is in the same region, 1 is that is not in the same region
	* If it's surrounded by other regions all the bits will be 0 */
	uint8 NeighbourFlags = 0;

	friend bool operator==(const FNNOpenSpan& Lhs, const FNNOpenSpan& Rhs)
	{
		return Lhs.X == Rhs.X && Lhs.Y == Rhs.Y && Lhs.NextOpenSpan == Rhs.NextOpenSpan;
	}

	/** Returns all 8 neighbours of the this span
 	* 0 - 3: Standard axis-neighbour order
 	* 4 - 7: Standard diagonal neighbours. */
	TArray<FNNOpenSpan*> GetDetailedNeighbours() const;
	/** Returns the center world position from the OpenSpan */
	FVector GetOpenSpanWorldPosition(const FNNOpenHeightField& OpenHeightField) const;
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
	int32 AmountOfSpans = 0;
	TArray<TUniquePtr<FNNOpenSpan>> Spans;

	/** The maximum distance from an edge a Span has from the Spans array */
	int32 SpanMaxEdgeDistance = INDEX_NONE;
	/** The regions that this OpenHeightFieldContains */
	TArray<FNNRegion> Regions;

	int32 GetRegionIndexByID(int32 ID) const;
};

struct FNNRegion
{
	FNNRegion(int32 InID) : ID(InID) {}
	int32 ID = INDEX_NONE;
	TArray<FNNOpenSpan*> Spans;
	static FNNRegion GenerateNewRegion();
};

class FOpenHeightFieldGenerator
{
public:
	FOpenHeightFieldGenerator(FNNAreaGeneratorData& InAreaGeneratorData) : AreaGeneratorData(InAreaGeneratorData) {}

	/** Generates a HeightField with the open spaces */
	void GenerateOpenHeightField(FNNOpenHeightField& OutOpenHeightField, const FNNHeightField& SolidHeightField, float MaxLedgeHeight, float AgentHeight) const;

protected:
	/** Get the edge from the OpenSpan to its nearest edge */
	int32 GetOpenSpanEdgeDistance(const FNNOpenSpan* OpenSpan) const;

	/** Sets the OpenSpan neighbours */
	void SetOpenSpanNeighbours(FNNOpenHeightField& OutOpenHeightField, const TArray<FVector2D>& PossibleNeighbours, FNNOpenSpan* OpenSpan, float MaxLedgeHeight, float AgentHeight) const;

private:
	FNNAreaGeneratorData& AreaGeneratorData;
};
