#pragma once

struct FNNAreaGeneratorData;
struct FNNHeightField;

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
	int32 EdgeDistance = INDEX_NONE;
};

class FOpenHeightFieldGenerator
{
public:
	FOpenHeightFieldGenerator(FNNAreaGeneratorData& InAreaGeneratorData) : AreaGeneratorData(InAreaGeneratorData) {}

	/** Generates a HeightField with the open spaces */
	void GenerateOpenHeightField(FNNOpenHeightField& OutOpenHeightField, const FNNHeightField& SolidHeightField) const;

private:
	FNNAreaGeneratorData& AreaGeneratorData;
};
