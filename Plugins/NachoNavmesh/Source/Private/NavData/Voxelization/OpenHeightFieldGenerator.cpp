#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

#include "NavData/Voxelization/HeightFieldGenerator.h"


FNNOpenHeightField::FNNOpenHeightField(int32 InUnitsWidth, int32 InUnitsDepth, int32 InUnitsHeight)
	: UnitsWidth(InUnitsWidth), UnitsDepth(InUnitsDepth), UnitsHeight(InUnitsHeight)
{
	const int32 Num = UnitsWidth * UnitsDepth;
	Spans.Reserve(Num);
	for (int32 i = 0; i < Num; ++i)
	{
		Spans.Add(nullptr);
	}
}

void FOpenHeightFieldGenerator::GenerateOpenHeightField(FNNOpenHeightField& OutOpenHeightField, const FNNHeightField& SolidHeightField) const
{
	OutOpenHeightField = FNNOpenHeightField(SolidHeightField.UnitsWidth, SolidHeightField.UnitsDepth, SolidHeightField.UnitsHeight);
	OutOpenHeightField.CellHeight = SolidHeightField.CellHeight;
	OutOpenHeightField.CellSize = SolidHeightField.CellSize;
	OutOpenHeightField.Bounds = FBox(SolidHeightField.MinPoint, SolidHeightField.MaxPoint);

	for (int32 i = 0; i < SolidHeightField.Spans.Num(); ++i)
	{
		const int32 X = (i % SolidHeightField.UnitsWidth);
		const int32 Y = (i / SolidHeightField.UnitsWidth);
		const int32 Index = X + Y * SolidHeightField.UnitsWidth;
		Span* Span = SolidHeightField.Spans[i].Get();
		while (Span)
		{
			if (Span->bWalkable)
			{
				const int32 MinHeight = Span->MaxSpanHeight;
				const int32 MaxHeight = Span->NextSpan ? Span->NextSpan->MinSpanHeight : INDEX_NONE;
				if (FNNOpenSpan* LastOpenSpan = OutOpenHeightField.Spans[Index].Get())
				{
					LastOpenSpan->NextOpenSpan = MakeUnique<FNNOpenSpan>(MinHeight, MaxHeight, X, Y);
				}
				else
				{
					OutOpenHeightField.Spans[Index] = MakeUnique<FNNOpenSpan>(MinHeight, MaxHeight, X, Y);
				}
			}
			Span = Span->NextSpan.Get();
		}

	}
}
