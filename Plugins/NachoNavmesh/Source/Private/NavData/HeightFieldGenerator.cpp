#include "NavData/HeightFieldGenerator.h"

// UE Includes
#include "Collision.h"
#include "Chaos/AABB.h"

// NN Includes
#include "NavData/NNAreaGenerator.h"

#define NN_LOG_SPAN_ATTACHMENT 0

FNNHeightField* FHeightFieldGenerator::InitializeHeightField(TArray<FNNRawGeometryElement>& RawGeometry, const FVector& BoundMinPoint, const FVector& BoundMaxPoint, float CellSize, float CellHeight) const
{
	// https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
	// Clip polygons in heightfields

	const FBox Bounds (BoundMinPoint, BoundMaxPoint);
	const int32 XHeightFieldNum = FMath::CeilToInt((BoundMaxPoint.X - BoundMinPoint.X) / CellSize);
	const int32 YHeightFieldNum = FMath::CeilToInt((BoundMaxPoint.Y - BoundMinPoint.Y) / CellSize);
	const int32 ZHeightFieldNum = FMath::CeilToInt((BoundMaxPoint.Z - BoundMinPoint.Z) / CellHeight);

	FNNHeightField* Field = new FNNHeightField(XHeightFieldNum, ZHeightFieldNum, YHeightFieldNum);
	Field->CellHeight = CellHeight;
	Field->CellSize = CellSize;
	Field->MaxPoint = BoundMaxPoint;
	Field->MinPoint = BoundMinPoint;

	for (FNNRawGeometryElement& GeometryElement : RawGeometry)
	{
		const int32 PolygonsNum = GeometryElement.GeomIndices.Num() / 3;
		for (int32 PolygonIndex = 0; PolygonIndex < PolygonsNum; ++PolygonIndex)
		{
			const FVector FirstPoint = GeometryElement.GetGeometryPosition(GeometryElement.GeomIndices[PolygonIndex * 3]);
			const FVector SecondPoint = GeometryElement.GetGeometryPosition(GeometryElement.GeomIndices[PolygonIndex * 3 + 1]);
			const FVector ThirdPoint = GeometryElement.GetGeometryPosition(GeometryElement.GeomIndices[PolygonIndex * 3 + 2]);
			TArray<FVector> Polygon = {FirstPoint, SecondPoint, ThirdPoint};

			// Create a 2D bound box of the current polygon in the X and Y axis
			FVector MinimumPoint;
			FVector MaximumPoint;
			Generate2DBoundingBoxForGeometry(Polygon, MinimumPoint, MaximumPoint, Bounds);

			// Iterate through the necessary cells to check if the polygon intersects with them
			const int32 StartingYSpan = FMath::FloorToInt((MinimumPoint.Y - BoundMinPoint.Y) / CellSize);
			const int32 EndingYSpan = FMath::CeilToInt((MaximumPoint.Y - BoundMinPoint.Y) / CellSize);
			const int32 StartingXSpan = FMath::FloorToInt((MinimumPoint.X - BoundMinPoint.X ) / CellSize);
			const int32 EndingXSpan = FMath::CeilToInt((MaximumPoint.X - BoundMinPoint.X) / CellSize);
			for (int32 j = StartingYSpan; j < EndingYSpan; ++j)
			{
				for (int32 i = StartingXSpan; i < EndingXSpan; ++i)
				{
					for (int32 k = 0; k < ZHeightFieldNum; ++k)
					{

						FVector MinCellPoint = BoundMinPoint;
						MinCellPoint += FVector(i * CellSize, j * CellSize, k * CellHeight);
						FVector MaxCellPoint = MinCellPoint + FVector(CellSize, CellSize, CellHeight);
						FBox Cell (MinCellPoint, MaxCellPoint);
						const FSeparatingAxisPointCheck PointCheck (Polygon, Cell.GetCenter(), Cell.GetExtent());

						// If the polygon intersects with the cell, create a new span
						if (PointCheck.bHit)
						{
							Span* NewSpan = new Span();
							// TODO (ignacio) Check if spawn is walkable
							NewSpan->bWalkable = false;
							NewSpan->MaxSpanHeight = k + 1;
							NewSpan->MinSpanHeight = k;

							// Add the new span in the HeightField
							Span* CurrentSpawn = Field->Spans[i + j * XHeightFieldNum];
#if WITH_EDITOR && NN_LOG_SPAN_ATTACHMENT
							UE_LOG(LogTemp, Warning, TEXT("----------"));
							if (CurrentSpawn)
							{
								UE_LOG(LogTemp, Warning, TEXT("Attaching %s with %s"), *CurrentSpawn->ToString(), *NewSpan->ToString());
							}
#endif
							Field->Spans[i + j * XHeightFieldNum] = AttachNewSpan(CurrentSpawn, NewSpan);
#if WITH_EDITOR && NN_LOG_SPAN_ATTACHMENT
							UE_LOG(LogTemp, Warning, TEXT("Result: %s"), *Field->Spans[i + j * XHeightFieldNum]->ToString())
#endif
						}
					}
				}
			}
		}
	}

	return Field;
}

bool FHeightFieldGenerator::Generate2DBoundingBoxForGeometry(TArray<FVector>& Polygon, FVector& OutMinimumPoint, FVector& OutMaximumPoint, const FBox& BoundBox)
{
	if (Polygon.Num() == 0)
	{
		return false;
	}

	// Copy the vector
	OutMinimumPoint = FVector(Polygon[0]);
	OutMaximumPoint = FVector(Polygon[0]);

	OutMinimumPoint.Z = BoundBox.Min.Z;
	OutMaximumPoint.Z = BoundBox.Min.Z;

	for (int32 i = 1; i < Polygon.Num(); ++i)
	{
		const FVector& Point = Polygon[i];
		if (Point.X < OutMinimumPoint.X)
		{
			OutMinimumPoint.X = Point.X;
		}
		else if (Point.X > OutMaximumPoint.X)
		{
			OutMaximumPoint.X = Point.X;
		}

		if (Point.Y < OutMinimumPoint.Y)
		{
			OutMinimumPoint.Y = Point.Y;
		}
		else if (Point.Y > OutMaximumPoint.Y)
		{
			OutMaximumPoint.Y = Point.Y;
		}
	}

	OutMinimumPoint = BoundBox.GetClosestPointTo(OutMinimumPoint);
	OutMaximumPoint = BoundBox.GetClosestPointTo(OutMaximumPoint);

	return true;
}

Span* FHeightFieldGenerator::AttachNewSpan(Span* CurrentSpan, Span* NewSpan)
{
	// Bug:
	/**
		LogTemp: Warning: Attaching (2, 3)->(4, 6) with (1, 2)
		LogTemp: Warning: Result: (1, 3)
	*/


	if (!CurrentSpan)
	{
		return NewSpan;
	}

	// They are the same Span
	if (CurrentSpan->MaxSpanHeight == NewSpan->MaxSpanHeight && CurrentSpan->MinSpanHeight == NewSpan->MinSpanHeight)
	{
		delete NewSpan;
		return CurrentSpan;
	}

	// The new span is above the current span
	if (CurrentSpan->MinSpanHeight <= NewSpan->MinSpanHeight)
	{
		// They are not colliding
		if (NewSpan->MinSpanHeight > CurrentSpan->MaxSpanHeight)
		{
			CurrentSpan->NextSpan = AttachNewSpan(CurrentSpan->NextSpan, NewSpan);
		}
		// They are colliding. We should combine them
		else
		{
			CombineSpans(CurrentSpan, NewSpan);
		}
		return CurrentSpan;
	}

	// The new span is below the current span

	// They are not colliding
	if (NewSpan->MaxSpanHeight < CurrentSpan->MinSpanHeight)
	{
		NewSpan->NextSpan = CurrentSpan;
	}
	// They are colliding. We should combine them
	else
	{
		CombineSpans(NewSpan, CurrentSpan);
	}
	return NewSpan;
}

Span* FHeightFieldGenerator::CombineSpans(Span* LowerSpan, const Span* HigherSpan)
{
	// The HigherSpan ceil is above the current LowerSpawn
	if (HigherSpan->MaxSpanHeight > LowerSpan->MaxSpanHeight)
	{
		LowerSpan->MaxSpanHeight = HigherSpan->MaxSpanHeight;
		LowerSpan->bWalkable = HigherSpan->bWalkable;

		if (HigherSpan->NextSpan)
		{
			LowerSpan->NextSpan = AttachNewSpan(LowerSpan->NextSpan, HigherSpan->NextSpan);
		}

		// Check if the current span needs to be combined with its next span
		if (LowerSpan->NextSpan)
		{
			Span* NextSpan = LowerSpan->NextSpan;
			LowerSpan->NextSpan = NextSpan->NextSpan;

			AttachNewSpan(LowerSpan, NextSpan);
		}
	}
	// The HigherSpan ceil is in the same height of the LowerSpan
	else if (HigherSpan->MaxSpanHeight == LowerSpan->MaxSpanHeight)
	{
		LowerSpan->bWalkable |= HigherSpan->bWalkable;
	}
	// The HigherSpan is inside the current one we don't do anything
	delete HigherSpan;

	return LowerSpan;
}

void FHeightFieldGenerator::AddDebugPoint(const FVector& Point, float Radius) const
{
	FBoxSphereBounds PointToDebug = FBoxSphereBounds(FSphere(Point, Radius));
	AreaGeneratorData.TemporaryBoxSpheres.Add(MoveTemp(PointToDebug));
}

void FHeightFieldGenerator::AddDebugText(const FVector& Location, const FString& Text) const
{
	AreaGeneratorData.TemporaryTexts.Emplace(Location, Text);
}

