#include "NavData/HeightFieldGenerator.h"

// UE Includes
#include "Collision.h"
#include "Chaos/AABB.h"
#include "Kismet/KismetMathLibrary.h"

// NN Includes
#include "NavData/NNAreaGenerator.h"
#include "NavData/NNNavMeshHelper.h"

#define NN_LOG_SPAN_ATTACHMENT 0

FNNHeightField* FHeightFieldGenerator::InitializeHeightField(TArray<FNNRawGeometryElement>& RawGeometry, const FVector& BoundMinPoint, const FVector& BoundMaxPoint, float CellSize, float CellHeight, float WalkableAngle, float AgentHeight, float MinLedgeHeight) const
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

	const float WalkableRadians = FMath::DegreesToRadians(WalkableAngle);

	for (FNNRawGeometryElement& GeometryElement : RawGeometry)
	{
		const int32 PolygonsNum = GeometryElement.GeomIndices.Num() / 3;
		for (int32 PolygonIndex = 0; PolygonIndex < PolygonsNum; ++PolygonIndex)
		{
			const FVector FirstPoint = GeometryElement.GetGeometryPosition(GeometryElement.GeomIndices[PolygonIndex * 3]);
			const FVector SecondPoint = GeometryElement.GetGeometryPosition(GeometryElement.GeomIndices[PolygonIndex * 3 + 1]);
			const FVector ThirdPoint = GeometryElement.GetGeometryPosition(GeometryElement.GeomIndices[PolygonIndex * 3 + 2]);
			TArray<FVector> Polygon = {FirstPoint, SecondPoint, ThirdPoint};
			FVector PolygonNormal = FNNNavMeshHelper::CalculatePolygonNormal(Polygon);

			FVector PolygonCenter = UKismetMathLibrary::GetVectorArrayAverage(Polygon);
			// AddDebugLine(PolygonCenter, PolygonCenter + PolygonNormal * 50.0f);

			const bool bPolygonWalkable  = IsPolygonWalkable(PolygonNormal, WalkableRadians);

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
							Span NewSpan;
							NewSpan.bWalkable = bPolygonWalkable;
							NewSpan.MaxSpanHeight = k + 1;
							NewSpan.MinSpanHeight = k;

							// Add the new span in the HeightField
							Span* CurrentSpan = Field->Spans[i + j * XHeightFieldNum].Get();
#if WITH_EDITOR && NN_LOG_SPAN_ATTACHMENT
							UE_LOG(LogTemp, Warning, TEXT("----------"));
							if (CurrentSpan)
							{
								UE_LOG(LogTemp, Warning, TEXT("Attaching %s with %s"), *CurrentSpan->ToString(), *NewSpan.ToString());
							}
#endif
							if (CurrentSpan)
							{
								AttachNewSpan(CurrentSpan, &NewSpan);
							}
							else
							{
								Field->Spans[i + j * XHeightFieldNum] = MakeUnique<Span>(NewSpan);
							}
							// AddDebugText(Cell.GetCenter(), FString::FromInt(i + j * XHeightFieldNum));
							// AddDebugText(Cell.GetCenter(), FString::Printf(TEXT("(%d, %d)"), i , j));
#if WITH_EDITOR && NN_LOG_SPAN_ATTACHMENT
							UE_LOG(LogTemp, Warning, TEXT("Result: %s"), *Field->Spans[i + j * XHeightFieldNum]->ToString())
#endif
						}
					}
				}
			}
		}
	}

	for (int32 i = 0; i < Field->Spans.Num(); ++i)
	{
		Span* CurrentSpan = Field->Spans[i].Get();
		while (CurrentSpan)
		{
			const int32 Y = (i / Field->UnitsWidth);
			const int32 X = (i % Field->UnitsWidth);
			CurrentSpan->bWalkable = IsSpanWalkable(Field, X, Y, CurrentSpan, AgentHeight, MinLedgeHeight);
			CurrentSpan = CurrentSpan->NextSpan.Get();
		}
	}

	return Field;
}

bool FHeightFieldGenerator::IsPolygonWalkable(const FVector& PolygonNormal, float MaxWalkableRadians) const
{
	// const FVector FloorZAxis = PolygonNormal;
	// const FVector FloorXAxis = FVector::RightVector ^ FloorZAxis;
	// const FVector FloorYAxis = FloorZAxis ^ FloorXAxis;
	// const float Pitch = FMath::Acos(FloorXAxis | FVector::UpVector);
	// const float Roll = FMath::Acos(FloorYAxis | FVector::UpVector);
	//
	// UE_LOG(LogTemp, Warning, TEXT("Pitch: %f, Rolls :%f"), Pitch, Roll);
	//
	// return Pitch > MaxWalkableRadians && Roll > MaxWalkableRadians;

	const float Rotation = FMath::Acos(FVector::DotProduct(PolygonNormal, FVector::UpVector));

	return Rotation < MaxWalkableRadians;
}

bool FHeightFieldGenerator::IsSpanWalkable(const FNNHeightField* HeightField, int32 XIndex, int32 YIndex, const Span* InSpan, float  AgentHeight, float MinLedgeHeight) const
{
	if (!InSpan->bWalkable)
	{
		return false;
	}

	// Check if there is enough space in top of span so the agent can step on it
	if (InSpan->NextSpan)
	{
		const int32 MaxSpanHeight = InSpan->MaxSpanHeight;
		const int32 MinNextSpanHeight = InSpan->NextSpan->MinSpanHeight;
		const float SpaceBetweenSpans = (MinNextSpanHeight - MaxSpanHeight) * HeightField->CellHeight;
		if (SpaceBetweenSpans < AgentHeight)
		{
			return false;
		}
	}

	// Check if the span is a ledge by checking the height of its neighbours
	const TArray<TUniquePtr<Span>>& Spans = HeightField->Spans;
	TArray<Span*> Neighbours = GetSpanNeighbours(HeightField, XIndex, YIndex, InSpan);

	// UE_LOG(LogTemp, Warning, TEXT("\n---------"));
	// UE_LOG(LogTemp, Warning, TEXT("%d neighbours are: "), XIndex + YIndex * HeightField->UnitsWidth);

	for (const Span* Neighbour : Neighbours)
	{
		// If its invalid it means the span is in the border of the HeightField
		// Should we consider it as ledge?
		float HeightDifference = 0.0f;
		if (Neighbour)
		{
			HeightDifference = FMath::Abs(Neighbour->MaxSpanHeight - InSpan->MaxSpanHeight) * HeightField->CellHeight;
		}
		else
		{
			HeightDifference = InSpan->MaxSpanHeight * HeightField->CellHeight;
		}
		if (HeightDifference > MinLedgeHeight)
		{
			// UE_LOG(LogTemp, Warning, TEXT("%d - FALSE"), NeighbourIndex);
			return false;
		}
		// UE_LOG(LogTemp, Warning, TEXT("%d - TRUE"), NeighbourIndex);
	}

	return true;
}


TArray<Span*> FHeightFieldGenerator::GetSpanNeighbours(const FNNHeightField* HeightField, int32 XIndex, int32 YIndex, const Span* CurrentSpan) const
{
	TArray<int32> NeighboursIndexes;
	NeighboursIndexes.Init(INDEX_NONE, 4);
	if (XIndex < HeightField->UnitsDepth - 1)
	{
		NeighboursIndexes[0] = XIndex + 1 + YIndex * HeightField->UnitsWidth;
	}
	if (XIndex >= 1)
	{
		NeighboursIndexes[1] = XIndex - 1 + YIndex * HeightField->UnitsWidth;
	}
	if (YIndex < HeightField->UnitsDepth - 1)
	{
		NeighboursIndexes[2] = XIndex + (YIndex + 1) * HeightField->UnitsWidth;
	}
	if (YIndex >= 1)
	{
		NeighboursIndexes[3] = XIndex + (YIndex - 1) * HeightField->UnitsWidth;
	}
	TArray<Span*> Neighbours;
	Neighbours.Init(nullptr, 4);
	for (int32 i = 0; i < 4; ++i)
	{
		int32 NeighbourIndex = NeighboursIndexes[i];

		// If its invalid it means the span is in the border of the HeightField
		// Should we consider it as ledge?
		if (NeighbourIndex >= 0 && NeighbourIndex < HeightField->Spans.Num())
		{
			Span* BestNeighbourSpan = HeightField->Spans[NeighbourIndex].Get();
			if (!BestNeighbourSpan)
			{
				continue;
			}

			int32 MinorDifference = FMath::Abs(BestNeighbourSpan->MaxSpanHeight - CurrentSpan->MaxSpanHeight);
			Span* NextSpan = BestNeighbourSpan->NextSpan.Get();
			// Search of the nearest span of the CurrentSpan
			while (NextSpan)
			{
				const int32 NextDifference = FMath::Abs(NextSpan->MaxSpanHeight - CurrentSpan->MaxSpanHeight);
				if (NextDifference < MinorDifference)
				{
					BestNeighbourSpan = NextSpan;
					MinorDifference = NextDifference;
					NextSpan = NextSpan->NextSpan.Get();
				}
				else
				{
					// There is no need to continue checking, the difference will keep growing
					break;
				}
			}
			Neighbours[i] = BestNeighbourSpan;
		}
	}

	return Neighbours;
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

void FHeightFieldGenerator::AttachNewSpan(Span* CurrentSpan, Span* NewSpan) const
{
	// They are the same Span
	if (CurrentSpan->MaxSpanHeight == NewSpan->MaxSpanHeight && CurrentSpan->MinSpanHeight == NewSpan->MinSpanHeight)
	{
		// I think this should be an &= but i have problems with planes
		CurrentSpan->bWalkable |= NewSpan->bWalkable;
	}

	// The new span is above the current span
	if (CurrentSpan->MinSpanHeight <= NewSpan->MinSpanHeight)
	{
		// They are not colliding
		if (NewSpan->MinSpanHeight > CurrentSpan->MaxSpanHeight)
		{
			if (CurrentSpan->NextSpan)
			{
				AttachNewSpan(CurrentSpan->NextSpan.Get(), NewSpan);
			}
			else
			{
				CurrentSpan->NextSpan = MakeUnique<Span>(*NewSpan);
			}
		}
		// They are colliding. We should combine them
		else
		{
			CombineSpans(CurrentSpan, NewSpan);
		}
	}
	else
	{
		// The new span is below the current span

		// They are not colliding
		if (NewSpan->MaxSpanHeight < CurrentSpan->MinSpanHeight)
		{
			NewSpan->NextSpan = MakeUnique<Span>(*CurrentSpan);
		}
		// They are colliding. We should combine them
		else
		{
			CombineSpans(NewSpan, CurrentSpan);
		}

		CurrentSpan->CopySpan(*NewSpan);
	}
}

Span* FHeightFieldGenerator::CombineSpans(Span* LowerSpan, Span* HigherSpan) const
{
	// The HigherSpan ceil is above the current LowerSpawn
	if (HigherSpan->MaxSpanHeight > LowerSpan->MaxSpanHeight)
	{
		LowerSpan->MaxSpanHeight = HigherSpan->MaxSpanHeight;
		LowerSpan->bWalkable = HigherSpan->bWalkable;

		if (HigherSpan->NextSpan)
		{
			if (LowerSpan->NextSpan)
			{
				AttachNewSpan(LowerSpan->NextSpan.Get(), HigherSpan->NextSpan.Get());
			}
			else
			{
				LowerSpan->NextSpan = MakeUnique<Span>(*HigherSpan->NextSpan.Release());
				HigherSpan->NextSpan = nullptr;
			}
		}

		// Check if the current span needs to be combined with its next span
		if (LowerSpan->NextSpan)
		{
			Span NextSpan (*LowerSpan->NextSpan);
			if (NextSpan.NextSpan)
			{
				LowerSpan->NextSpan->CopySpan(*NextSpan.NextSpan);
			}
			else
			{
				LowerSpan->NextSpan = nullptr;
			}
			AttachNewSpan(LowerSpan, &NextSpan);
		}
	}
	// The HigherSpan ceil is in the same height of the LowerSpan
	else if (HigherSpan->MaxSpanHeight == LowerSpan->MaxSpanHeight)
	{
		LowerSpan->bWalkable |= HigherSpan->bWalkable;
	}

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

void FHeightFieldGenerator::AddDebugLine(const FVector& Start, const FVector& End) const
{
	AreaGeneratorData.TemporaryLines.Emplace(Start, End, FColor::Blue, 2.0f);
}

