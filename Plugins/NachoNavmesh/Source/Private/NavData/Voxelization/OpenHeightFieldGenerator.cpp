#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

// NN Includes
#include "NavData/Voxelization/HeightFieldGenerator.h"

#define DEBUG_OPENHEIGHTFIELD 0

namespace NNDistanceField
{
	// Represents a border span. It has less than 4 neighbours
	constexpr int32 BorderSpan = 0;

	// Represents an uninitialized non border span
	constexpr int32 NeedsInitSpan = MAX_int32;

	constexpr int32 DefaultDistance = 1;
	constexpr int32 AxisNeighbourDistance = 2;
	constexpr int32 DiagonalNeighbourDistance = 3;

	int32 GetDistanceFromNeighbourFirstPass(const FNNOpenSpan& Neighbour, int32 CurrentDistance, bool bIsDiagonal)
	{
		int32 NeighbourDistance = Neighbour.EdgeDistance;
		if (!bIsDiagonal)
		{
			NeighbourDistance =  NeighbourDistance == NeedsInitSpan ? DefaultDistance : NeighbourDistance + AxisNeighbourDistance;
		}
		else
		{
			NeighbourDistance = NeighbourDistance == NeedsInitSpan ? DefaultDistance + 1 : NeighbourDistance + DiagonalNeighbourDistance;
		}
		return FMath::Min(CurrentDistance, NeighbourDistance);
	}

	int32 GetDistanceFromNeighbourSecondPass(const FNNOpenSpan& Neighbour, int32 CurrentDistance, bool bIsDiagonal)
	{
		const int32 ExtraDistance = bIsDiagonal ? DiagonalNeighbourDistance : AxisNeighbourDistance;
		const int32 NeighbourDistance = Neighbour.EdgeDistance + ExtraDistance;
		return FMath::Min(CurrentDistance, NeighbourDistance);
	}
}

TArray<FNNOpenSpan*> FNNOpenSpan::GetDetailedNeighbours() const
{
	TArray<FNNOpenSpan*> DetailedNeighbours;
	DetailedNeighbours.Init(nullptr, 8);
	for (int32 i = 0; i < Neighbours.Num(); ++i)
	{
		FNNOpenSpan* Neighbour = Neighbours[i];
		if (Neighbour)
		{
			DetailedNeighbours[i] = Neighbour;
			DetailedNeighbours[i + 4] = Neighbour->Neighbours[(i + 1) % 4];
			DetailedNeighbours[((i + 3) % 4) + 4] = Neighbour->Neighbours[(i + 3)  % 4];
		}
	}
	return DetailedNeighbours;
}

FVector FNNOpenSpan::GetOpenSpanWorldPosition(const FNNOpenHeightField& OpenHeightField) const
{
	const float WorldX = X * OpenHeightField.CellSize + OpenHeightField.CellSize / 2;
	const float WorldY = Y * OpenHeightField.CellSize + OpenHeightField.CellSize / 2;
	const float WorldZ = MinHeight * OpenHeightField.CellHeight;
	return OpenHeightField.Bounds.Min + FVector(WorldX, WorldY, WorldZ);
}

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

int32 FNNOpenHeightField::GetRegionIndexByID(int32 ID) const
{
	return Regions.IndexOfByPredicate([ID](const FNNRegion& Region) { return Region.ID == ID; });
}

int32 FNNOpenHeightField::GetSpanMaxEdgeDistance() const
{
	if (SpanMaxEdgeDistance == INDEX_NONE)
	{
		CalculateSpanEdgeDistances();
	}
	return SpanMaxEdgeDistance;
}

int32 FNNOpenHeightField::GetSpanMinEdgeDistance() const
{
	if (SpanMinEdgeDistance == INDEX_NONE)
	{
		CalculateSpanEdgeDistances();
	}
	return SpanMinEdgeDistance;
}

FVector FNNOpenHeightField::TransformVectorToWorldPosition(const FVector& Vector) const
{
	const float X = Vector.X * CellSize;
	const float Y = Vector.Y * CellSize;
	const float Z = Vector.Z * CellHeight;
	return Bounds.Min + FVector(X, Y, Z);
}

FVector FNNOpenHeightField::TransformToHeightFieldPosition(const FVector& Vector) const
{
	FVector Result = Vector - Bounds.Min;
	Result.X /= CellSize;
	Result.Y /= CellSize;
	Result.Z /= CellHeight;
	return Result;
}

void FNNOpenHeightField::CalculateSpanEdgeDistances() const
{
	SpanMaxEdgeDistance = INDEX_NONE;
	SpanMinEdgeDistance = MAX_int32;
	for (const TUniquePtr<FNNOpenSpan>& Span : Spans)
	{
		for (FNNOpenSpan* CurrentSpan = Span.Get(); CurrentSpan; CurrentSpan = CurrentSpan->NextOpenSpan.Get())
		{
			if (SpanMaxEdgeDistance < CurrentSpan->EdgeDistance)
			{
				SpanMaxEdgeDistance = CurrentSpan->EdgeDistance;
			}
			if (SpanMinEdgeDistance > CurrentSpan->EdgeDistance)
			{
				SpanMinEdgeDistance = CurrentSpan->EdgeDistance;
			}
		}
	}
}

FNNOpenHeightFieldIterator::FNNOpenHeightFieldIterator(const FNNOpenHeightField& InOpenHeightField)
	: OpenHeightField(InOpenHeightField)
{
	CurrentSpan = GetNextValidOpenSpan();
}

void FNNOpenHeightFieldIterator::operator++()
{
	CurrentSpan = CurrentSpan->NextOpenSpan.Get();
	if (!CurrentSpan)
	{
		CurrentSpan = GetNextValidOpenSpan();
	}
}

FNNOpenSpan* FNNOpenHeightFieldIterator::GetNextValidOpenSpan()
{
	++CurrentIndex;
	for (; CurrentIndex < OpenHeightField.Spans.Num(); ++CurrentIndex)
	{
		if (OpenHeightField.Spans[CurrentIndex].IsValid())
		{
			return OpenHeightField.Spans[CurrentIndex].Get();
		}
	}
	return nullptr;
}

FNNRegion FNNRegion::GenerateNewRegion()
{
	static int32 UniqueIdentifier = 0;
	FNNRegion Region (UniqueIdentifier);
	++UniqueIdentifier;
	return Region;
}

void FOpenHeightFieldGenerator::GenerateOpenHeightField(FNNOpenHeightField& OutOpenHeightField, const FNNHeightField& SolidHeightField, float MaxLedgeHeight, float AgentHeight) const
{
	OutOpenHeightField = FNNOpenHeightField(SolidHeightField.UnitsWidth, SolidHeightField.UnitsDepth, SolidHeightField.UnitsHeight);
	OutOpenHeightField.CellHeight = SolidHeightField.CellHeight;
	OutOpenHeightField.CellSize = SolidHeightField.CellSize;
	OutOpenHeightField.Bounds = FBox(SolidHeightField.MinPoint, SolidHeightField.MaxPoint);

	// Create the open spans
	for (int32 i = 0; i < SolidHeightField.Spans.Num(); ++i)
	{
		const int32 X = (i % SolidHeightField.UnitsWidth);
		const int32 Y = (i / SolidHeightField.UnitsWidth);
		const int32 Index = X + Y * SolidHeightField.UnitsWidth;
		FNNOpenSpan* LastOpenSpan = nullptr;
		Span* Span = SolidHeightField.Spans[i].Get();
		while (Span)
		{
			if (Span->bWalkable)
			{
				const int32 MinHeight = Span->MaxSpanHeight;
				const int32 MaxHeight = Span->NextSpan ? Span->NextSpan->MinSpanHeight : TNumericLimits<int32>::Max();
				if (LastOpenSpan)
				{
					LastOpenSpan->NextOpenSpan = MakeUnique<FNNOpenSpan>(MinHeight, MaxHeight, X, Y);
					LastOpenSpan = LastOpenSpan->NextOpenSpan.Get();
				}
				else
				{
					OutOpenHeightField.Spans[Index] = MakeUnique<FNNOpenSpan>(MinHeight, MaxHeight, X, Y);
					LastOpenSpan = OutOpenHeightField.Spans[Index].Get();
				}
				++OutOpenHeightField.AmountOfSpans;
			}
			Span = Span->NextSpan.Get();
		}
	}

	// Add linked neighbours to the open spans
	const TArray<FVector2D> PossibleNeighbours = {FVector2D(-1, 0), FVector2D(0, 1), FVector2D(1, 0), FVector2D(0, -1)};
	for (int32 i = 0; i < OutOpenHeightField.Spans.Num(); ++i)
	{
		FNNOpenSpan* OpenSpan = OutOpenHeightField.Spans[i].Get();
		while (OpenSpan)
		{
			SetOpenSpanNeighbours(OutOpenHeightField, PossibleNeighbours, OpenSpan, MaxLedgeHeight, AgentHeight);
			OpenSpan = OpenSpan->NextOpenSpan.Get();
		}
	}

	GenerateDistanceField(OutOpenHeightField);

#if DEBUG_OPENHEIGHTFIELD
	for (int32 i = 0; i < OutOpenHeightField.Spans.Num(); ++i)
	{
		FNNOpenSpan* OpenSpan = OutOpenHeightField.Spans[i].Get();
		while (OpenSpan)
		{
			// Debug distances
			const FVector OpenSpanPosition = OpenSpan->GetOpenSpanWorldPosition(OutOpenHeightField);
			AreaGeneratorData.AddDebugText(OpenSpanPosition, FString::FromInt(OpenSpan->EdgeDistance));
			OpenSpan = OpenSpan->NextOpenSpan.Get();
		}
	}
#endif // DEBUG_OPENHEIGHTFIELD_DISTANCE
}

void FOpenHeightFieldGenerator::SetOpenSpanNeighbours(FNNOpenHeightField& OutOpenHeightField, const TArray<FVector2D>& PossibleNeighbours, FNNOpenSpan* OpenSpan, float MaxLedgeHeight, float AgentHeight) const
{
	for (int32 NeighbourIndex = 0; NeighbourIndex < PossibleNeighbours.Num(); ++NeighbourIndex)
	{
		const FVector2D& Neighbour = PossibleNeighbours[NeighbourIndex];
		const int32 X = OpenSpan->X + Neighbour.X;
		const int32 Y = OpenSpan->Y + Neighbour.Y;
		FNNOpenSpan* NeighbourSpan = OutOpenHeightField.Spans[X + Y * OutOpenHeightField.UnitsWidth].Get();
		FNNOpenSpan* NearestNeighbourSpan = nullptr;
		int32 NearestDistance = INDEX_NONE;
		while (NeighbourSpan)
		{
			// Head Bonk Test
			bool bWillHeadBonk = false;
			int32 SpaceBetween = FMath::Abs(NeighbourSpan->MaxHeight - OpenSpan->MinHeight);
			// We need to check both ways
			SpaceBetween = FMath::Min(SpaceBetween, FMath::Abs(OpenSpan->MaxHeight - NeighbourSpan->MinHeight));
			if (SpaceBetween * OutOpenHeightField.CellHeight < AgentHeight)
			{
				bWillHeadBonk = true;
			}

			if (!bWillHeadBonk)
			{
				const int32 Distance = FMath::Abs(NeighbourSpan->MinHeight - OpenSpan->MinHeight);
				if (Distance * OutOpenHeightField.CellHeight < MaxLedgeHeight)
				{
					if ((NearestDistance == INDEX_NONE || Distance < NearestDistance))
					{
						NearestDistance = Distance;
						NearestNeighbourSpan = NeighbourSpan;
					}
					else
					{
						// The distance will only continue to increase
						break;
					}
				}
			}

			NeighbourSpan = NeighbourSpan->NextOpenSpan.Get();
		}
		OpenSpan->Neighbours[NeighbourIndex] = NearestNeighbourSpan;

#if DEBUG_OPENHEIGHTFIELD
		// Debug neighbours
		if (NearestNeighbourSpan)
		{
			FVector CurrentSpanLocation = OpenSpan->GetOpenSpanWorldPosition(OutOpenHeightField);
			FVector NeighbourLocation =NearestNeighbourSpan-> GetOpenSpanWorldPosition(OutOpenHeightField);
			AreaGeneratorData.AddDebugLine(CurrentSpanLocation, NeighbourLocation);
		}
#endif // DEBUG_OPENHEIGHTFIELD_DISTANCE
	}
}

void FOpenHeightFieldGenerator::GenerateDistanceField(FNNOpenHeightField& OpenHeightField) const
{
	if (OpenHeightField.Spans.Num() == 0)
	{
		return;
	}

	// Initialization
	for (TUniquePtr<FNNOpenSpan>& OpenSpan : OpenHeightField.Spans)
	{
		for (FNNOpenSpan* CurrentSpan = OpenSpan.Get(); CurrentSpan; CurrentSpan = CurrentSpan->NextOpenSpan.Get())
		{
			bool bIsBorder = false;
			for (int32 i = 0; i < 4; ++i)
			{
				const FNNOpenSpan* Neighbour = CurrentSpan->Neighbours[i];
				// If the axis neighbour or the diagonal neighbour is null then this is a border span
				if (!Neighbour || !Neighbour->Neighbours[(i + 1) % 4])
				{
					bIsBorder = true;
					break;
				}
			}
			CurrentSpan->EdgeDistance = bIsBorder ? NNDistanceField::BorderSpan : NNDistanceField::NeedsInitSpan;
		}
	}

	// Pass 1 the following neighbours will be checked: (-1, 0), (-1, -1), (0, -1), (1, -1)
	for (TUniquePtr<FNNOpenSpan>& OpenSpan : OpenHeightField.Spans)
	{
		for (FNNOpenSpan* CurrentSpan = OpenSpan.Get(); CurrentSpan; CurrentSpan = CurrentSpan->NextOpenSpan.Get())
		{
			int32 SpanDistance = CurrentSpan->EdgeDistance;
			if (SpanDistance == NNDistanceField::BorderSpan)
			{
				continue;
			}

			// (-1, 0)
			FNNOpenSpan* Neighbour = CurrentSpan->Neighbours[0];
			SpanDistance = NNDistanceField::GetDistanceFromNeighbourFirstPass(*Neighbour, SpanDistance, false);

			// (-1, -1)
			Neighbour = Neighbour->Neighbours[3];
			if (Neighbour)
			{
				SpanDistance = NNDistanceField::GetDistanceFromNeighbourFirstPass(*Neighbour, SpanDistance, true);
			}

			// (0, -1)
			Neighbour = CurrentSpan->Neighbours[3];
			SpanDistance = NNDistanceField::GetDistanceFromNeighbourFirstPass(*Neighbour, SpanDistance, false);

			// (1, -1)
			Neighbour = Neighbour->Neighbours[2];
			if (Neighbour)
			{
				SpanDistance = NNDistanceField::GetDistanceFromNeighbourFirstPass(*Neighbour, SpanDistance, true);
			}

			CurrentSpan->EdgeDistance = SpanDistance;
		}
	}

	// Pass 2. Neighbours checked (1, 0), (1, 1), (0, 1), (-1, 1)
	// Don't need to handle the NeedsInits special case
	for (int32 i = OpenHeightField.Spans.Num() - 1; i >= 0; --i)
	{
		TUniquePtr<FNNOpenSpan>& OpenSpan = OpenHeightField.Spans[i];
		for (FNNOpenSpan* CurrentSpan = OpenSpan.Get(); CurrentSpan; CurrentSpan = CurrentSpan->NextOpenSpan.Get())
		{
			int32 SpanDistance = CurrentSpan->EdgeDistance;
			if (SpanDistance == NNDistanceField::BorderSpan)
			{
				continue;
			}

			// (1, 0)
			FNNOpenSpan* Neighbour = CurrentSpan->Neighbours[2];
			SpanDistance = NNDistanceField::GetDistanceFromNeighbourSecondPass(*Neighbour, SpanDistance, false);

			// (1, 1)
			Neighbour = Neighbour->Neighbours[1];
			if (Neighbour)
			{
				SpanDistance = NNDistanceField::GetDistanceFromNeighbourSecondPass(*Neighbour, SpanDistance, true);
			}

			// (0, 1)
			Neighbour = CurrentSpan->Neighbours[1];
			SpanDistance = NNDistanceField::GetDistanceFromNeighbourSecondPass(*Neighbour, SpanDistance, false);

			// (-1, 1)
			Neighbour = Neighbour->Neighbours[0];
			if (Neighbour)
			{
				SpanDistance = NNDistanceField::GetDistanceFromNeighbourSecondPass(*Neighbour, SpanDistance, true);
			}

			CurrentSpan->EdgeDistance = SpanDistance;
		}
	}
}



