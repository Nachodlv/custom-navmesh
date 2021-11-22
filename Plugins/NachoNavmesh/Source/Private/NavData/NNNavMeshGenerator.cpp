#include "NavData/NNNavMeshGenerator.h"

// UE Includes
#include "Kismet/KismetMathLibrary.h"

// NN Includes
#include "NavData/NNNavMeshRenderingComp.h"
#include "NavData/Voxelization/HeightFieldGenerator.h"
#include "NavData/Voxelization/OpenHeightFieldGenerator.h"


FNNNavMeshGenerator::FNNNavMeshGenerator(ANNNavMesh& InNavMesh)
	: NavMesh(&InNavMesh), NavBounds(InNavMesh.GetRegisteredBounds())
{}

void FNNNavMeshGenerator::TickAsyncBuild(float DeltaSeconds)
{
	ProcessDirtyAreas();
}

bool FNNNavMeshGenerator::RebuildAll()
{
	for (const FNavigationBounds& NavBound : NavBounds)
	{
		DirtyAreas.AddUnique(NavBound.UniqueID);
	}
	for (const auto& Data : GeneratorsData)
	{
		delete Data.Value;
	}
	GeneratorsData.Reset();
	return true;
}

void FNNNavMeshGenerator::RebuildDirtyAreas(const TArray<FNavigationDirtyArea>& NavigationDirtyAreas)
{
	// TODO (ignacio) I probably want to rebuild only the dirty areas
	RebuildAll();
}

FNNAreaGenerator* FNNNavMeshGenerator::CreateAreaGenerator(const FNavigationBounds& DirtyArea)
{
	return new FNNAreaGenerator(this, DirtyArea);
}

void FNNNavMeshGenerator::ProcessDirtyAreas()
{
	if (DirtyAreas.Num() == 0)
	{
		return;
	}

	for (int32 i = DirtyAreas.Num() - 1; i >= 0; --i)
	{
		FNavigationBounds DirtyAreaSearch;
		DirtyAreaSearch.UniqueID = DirtyAreas[i];
		const FNavigationBounds* DirtyArea = NavBounds.Find(DirtyAreaSearch);
		// The area might have been deleted
		if (DirtyArea)
		{
			FNNAreaGenerator* AreaGenerator = CreateAreaGenerator(*DirtyArea);
			AreaGenerator->DoWork();

			// TODO (ignacio) is it necessary to delete the data if it's going to be overriden?
			// Does the TMap does this for use?
			if (FNNAreaGeneratorData** Data = GeneratorsData.Find(DirtyAreas[i]))
			{
				delete *Data;
			}
			GeneratorsData.Add(DirtyAreas[i], AreaGenerator->GetAreaGeneratorData());
			delete AreaGenerator;
		}
	}
	Cast<UNNNavMeshRenderingComp>(NavMesh->RenderingComp)->ForceUpdate();
	DirtyAreas.Reset();
}

FBox FNNNavMeshGenerator::GrowBoundingBox(const FBox& BBox, bool bUseAgentHeight) const
{
	FVector BBoxGrowOffsetMin = FVector(0.0f);
	if (bUseAgentHeight && GetOwner().IsValid())
	{
		BBoxGrowOffsetMin += FVector(0.0f, 0.0f, GetOwner()->AgentHeight);
	}

	return FBox(BBox.Min - BBoxGrowth - BBoxGrowOffsetMin, BBox.Max + BBoxGrowth);
}

void FNNNavMeshGenerator::GrabDebuggingInfo(FNNNavMeshDebuggingInfo& DebuggingInfo) const
{
	for (const auto& Result : GeneratorsData)
	{
		DebuggingInfo.RawGeometryToDraw = Result.Value->RawGeometry;
		DebuggingInfo.TemporaryBoxSpheres.Append(Result.Value->TemporaryBoxSpheres);
		DebuggingInfo.TemporaryTexts.Append(Result.Value->TemporaryTexts);
		DebuggingInfo.TemporaryLines.Append(Result.Value->TemporaryLines);

		const TArray<TUniquePtr<Span>>& Spans = Result.Value->HeightField->Spans;

		// TODO (ignacio) this can be moved to a function
		FNavigationBounds DataSearch;
		DataSearch.UniqueID = Result.Key;
		const FNavigationBounds* GeneratorArea = NavBounds.Find(DataSearch);
		const FVector BoundMinPoint = GeneratorArea->AreaBox.Min;

		// Converts the HeightField Spans into FBoxes
		const float CellSize = Result.Value->HeightField->CellSize;
		const float CellHeight = Result.Value->HeightField->CellHeight;
		const int32 UnitsWidth = Result.Value->HeightField->UnitsWidth;
		for (int32 i = 0; i < Spans.Num(); ++i)
		{
			const float Y = (i / UnitsWidth) * CellSize;
			const float X = (i % UnitsWidth) * CellSize;
			const Span* CurrentSpan = Spans[i].Get();
			while (CurrentSpan)
			{
				const float MinZ = CurrentSpan->MinSpanHeight * CellHeight;
				const float MaxZ = CurrentSpan->MaxSpanHeight * CellHeight;
				FVector MinPoint = BoundMinPoint;
				MinPoint += FVector(X, Y, MinZ);
				FVector MaxPoint = BoundMinPoint + FVector(X + CellSize, Y + CellSize, MaxZ);

				FNNNavMeshDebuggingInfo::HeightFieldDebugBox DebugBox;
				DebugBox.Box = FBox(MinPoint, MaxPoint);
				DebugBox.Color = CurrentSpan->bWalkable ? FColor::Green : FColor::Red;
				DebuggingInfo.HeightField.Add(MoveTemp(DebugBox));

				CurrentSpan = CurrentSpan->NextSpan.Get();
			}
		}

		// Converts the OpenHeightField Spans into FBoxes
		if (Result.Value->OpenHeightField->Spans.Num() > 0)
		{
			TArray<TUniquePtr<FNNOpenSpan>>& OpenSpans = Result.Value->OpenHeightField->Spans;

			int32 MaxDistance = INDEX_NONE;
			int32 MinDistance = 1; // There is always a span with distance 0
			for (TUniquePtr<FNNOpenSpan>& OpenSpan : OpenSpans)
			{
				if (OpenSpan)
				{
					if (MaxDistance < OpenSpan->EdgeDistance)
					{
						MaxDistance = OpenSpan->EdgeDistance;
					}
					else if (MinDistance > OpenSpan->EdgeDistance)
					{
						MinDistance = OpenSpan->EdgeDistance;
					}
				}
			}

			FLinearColor MaxColor = FLinearColor::Red;
			FLinearColor MinColor = FLinearColor::Green;
			float MaxHeight = Result.Value->OpenHeightField->Bounds.Max.Z;
			for (int32 i = 0; i < OpenSpans.Num(); ++i)
			{
				FNNOpenSpan* OpenSpan = OpenSpans[i].Get();
				while (OpenSpan)
				{
					const int32 X = OpenSpan->X * CellSize;
					const int32 Y = OpenSpan->Y * CellSize;
					const float MinZ = OpenSpan->MinHeight * CellHeight;
					const float MaxZ = OpenSpan->MaxHeight  * CellHeight < MaxHeight ? OpenSpan->MaxHeight * CellHeight : MaxHeight;
					FVector MinPoint = BoundMinPoint;
					MinPoint += FVector(X, Y, MinZ);
					FVector MaxPoint = BoundMinPoint + FVector(X + CellSize, Y + CellSize, MaxZ);

					FNNNavMeshDebuggingInfo::HeightFieldDebugBox DebugBox;
					DebugBox.Box = FBox(MinPoint, MaxPoint);
					float DistanceNormalized = UKismetMathLibrary::NormalizeToRange(OpenSpan->EdgeDistance, MinDistance, MaxDistance);
					DebugBox.Color = FLinearColor::LerpUsingHSV(MinColor, MaxColor, DistanceNormalized).ToFColor(false);
					DebuggingInfo.OpenHeightField.Add(MoveTemp(DebugBox));

					OpenSpan = OpenSpan->NextOpenSpan.Get();
				}
			}
		}
	}
}
