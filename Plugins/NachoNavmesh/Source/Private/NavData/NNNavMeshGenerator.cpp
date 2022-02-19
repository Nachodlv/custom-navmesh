#include "NavData/NNNavMeshGenerator.h"

// UE Includes
#include "Async/AsyncWork.h"
#include "Kismet/KismetMathLibrary.h"

// NN Includes
#include "NavData/Contour/NNContourGeneration.h"
#include "NavData/NNNavMeshRenderingComp.h"
#include "NavData/Voxelization/HeightFieldGenerator.h"
#include "NavData/Voxelization/OpenHeightFieldGenerator.h"

namespace NNNavMeshGeneratorHelpers
{
	FNNNavMeshDebuggingInfo::PolygonDebugInfo BuildDebugInfoFromPolygon(
		const FNNOpenHeightField& OpenHeightField, const FNNPolygonMesh& PolygonMesh, const FNNPolygon& Polygon)
	{
		TArray<FVector> Vertexes;
		TArray<int32> Indexes;
		Vertexes.Reserve(Polygon.Indexes.Num());
		for (int32 i = 0; i < Polygon.Indexes.Num(); ++i)
		{
			const int32 Index = Polygon.Indexes[i];
			if (Index >= 0)
			{
				const FVector& PolygonVector = PolygonMesh.Vertexes[Polygon.Indexes[i]];
				Vertexes.Add(OpenHeightField.TransformVectorToWorldPosition(PolygonVector));
				Indexes.Add(i);
			}
		}
		return FNNNavMeshDebuggingInfo::PolygonDebugInfo(Vertexes, Indexes);
	}
}

FNNNavMeshGenerator::FNNNavMeshGenerator(ANNNavMesh& InNavMesh)
	: NavMesh(&InNavMesh), NavBounds(InNavMesh.GetRegisteredBounds())
{}

FNNNavMeshGenerator::~FNNNavMeshGenerator()
{
	for (const auto& WorkingTask : WorkingTasks)
	{
		if (WorkingTask.Value.bStarted && !WorkingTask.Value.Task->Cancel())
		{
			WorkingTask.Value.Task->EnsureCompletion();
		}
	}
	for (FAsyncTask<FNNAreaGenerator>* CanceledTask : CanceledTasks)
	{
		if (!CanceledTask->Cancel() || !CanceledTask->IsDone())
		{
			CanceledTask->EnsureCompletion();
		}
	}
	WorkingTasks.Reset();
	CanceledTasks.Reset();
}

void FNNNavMeshGenerator::TickAsyncBuild(float DeltaSeconds)
{
	CheckAsyncTasks();
	ProcessDirtyAreas();
}

bool FNNNavMeshGenerator::RebuildAll()
{
	for (const FNavigationBounds& NavBound : NavBounds)
	{
		DirtyAreas.AddUnique(NavBound.UniqueID);
	}
	GeneratorsData.Reset();
	return true;
}

void FNNNavMeshGenerator::RebuildDirtyAreas(const TArray<FNavigationDirtyArea>& NavigationDirtyAreas)
{
	for (const FNavigationDirtyArea& NavigationDirtyArea : NavigationDirtyAreas)
	{
		for (const FNavigationBounds& NavBound : NavBounds)
		{
			if (NavBound.AreaBox.Intersect(NavigationDirtyArea.Bounds))
			{
				DirtyAreas.Add(NavBound.UniqueID);
			}
		}
	}
}

FNNAreaGenerator* FNNNavMeshGenerator::CreateAreaGenerator(const FNavigationBounds& DirtyArea)
{
	return new FNNAreaGenerator(this, DirtyArea);
}

void FNNNavMeshGenerator::CheckAsyncTasks()
{
	const float TimeSeconds = NavMesh->GetWorld()->GetTimeSeconds();

	TArray<uint32> BoundsID;
	WorkingTasks.GetKeys(BoundsID);
	bool bRefreshRenderer = false;
	for (int32 i = BoundsID.Num() - 1; i >= 0; --i)
	{
		const uint32 BoundID = BoundsID[i];
		FNNWorkingAsyncTask& WorkingTask = WorkingTasks[BoundID];
		if (!WorkingTask.bStarted)
		{
			if (WorkingTask.StartTime < TimeSeconds)
			{
				WorkingTask.Task->StartBackgroundTask();
				WorkingTask.bStarted = true;
			}
		}
		else if (WorkingTask.Task->IsDone())
		{
			bRefreshRenderer = true;
			FNNAreaGenerator& AreaGenerator = WorkingTask.Task->GetTask();
			GeneratorsData.Add(BoundID, AreaGenerator.RetrieveGeneratorData());
			WorkingTasks.Remove(BoundID);
		}
	}

	if (bRefreshRenderer)
	{
		Cast<UNNNavMeshRenderingComp>(NavMesh->RenderingComp)->ForceUpdate();
	}

	for (int32 i = CanceledTasks.Num() - 1; i >= 0; --i)
	{
		FAsyncTask<FNNAreaGenerator>* TaskCanceled = CanceledTasks[i];
		if (TaskCanceled->Cancel() || TaskCanceled->IsDone())
		{
			CanceledTasks.RemoveAt(i);
		}
	}
}

void FNNNavMeshGenerator::ProcessDirtyAreas()
{
	if (DirtyAreas.Num() == 0)
	{
		return;
	}

	const float TimeSeconds = NavMesh->GetWorld()->GetTimeSeconds();
	const float StartWorkingTasks = TimeSeconds + WaitTimeToStartWorkingTask;
	for (int32 i = DirtyAreas.Num() - 1; i >= 0; --i)
	{
		const uint32 BoundsID = DirtyAreas[i];
		FNavigationBounds DirtyAreaSearch;
		DirtyAreaSearch.UniqueID = BoundsID;
		const FNavigationBounds* DirtyArea = NavBounds.Find(DirtyAreaSearch);

		// Deletes the data of this area previously calculated
		if (FNNAreaGeneratorData** GeneratorData = GeneratorsData.Find(BoundsID))
		{
			delete (*GeneratorData);
			GeneratorsData.Remove(BoundsID);
		}

		// The area might have been deleted
		if (DirtyArea)
		{
			// Cancels any task that is currently calculating the same area
			if (const FNNWorkingAsyncTask* WorkingAsyncTask = WorkingTasks.Find(BoundsID))
			{
				if (WorkingAsyncTask->bStarted)
				{
					CanceledTasks.Add(WorkingAsyncTask->Task);
				}
			}

			// Starts calculating the navmesh for this area async
			FAsyncTask<FNNAreaGenerator>* Task = new FAsyncTask<FNNAreaGenerator>(this, *DirtyArea);
			FNNWorkingAsyncTask WorkingTask = FNNWorkingAsyncTask(Task, StartWorkingTasks);
			WorkingTasks.Emplace(BoundsID, MoveTemp(WorkingTask));
		}
	}
	DirtyAreas.Reset();
}

int32 FNNNavMeshGenerator::GetNumRunningBuildTasks() const
{
	int32 RunningTasks = 0;
	for (const auto& WorkingTask : WorkingTasks)
	{
		if (WorkingTask.Value.bStarted)
		{
			++RunningTasks;
		}
	}
	return RunningTasks;
}

bool FNNNavMeshGenerator::IsBuildInProgressCheckDirty() const
{
	return DirtyAreas.Num() > 0 || GetNumRunningBuildTasks() > 0;
}

void FNNNavMeshGenerator::CancelBuild()
{
	for (auto& WorkingTask : WorkingTasks)
	{
		if (WorkingTask.Value.bStarted)
		{
			CanceledTasks.Add(WorkingTask.Value.Task);
		}
	}
	WorkingTasks.Empty();
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
		DebuggingInfo.TemporaryArrows.Append(Result.Value->TemporaryArrows);

		const TArray<TUniquePtr<Span>>& Spans = Result.Value->HeightField.Spans;

		// TODO (ignacio) this can be moved to a function
		FNavigationBounds DataSearch;
		DataSearch.UniqueID = Result.Key;
		const FNavigationBounds* GeneratorArea = NavBounds.Find(DataSearch);
		const FVector BoundMinPoint = GeneratorArea->AreaBox.Min;

		// Converts the HeightField Spans into FBoxes
		const float CellSize = Result.Value->HeightField.CellSize;
		const float CellHeight = Result.Value->HeightField.CellHeight;
		const int32 UnitsWidth = Result.Value->HeightField.UnitsWidth;
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
		const FNNOpenHeightField& OpenHeightField = Result.Value->OpenHeightField;
		if (OpenHeightField.Spans.Num() > 0)
		{
			const TArray<TUniquePtr<FNNOpenSpan>>& OpenSpans = OpenHeightField.Spans;

			int32 MaxDistance = INDEX_NONE;
			int32 MinDistance = 1; // There is always a span with distance 0
			for (const TUniquePtr<FNNOpenSpan>& OpenSpan : OpenSpans)
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
			float MaxHeight = Result.Value->OpenHeightField.Bounds.Max.Z;
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

		// Grab the Regions debugging info
		const TArray<FNNRegion>& Regions = OpenHeightField.Regions;
		DebuggingInfo.Regions.Reserve(Regions.Num());
		for (const FNNRegion& Region : Regions)
		{
			TArray<FBox> RegionSpans;
			RegionSpans.Reserve(Region.Spans.Num());
			for (const FNNOpenSpan* OpenSpan : Region.Spans)
			{
				const float X = OpenSpan->X * OpenHeightField.CellSize;
				const float Y = OpenSpan->Y * OpenHeightField.CellSize;
				const float Z = OpenSpan->MinHeight * OpenHeightField.CellHeight;
				FVector MinPoint = BoundMinPoint + FVector(X, Y, Z);
				FVector MaxPoint = MinPoint + FVector(OpenHeightField.CellSize, OpenHeightField.CellSize, 0.0f);
				RegionSpans.Emplace(FBox(MinPoint, MaxPoint));
			}
			DebuggingInfo.Regions.Emplace(FNNNavMeshDebuggingInfo::RegionDebugInfo(RegionSpans));
		}

		// Grab the contour debugging info
		const TArray<FNNContour>& Contours = Result.Value->Contours;
		DebuggingInfo.Contours.Reserve(Contours.Num());
		for (const FNNContour& Contour : Contours)
		{
			TArray<FVector> DebugSimplifiedVertexes;
			DebugSimplifiedVertexes.Reserve(Contour.SimplifiedVertexes.Num());
			for (const FVector& Vertex : Contour.SimplifiedVertexes)
			{
				FVector WorldVertex = OpenHeightField.TransformVectorToWorldPosition(Vertex);
				DebugSimplifiedVertexes.Add(MoveTemp(WorldVertex));
			}
			TArray<FVector> DebugRawVertexes;
			DebugRawVertexes.Reserve(Contour.RawVertexes.Num());
			for (const FVector& RawVertex : Contour.RawVertexes)
			{
				FVector WorldVertex = OpenHeightField.TransformVectorToWorldPosition(RawVertex);
				DebugRawVertexes.Add(MoveTemp(WorldVertex));
			}

			FNNNavMeshDebuggingInfo::ContourDebugInfo DebugInfo(MoveTemp(DebugRawVertexes), MoveTemp(DebugSimplifiedVertexes));
			DebuggingInfo.Contours.Add(MoveTemp(DebugInfo));
		}

		const FNNPolygonMesh& PolygonMesh = Result.Value->PolygonMesh;
		DebuggingInfo.MeshTriangulated.Reserve(PolygonMesh.PolygonIndexes.Num());
		for (const FNNPolygon& Triangle : PolygonMesh.TriangleIndexes)
		{
			FNNNavMeshDebuggingInfo::PolygonDebugInfo DebugInfo = NNNavMeshGeneratorHelpers::BuildDebugInfoFromPolygon(OpenHeightField, PolygonMesh, Triangle);
			DebuggingInfo.MeshTriangulated.Add(MoveTemp(DebugInfo));
		}
		for (const FNNPolygon& Polygon : PolygonMesh.PolygonIndexes)
		{
			FNNNavMeshDebuggingInfo::PolygonDebugInfo DebugInfo = NNNavMeshGeneratorHelpers::BuildDebugInfoFromPolygon(OpenHeightField, PolygonMesh, Polygon);
			DebuggingInfo.PolygonMesh.Add(MoveTemp(DebugInfo));
		}
	}
}
