// Fill out your copyright notice in the Description page of Project Settings.

#include "NavData/NNNavMeshRenderingComp.h"

// UE Includes
#include "Debug/DebugDrawService.h"
#include "DebugRenderSceneProxy.h"
#include "Engine/Engine.h"

#if WITH_EDITOR
#include "Editor.h"
#endif // WITH_EDITOR

// NN Includes
#include "NavData/NNAreaGenerator.h"
#include "NavData/NNNavMesh.h"


namespace NNNavMeshRenderingCompHelper
{

	constexpr float DrawDistanceSquared = 50000.0f;

	// TODO (ignacio) I think this function is not working
	bool PointInView(const FVector& Position, const FSceneView* View)
	{
		if (FVector::DistSquaredXY(Position, View->ViewMatrices.GetViewOrigin()) > DrawDistanceSquared)
		{
			return false;
		}

		for (int32 PlaneIdx = 0; PlaneIdx < View->ViewFrustum.Planes.Num(); ++PlaneIdx)
		{
			const FPlane& CurPlane = View->ViewFrustum.Planes[PlaneIdx];
			if (CurPlane.PlaneDot(Position) > 0.f)
			{
				return false;
			}
		}

		return true;
	}
}

void FNNNavMeshSceneProxyData::Reset()
{
	MeshBuilders.Reset();
	ThickLineItems.Reset();
	TileEdgeLines.Reset();
	NavMeshEdgeLines.Reset();
	NavLinkLines.Reset();
	ClusterLinkLines.Reset();
	AuxLines.Reset();
	AuxPoints.Reset();
	AuxBoxes.Reset();
	DebugLabels.Reset();
	OctreeBounds.Reset();
	Bounds.Init();

	bNeedsNewData = true;
	bDataGathered = false;
	NavDetailFlags = 0;
}

void FNNNavMeshSceneProxyData::GatherData(const ANNNavMesh* NavMesh)
{
	FNNNavMeshDebuggingInfo DebuggingInfo;
	NavMesh->GrabDebuggingInfo(DebuggingInfo);

	if (NavMesh->bDrawGeometry)
	{
		for (const FNNRawGeometryElement& GeometryToDraw : DebuggingInfo.RawGeometryToDraw)
		{
			const TArray<float>& Coords = GeometryToDraw.GeomCoords;
			const int32 Geometries = GeometryToDraw.GeomCoords.Num() / 3;
			const int32 Indices = GeometryToDraw.GeomIndices.Num() / 3;

			// Gather vertices
			for (int32 i = 0; i < Geometries; ++i)
			{
				FVector Position = GeometryToDraw.GetGeometryPosition(i);
				FDebugPoint Point (Position, FColor::Green, 10.0f);
				AuxPoints.Add(MoveTemp(Point));
			}

			// Gather polygons
			const FColor PolygonColor = FColor::Blue;
			constexpr float PolygonThickness = 2.0f;
			for (int32 i = 0; i < Indices; ++i)
			{
				FVector FirstPoint = GeometryToDraw.GetGeometryPosition(GeometryToDraw.GeomIndices[i * 3]);
				FVector SecondPoint = GeometryToDraw.GetGeometryPosition(GeometryToDraw.GeomIndices[i * 3 + 1]);
				FVector ThirdPoint = GeometryToDraw.GetGeometryPosition(GeometryToDraw.GeomIndices[i * 3 + 2]);
				FDebugRenderSceneProxy::FDebugLine FirstLine (FirstPoint, SecondPoint, PolygonColor, PolygonThickness);
				FDebugRenderSceneProxy::FDebugLine SecondLine (SecondPoint, ThirdPoint, PolygonColor, PolygonThickness);
				FDebugRenderSceneProxy::FDebugLine ThirdLine (ThirdPoint, FirstPoint, PolygonColor, PolygonThickness);
				AuxLines.Add(MoveTemp(FirstLine));
				AuxLines.Add(MoveTemp(SecondLine));
				AuxLines.Add(MoveTemp(ThirdLine));
			}
		}

		// Gather the HeightField spans
		const FColor HeightFieldColor = FColor::Red;
		for (const FBox& HeightField : DebuggingInfo.HeightFields)
		{
			FDebugRenderSceneProxy::FDebugBox Box (HeightField, HeightFieldColor);
			AuxBoxes.Add(MoveTemp(Box));
		}
	}
}

SIZE_T FNNNavMeshSceneProxy::GetTypeHash() const
{
	static size_t UniquePointer;
	return reinterpret_cast<size_t>(&UniquePointer);
}

FNNNavMeshSceneProxy::FNNNavMeshSceneProxy(const UPrimitiveComponent* InComponent, FNNNavMeshSceneProxyData* InProxyData, bool ForceToRender)
	: FDebugRenderSceneProxy(InComponent)
	  , bRequestedData(false)
	  , bForceRendering(ForceToRender)
{
	DrawType = EDrawType::WireMesh;

	if (InProxyData)
	{
		ProxyData = *InProxyData;
		Boxes.Append(InProxyData->AuxBoxes);
	}

	RenderingComponent = MakeWeakObjectPtr(const_cast<UNNNavMeshRenderingComp*>(Cast<UNNNavMeshRenderingComp>(InComponent)));
	bSkipDistanceCheck = GIsEditor && (GEngine->GetDebugLocalPlayer() == nullptr);
	bUseThickLines = GIsEditor;
}

FNNNavMeshSceneProxy::~FNNNavMeshSceneProxy()
{
}

void FNNNavMeshSceneProxy::GetDynamicMeshElements(const TArray<const FSceneView*>& SceneViews,
                                                  const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const
{
	FDebugRenderSceneProxy::GetDynamicMeshElements(SceneViews, ViewFamily, VisibilityMap, Collector);

	for (int32 ViewIndex = 0; ViewIndex < SceneViews.Num(); ++ViewIndex)
	{
		if (VisibilityMap & (1 << ViewIndex))
		{
			const FSceneView* View = SceneViews[ViewIndex];
			const bool bVisible = !!View->Family->EngineShowFlags.Navigation || bForceRendering;
			if (!bVisible)
			{
				continue;
			}
			FPrimitiveDrawInterface* PDI = Collector.GetPDI(ViewIndex);

			// Draw AuxPoints
			for (int32 Index = 0; Index < ProxyData.AuxPoints.Num(); ++Index)
			{
				const auto& Point = ProxyData.AuxPoints[Index];
				// TODO (ignacio) here I should check if the point is in view
				// if (NNNavMeshRenderingCompHelper::PointInView(Point.Position, View))
				{
					PDI->DrawPoint(Point.Position, Point.Color, Point.Size, SDPG_World);
				}
			}

			// Draw AuxLine
			for (int32 Index = 0; Index < ProxyData.AuxLines.Num(); ++Index)
			{
				const auto& Line = ProxyData.AuxLines[Index];
				// TODO (ignacio) here I should check if the point is in view
				// if (NNNavMeshRenderingCompHelper::PointInView(Point.Position, View))
				{
					PDI->DrawLine(Line.Start, Line.End, Line.Color, SDPG_World, Line.Thickness);
				}
			}
		}
	}
}

FPrimitiveViewRelevance FNNNavMeshSceneProxy::GetViewRelevance(const FSceneView* View) const
{
	const bool bVisible = !!View->Family->EngineShowFlags.Navigation || bForceRendering;
	FPrimitiveViewRelevance Result;
	Result.bDrawRelevance = bVisible && IsShown(View);
	Result.bDynamicRelevance = true;
	// ideally the TranslucencyRelevance should be filled out by the material, here we do it conservative
	Result.bSeparateTranslucency = Result.bNormalTranslucency = bVisible && IsShown(View);
	return Result;
}

uint32 FNNNavMeshSceneProxy::GetAllocatedSize() const
{
	return FPrimitiveSceneProxy::GetAllocatedSize() +
		Cylinders.GetAllocatedSize() +
		ArrowLines.GetAllocatedSize() +
		Stars.GetAllocatedSize() +
		DashedLines.GetAllocatedSize() +
		Lines.GetAllocatedSize() +
		Boxes.GetAllocatedSize() +
		Spheres.GetAllocatedSize() +
		Texts.GetAllocatedSize();
}

void FNNNavMeshDebugDrawHelper::InitDelegateHelper(const FNNNavMeshSceneProxy* InSceneProxy)
{
	FDebugDrawDelegateHelper::InitDelegateHelper(InSceneProxy);

	DebugLabels.Reset();
	DebugLabels.Append(InSceneProxy->ProxyData.DebugLabels);
	bForceRendering = InSceneProxy->bForceRendering;
	bNeedsNewData = InSceneProxy->ProxyData.bNeedsNewData;
}

void FNNNavMeshDebugDrawHelper::RegisterDebugDrawDelgate()
{
	ensureMsgf(State != RegisteredState, TEXT("RegisterDebugDrawDelgate is already Registered!"));
	if (State == InitializedState)
	{
		DebugTextDrawingDelegate = FDebugDrawDelegate::CreateRaw(this, &FNNNavMeshDebugDrawHelper::DrawDebugLabels);
		DebugTextDrawingDelegateHandle = UDebugDrawService::Register(TEXT("Navigation"), DebugTextDrawingDelegate);
		State = RegisteredState;
	}
}

void FNNNavMeshDebugDrawHelper::UnregisterDebugDrawDelgate()
{
	ensureMsgf(State != InitializedState, TEXT("UnegisterDebugDrawDelgate is in an invalid State: %i !"), State);
	if (State == RegisteredState)
	{
		check(DebugTextDrawingDelegate.IsBound());
		UDebugDrawService::Unregister(DebugTextDrawingDelegateHandle);
		State = InitializedState;
	}
}

FPrimitiveSceneProxy* UNNNavMeshRenderingComp::CreateSceneProxy()
{
	FNNNavMeshSceneProxy* Proxy = nullptr;
	if (IsVisible())
	{
		ANNNavMesh* NavMesh = Cast<ANNNavMesh>(GetOwner());
		if (NavMesh && NavMesh->IsDrawingEnabled())
		{
			FNNNavMeshSceneProxyData ProxyData;
			ProxyData.GatherData(NavMesh);
			Proxy = new FNNNavMeshSceneProxy(this, &ProxyData);
		}
	}

	if (Proxy)
	{
		NavMeshDebugDrawHelper.InitDelegateHelper(Proxy);
		NavMeshDebugDrawHelper.ReregisterDebugDrawDelgate();
	}
	return Proxy;
}

void UNNNavMeshRenderingComp::OnRegister()
{
	Super::OnRegister();

	const FTimerDelegate Delegate = FTimerDelegate::CreateUObject(this, &UNNNavMeshRenderingComp::TimerFunction);
// #if WITH_EDITOR
// 	if (GEditor)
// 	{
// 		GEditor->GetTimerManager()->SetTimer(TimerHandle, Delegate, 1, true);
// 	}
// #endif // WITH_EDITOR
	GetWorld()->GetTimerManager().SetTimer(TimerHandle, Delegate, 1, true);
}

void UNNNavMeshRenderingComp::OnUnregister()
{
// #if WITH_EDITOR
// 	if (GEditor)
// 	{
// 		GEditor->GetTimerManager()->ClearTimer(TimerHandle);
// 	}
// 	else
// #endif //WITH_EDITOR
	{
		GetWorld()->GetTimerManager().ClearTimer(TimerHandle);
	}
	Super::OnUnregister();
}

void UNNNavMeshRenderingComp::CreateRenderState_Concurrent(FRegisterComponentContext* Context)
{
	Super::CreateRenderState_Concurrent(Context);
	NavMeshDebugDrawHelper.RegisterDebugDrawDelgate();
}

void UNNNavMeshRenderingComp::DestroyRenderState_Concurrent()
{
	NavMeshDebugDrawHelper.UnregisterDebugDrawDelgate();
	Super::DestroyRenderState_Concurrent();
}

FBoxSphereBounds UNNNavMeshRenderingComp::CalcBounds(const FTransform& LocalToWorld) const
{
	FBox BoundingBox(ForceInit);

	if (ANNNavMesh* NavMesh = Cast<ANNNavMesh>(GetOwner()))
	{
		BoundingBox = NavMesh->GetNavMeshBounds();
	}

	return FBoxSphereBounds(BoundingBox);
}

void UNNNavMeshRenderingComp::TimerFunction()
{
	if (bForceUpdate)
	{
		bForceUpdate = false;
		MarkRenderStateDirty();
	}
}

