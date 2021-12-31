// Fill out your copyright notice in the Description page of Project Settings.

#include "NavData/NNNavMeshRenderingComp.h"

// UE Includes
#include "DebugRenderSceneProxy.h"
#include "Debug/DebugDrawService.h"
#include "Engine/Canvas.h"
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

	FColor GetRandomColor()
	{
		const int32 R = FMath::RandRange(0, 255);
		const int32 G = FMath::RandRange(0, 255);
		const int32 B = FMath::RandRange(0, 255);
		return  FColor(R, G, B, 255);
	}

	void DrawArrayOfPoints(const TArray<FVector>& InPoints, const FColor& DebugColor,
		TArray<FNNNavMeshSceneProxyData::FDebugPoint>& OutDebugPoints, TArray<FDebugRenderSceneProxy::FDebugLine>& OutDebugLines)
	{
		const uint32 VerticesNum = InPoints.Num();
		for (uint32 i = 0; i < VerticesNum; ++i)
		{
			const FVector& Start = InPoints[i];
			OutDebugPoints.Emplace(Start, DebugColor, 10.0f);
			OutDebugLines.Emplace(Start, InPoints[(i + 1) % VerticesNum], DebugColor, 2.0f);
		}
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
		bNeedsNewData = false;

		if (NavMesh->bDrawPolygons)
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
		}

		// Gather the HeightField spans
		if (NavMesh->bDrawHeightField)
		{
			for (const FNNNavMeshDebuggingInfo::HeightFieldDebugBox& HeightField : DebuggingInfo.HeightField)
			{
				FDebugRenderSceneProxy::FDebugBox Box (HeightField.Box, HeightField.Color);
				AuxBoxes.Add(MoveTemp(Box));
			}
		}

		// Gather the OpenHeightField spans
		if (NavMesh->bDrawOpenHeightField)
		{
			for (const FNNNavMeshDebuggingInfo::HeightFieldDebugBox& OpenSpan : DebuggingInfo.OpenHeightField)
			{
				FDebugRenderSceneProxy::FDebugBox Box = FDebugRenderSceneProxy::FDebugBox(OpenSpan.Box, OpenSpan.Color);
				if (NavMesh->bDrawOpenHeightFieldFloor)
				{
					const FVector MinPoint = OpenSpan.Box.Min;
					FVector MaxPoint = OpenSpan.Box.Max;
					MaxPoint.Z = MinPoint.Z + 1.0f;
					FBox CeilBox = FBox(MinPoint, MaxPoint);
					Box = FDebugRenderSceneProxy::FDebugBox(CeilBox, OpenSpan.Color);
				}
				AuxBoxes.Add(MoveTemp(Box));
			}
		}

		if (NavMesh->bDrawRegions)
		{
			for (const FNNNavMeshDebuggingInfo::RegionDebugInfo& Region : DebuggingInfo.Regions)
			{
				FColor RandomColor = NNNavMeshRenderingCompHelper::GetRandomColor();
				for (const FBox& Span : Region.Spans)
				{
					FVector FirstVertex = Span.Min;
					FVector SecondVertex = FirstVertex;
					SecondVertex.Y = Span.Max.Y;
					FVector ThirdVertex = Span.Max;
					FVector FourthVertex = ThirdVertex;
					FourthVertex.Y = Span.Min.Y;
					FDebugMeshData MeshData;
					MeshData.Vertices.Append({FirstVertex, SecondVertex, ThirdVertex, FourthVertex});
					MeshData.Indices.Append({0, 1, 2, 2, 3, 0});
					MeshData.ClusterColor = RandomColor;
					MeshBuilders.Add(MoveTemp(MeshData));
				}
			}
		}

		// Draw the region contours
		for (const auto& Contour : DebuggingInfo.Contours)
		{
			FColor RandomColor = NNNavMeshRenderingCompHelper::GetRandomColor();
			if (NavMesh->bDrawContours)
			{
				NNNavMeshRenderingCompHelper::DrawArrayOfPoints(Contour.SimplifiedVertexes, RandomColor, AuxPoints, AuxLines);
			}
			if (NavMesh->bDrawRawVertexesContour)
			{
				NNNavMeshRenderingCompHelper::DrawArrayOfPoints(Contour.RawVertexes, RandomColor, AuxPoints, AuxLines);
			}
		}

		// Draw the triangulated mesh
		if (NavMesh->bDrawPolygonMesh || NavMesh->bDrawPolyMeshTriangles)
		{
			for (const FNNNavMeshDebuggingInfo::PolygonDebugInfo& MeshTriangulated : DebuggingInfo.MeshTriangulated)
			{
				FDebugMeshData MeshData;
				MeshData.Vertices.Append(MeshTriangulated.Vertexes);
				MeshData.Indices.Append(MeshTriangulated.Indexes);
				MeshData.ClusterColor = FColor::Green;
				MeshBuilders.Add(MoveTemp(MeshData));
				if (NavMesh->bDrawPolyMeshTriangles)
				{
					NNNavMeshRenderingCompHelper::DrawArrayOfPoints(MeshTriangulated.Vertexes, FColor::Blue, AuxPoints, AuxLines);
				}
			}
		}
		if (NavMesh->bDrawPolygonMesh)
		{
			for (const FNNNavMeshDebuggingInfo::PolygonDebugInfo& PolygonMesh : DebuggingInfo.PolygonMesh)
			{
				NNNavMeshRenderingCompHelper::DrawArrayOfPoints(PolygonMesh.Vertexes, FColor::Blue, AuxPoints, AuxLines);
			}
		}

		// Gather the BoxSpheresTemporary
		for (const FBoxSphereBounds& BoxSphere : DebuggingInfo.TemporaryBoxSpheres)
		{
			AuxPoints.Emplace(BoxSphere.Origin, FColor::Orange, BoxSphere.SphereRadius);
		}

		for (const FNNNavMeshSceneProxyData::FDebugText& Text : DebuggingInfo.TemporaryTexts)
		{
			DebugLabels.Add(Text);
		}

		for (const FDebugRenderSceneProxy::FDebugLine& Line : DebuggingInfo.TemporaryLines)
		{
			AuxLines.Add(Line);
		}

		for (const FDebugRenderSceneProxy::FArrowLine& Arrow : DebuggingInfo.TemporaryArrows)
		{
			AuxArrows.Add(Arrow);
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
	  , VertexFactory(GetScene().GetFeatureLevel(), "FNNNavMeshSceneProxy")
	  , bRequestedData(false)
	  , bForceRendering(ForceToRender)
{
	DrawType = EDrawType::SolidMesh;

	if (InProxyData)
	{
		ProxyData = *InProxyData;
		Boxes.Append(InProxyData->AuxBoxes);
	}

	const int32 NumberOfMeshes = ProxyData.MeshBuilders.Num();
	if (!NumberOfMeshes)
	{
		return;
	}

	MeshColors.Reserve(NumberOfMeshes);
	MeshBatchElements.Reserve(NumberOfMeshes);
	const FMaterialRenderProxy* ParentMaterial = GEngine->DebugMeshMaterial->GetRenderProxy();

	TArray<FDynamicMeshVertex> Vertices;
	for (int32 Index = 0; Index < NumberOfMeshes; ++Index)
	{
		const auto& CurrentMeshBuilder = ProxyData.MeshBuilders[Index];

		FMeshBatchElement Element;
		Element.FirstIndex = IndexBuffer.Indices.Num();
		Element.NumPrimitives = FMath::FloorToInt(CurrentMeshBuilder.Indices.Num() / 3);
		Element.MinVertexIndex = Vertices.Num();
		Element.MaxVertexIndex = Element.MinVertexIndex + CurrentMeshBuilder.Vertices.Num() - 1;
		Element.IndexBuffer = &IndexBuffer;
		MeshBatchElements.Add(Element);

		MeshColors.Add(FColoredMaterialRenderProxy(ParentMaterial, CurrentMeshBuilder.ClusterColor));

		const int32 VertexIndexOffset = Vertices.Num();
		Vertices.Append(CurrentMeshBuilder.Vertices);
		if (VertexIndexOffset == 0)
		{
			IndexBuffer.Indices.Append(CurrentMeshBuilder.Indices);
		}
		else
		{
			IndexBuffer.Indices.Reserve(IndexBuffer.Indices.Num() + CurrentMeshBuilder.Indices.Num());
			for (const auto VertIndex : CurrentMeshBuilder.Indices)
			{
				IndexBuffer.Indices.Add(VertIndex + VertexIndexOffset);
			}
		}
	}

	MeshColors.Add(FColoredMaterialRenderProxy(ParentMaterial, FColor::Green));

	if (Vertices.Num())
	{
		VertexBuffers.InitFromDynamicVertex(&VertexFactory, Vertices);
	}
	if (IndexBuffer.Indices.Num())
	{
		BeginInitResource(&IndexBuffer);
	}

	RenderingComponent = MakeWeakObjectPtr(const_cast<UNNNavMeshRenderingComp*>(Cast<UNNNavMeshRenderingComp>(InComponent)));
	bSkipDistanceCheck = GIsEditor && (GEngine->GetDebugLocalPlayer() == nullptr);
	bUseThickLines = GIsEditor;
}

FNNNavMeshSceneProxy::~FNNNavMeshSceneProxy()
{
	VertexBuffers.PositionVertexBuffer.ReleaseResource();
	VertexBuffers.StaticMeshVertexBuffer.ReleaseResource();
	VertexBuffers.ColorVertexBuffer.ReleaseResource();
	IndexBuffer.ReleaseResource();
	VertexFactory.ReleaseResource();
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

			// Draw Mesh
			if (MeshBatchElements.Num())
			{
				for (int32 Index = 0; Index < MeshBatchElements.Num(); ++Index)
				{
					if (MeshBatchElements[Index].NumPrimitives == 0)
					{
						continue;
					}

					FMeshBatch& Mesh = Collector.AllocateMesh();
					FMeshBatchElement& BatchElement = Mesh.Elements[0];
					BatchElement = MeshBatchElements[Index];

					FDynamicPrimitiveUniformBuffer& DynamicPrimitiveUniformBuffer = Collector.AllocateOneFrameResource<FDynamicPrimitiveUniformBuffer>();
					DynamicPrimitiveUniformBuffer.Set(FMatrix::Identity, FMatrix::Identity, GetBounds(), GetLocalBounds(), false, false, DrawsVelocity(), false);
					BatchElement.PrimitiveUniformBufferResource = &DynamicPrimitiveUniformBuffer.UniformBuffer;

					Mesh.bWireframe = false;
					Mesh.VertexFactory = &VertexFactory;
					Mesh.MaterialRenderProxy = &MeshColors[Index];
					Mesh.ReverseCulling = IsLocalToWorldDeterminantNegative();
					Mesh.Type = PT_TriangleList;
					Mesh.DepthPriorityGroup = SDPG_World;
					Mesh.bCanApplyViewModeOverrides = false;
					Collector.AddMesh(ViewIndex, Mesh);
				}
			}

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

			// Draw AuxArrows
			for (int32 Index = 0; Index < ProxyData.AuxArrows.Num(); ++Index)
			{
				const auto& Arrow = ProxyData.AuxArrows[Index];
				NNDrawLineArrow(PDI, Arrow.Start, Arrow.End, Arrow.Color, 10.0f);
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

void FNNNavMeshSceneProxy::NNDrawLineArrow(FPrimitiveDrawInterface* PDI, const FVector& Start, const FVector& End,
	const FColor& Color, float Mag) const
{
	// draw a pretty arrow
	FVector Dir = End - Start;
	const float DirMag = Dir.Size();
	Dir /= DirMag;
	FVector YAxis, ZAxis;
	Dir.FindBestAxisVectors(YAxis,ZAxis);
	FMatrix ArrowTM(Dir,YAxis,ZAxis,Start);
	DrawDirectionalArrow(PDI,ArrowTM,Color,DirMag,Mag,SDPG_World);
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

void FNNNavMeshDebugDrawHelper::DrawDebugLabels(UCanvas* Canvas, APlayerController* PlayerController)
{
	if (!Canvas)
	{
		return;
	}

	const bool bVisible = (Canvas->SceneView && !!Canvas->SceneView->Family->EngineShowFlags.Navigation) || bForceRendering;
	if (!bVisible || bNeedsNewData || DebugLabels.Num() == 0)
	{
		return;
	}

	const FColor OldDrawColor = Canvas->DrawColor;
	Canvas->SetDrawColor(FColor::White);
	const FSceneView* View = Canvas->SceneView;
	const UFont* Font = GEngine->GetSmallFont();
	const FNNNavMeshSceneProxyData::FDebugText* DebugText = DebugLabels.GetData();
	for (int32 Idx = 0; Idx < DebugLabels.Num(); ++Idx, ++DebugText)
	{
		if (View->ViewFrustum.IntersectSphere(DebugText->Location, 1.0f))
		{
			const FVector ScreenLoc = Canvas->Project(DebugText->Location);
			Canvas->DrawText(Font, DebugText->Text, ScreenLoc.X, ScreenLoc.Y);
		}
	}

	Canvas->SetDrawColor(OldDrawColor);
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

