// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

// UE Includes
#include "CoreMinimal.h"
#include "DebugRenderSceneProxy.h"

#include "NNNavMeshRenderingComp.generated.h"

struct FNNRawGeometryElement;
struct FNNRegion;

class ANNNavMesh;
class UNNNavMeshRenderingComp;

/** Contains all the information that needs to be drawn on screen */
struct NACHONAVMESH_API FNNNavMeshSceneProxyData : public TSharedFromThis<FNNNavMeshSceneProxyData, ESPMode::ThreadSafe>
{
	struct FDebugMeshData
	{
		TArray<FDynamicMeshVertex> Vertices;
		TArray<uint32> Indices;
		FColor ClusterColor;
	};
	TArray<FDebugMeshData> MeshBuilders;

	struct FDebugPoint
	{
		FDebugPoint() {}
		FDebugPoint(const FVector& InPosition, const FColor& InColor, const float InSize) : Position(InPosition), Color(InColor), Size(InSize) {}
		FVector Position;
		FColor Color;
		float Size;
	};

	TArray<FDebugRenderSceneProxy::FDebugLine> ThickLineItems;
	TArray<FDebugRenderSceneProxy::FDebugLine> TileEdgeLines;
	TArray<FDebugRenderSceneProxy::FDebugLine> NavMeshEdgeLines;
	TArray<FDebugRenderSceneProxy::FDebugLine> NavLinkLines;
	TArray<FDebugRenderSceneProxy::FDebugLine> ClusterLinkLines;
	TArray<FDebugRenderSceneProxy::FDebugLine> AuxLines;
	TArray<FDebugRenderSceneProxy::FDebugBox> AuxBoxes;
	TArray<FDebugPoint> AuxPoints;
	TArray<FDebugRenderSceneProxy::FMesh> Meshes;

	struct FDebugText
	{
		FVector Location;
		FString Text;

		FDebugText() {}
		FDebugText(const FVector& InLocation, const FString& InText) : Location(InLocation), Text(InText) {}
	};
	TArray<FDebugText> DebugLabels;

	TArray<FBoxCenterAndExtent>	OctreeBounds;

	FBox Bounds;
	FVector NavMeshDrawOffset;
	uint32 bDataGathered : 1;
	uint32 bNeedsNewData : 1;
	int32 NavDetailFlags;

	FNNNavMeshSceneProxyData() : NavMeshDrawOffset(0, 0, 10.f),
		bDataGathered(false), bNeedsNewData(true), NavDetailFlags(0) {}

	/** Resets all the information to draw */
	void Reset();

	/** Gathers all the information to draw from the NavMesh */
	void GatherData(const ANNNavMesh* NavMesh);
};

/** Draws the information provided by the FNNNavMeshSceneProxyData */
class  FNNNavMeshSceneProxy final : public FDebugRenderSceneProxy
{
	friend class FNNNavMeshDebugDrawHelper;

public:
	virtual SIZE_T GetTypeHash() const override;

	FNNNavMeshSceneProxy(const UPrimitiveComponent* InComponent, FNNNavMeshSceneProxyData* InProxyData, bool ForceToRender = false);
	virtual ~FNNNavMeshSceneProxy();

	/** Draws the debugging information in the screen */
	virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const override;

protected:
	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override;
	virtual uint32 GetMemoryFootprint() const override { return sizeof(*this) + FNNNavMeshSceneProxy::GetAllocatedSize(); }
	uint32 GetAllocatedSize() const;

	TArray<FColoredMaterialRenderProxy> MeshColors;
	TArray<FMeshBatchElement> MeshBatchElements;
	FDynamicMeshIndexBuffer32 IndexBuffer;
	FStaticMeshVertexBuffers VertexBuffers;
	FLocalVertexFactory VertexFactory;

private:
	/** Contains all the information to draw */
	FNNNavMeshSceneProxyData ProxyData;

	TWeakObjectPtr<UNNNavMeshRenderingComp> RenderingComponent;
	uint32 bRequestedData : 1;
	uint32 bForceRendering : 1;
	uint32 bSkipDistanceCheck : 1;
	uint32 bUseThickLines : 1;
};

/** Contains the information used to calculate the debugging info */
struct FNNNavMeshDebuggingInfo
{

	struct HeightFieldDebugBox
	{
		FBox Box;
		FColor Color;
	};
	struct RegionDebugInfo
	{
		RegionDebugInfo(const TArray<FBox>& InSpans) : Spans(InSpans) {}
		TArray<FBox> Spans;
	};

	/** The geometry vertices */
	TArray<FNNRawGeometryElement> RawGeometryToDraw;
	/** The HeightField spans represented as boxes */
	TArray<HeightFieldDebugBox> HeightField;
	/** The OpenHeightField spans represented as boxes */
	TArray<HeightFieldDebugBox> OpenHeightField;
	/** Box spheres for debugging. No specific usage */
	TArray<FBoxSphereBounds> TemporaryBoxSpheres;
	/** Texts for debugging. No specific usage */
	TArray<FNNNavMeshSceneProxyData::FDebugText> TemporaryTexts;
	/** Lines for debugging. No specific usage */
	TArray<FDebugRenderSceneProxy::FDebugLine> TemporaryLines;
	TArray<RegionDebugInfo> Regions;
};

class FNNNavMeshDebugDrawHelper : public FDebugDrawDelegateHelper
{
public:
	FNNNavMeshDebugDrawHelper()
		: bForceRendering(false)
		  , bNeedsNewData(false) {}

	virtual void InitDelegateHelper(const FDebugRenderSceneProxy* InSceneProxy) override { check(false); }
	void InitDelegateHelper(const FNNNavMeshSceneProxy* InSceneProxy);

	virtual void RegisterDebugDrawDelgate() override;
	virtual void UnregisterDebugDrawDelgate() override;

	/** Draws the texts from the DebugLabels array */
	virtual void DrawDebugLabels(UCanvas* Canvas, APlayerController*) override;
private:
	TArray<FNNNavMeshSceneProxyData::FDebugText> DebugLabels;
	uint32 bForceRendering : 1;
	uint32 bNeedsNewData : 1;
};

UCLASS()
class NACHONAVMESH_API UNNNavMeshRenderingComp : public UPrimitiveComponent
{
	GENERATED_BODY()

public:
	//~ Begin UPrimitiveComponent Interface
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual void OnRegister()  override;
	virtual void OnUnregister()  override;
	//~ End UPrimitiveComponent Interface

	// ~ Being UActorComponent
	virtual void CreateRenderState_Concurrent(FRegisterComponentContext* Context) override;
	virtual void DestroyRenderState_Concurrent() override;
	// ~ End UActorComponent

	// ~ Begin USceneComponent
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
	// ~ End USceneComponent

	void ForceUpdate() { bForceUpdate = true; }
	bool IsForcingUpdate() const { return bForceUpdate; }

protected:
	void TimerFunction();

	FTimerHandle TimerHandle;

	FNNNavMeshDebugDrawHelper NavMeshDebugDrawHelper;

private:
	bool bForceUpdate = false;
};


