#pragma once

// NN Includes
#include "Contour/NNContourGeneration.h"
#include "ConvexPolygon/NNPolyMeshBuilder.h"
#include "NNNavMeshRenderingComp.h"
#include "NavData/Voxelization/HeightFieldGenerator.h"
#include "Pathfinding/NNPathfinding.h"
#include "Voxelization/OpenHeightFieldGenerator.h"

class UNavigationSystemV1;
class FNNNavMeshGenerator;

/** A geometry element representation */
struct FNNRawGeometryElement
{
	// Instance geometry
	TArray<float> GeomCoords;
	TArray<int32> GeomIndices;

	// Per instance transformations in unreal coords
	// When empty geometry is in world space
	TArray<FTransform>	PerInstanceTransform;

	/** Returns the geometry position for the given Index */
	FVector GetGeometryPosition(int32 Index) const;
};

/** Caches a geometry element */
struct FNNGeometryCache
{
	struct FHeader
	{
		FNavigationRelevantData::FCollisionDataHeader Validation;

		int32 NumVerts;
		int32 NumFaces;

		// TODO (ignacio) no idea with I need these two members
		struct FWalkableSlopeOverride SlopeOverride;
		static uint32 StaticMagicNumber;
	};

	FHeader Header;

	/** recast coords of vertices (size: NumVerts * 3) */
	float* Verts;

	/** vert indices for triangles (size: NumFaces * 3) */
	int32* Indices;

	FNNGeometryCache() {}
	FNNGeometryCache(const uint8* Memory);
};

/** The result of the FNNAreaGenerator */
struct FNNAreaGeneratorData
{
	// tile's geometry: without voxel cache
	TArray<FNNRawGeometryElement> RawGeometry;

	FNNHeightField HeightField;

	FNNOpenHeightField OpenHeightField;

	TArray<FNNContour> Contours;

	FNNPolygonMesh PolygonMesh;

	FNNGraph PathfindingGraph;

	/** BoxSpheres used for debugging */
	TArray<FBoxSphereBounds> TemporaryBoxSpheres;

	/** Lines used for debugging */
	TArray<FDebugRenderSceneProxy::FDebugLine>  TemporaryLines;
	TArray<FDebugRenderSceneProxy::FArrowLine>  TemporaryArrows;;

	/** Texts used for debugging */
	TArray<FNNNavMeshSceneProxyData::FDebugText> TemporaryTexts;

	/** Creates a debug point in the world */
	void AddDebugPoint(const FVector& Point, float Radius = 20.0f);
	/** Creates a debug text in the world */
	void AddDebugText(const FVector& Location, const FString& Text);
	/** Creates a debug line in the world */
	void AddDebugLine(const FVector& Start, const FVector& End);
	/** Creates a debug arrow line in the world */
	void AddDebugArrow(const FVector& Start, const FVector& End, const FColor& Color);
};

/** Calculates the nav mesh for a specific FNavigationBounds */
class NACHONAVMESH_API FNNAreaGenerator : public FNonAbandonableTask
{
public:
	FNNAreaGenerator(const FNNNavMeshGenerator* InParentGenerator, const FNavigationBounds& Bounds);

	/** Gathers the geometry inside the AreaBounds */
	void DoWork();

	/** Declaration necessary for FNonAbandonableTask */
	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FNNAreaGeneratorData, STATGROUP_ThreadPoolAsyncTasks);
	}

	friend bool operator==(const FNNAreaGenerator& Lhs, const FNNAreaGenerator& Rhs)
	{
		return Lhs.AreaBounds == Rhs.AreaBounds;
	}

	/** Returns the bounds assigned to this generator */
	const FNavigationBounds& GetAreaBounds() const { return AreaBounds; }

	/** Returns the resulting data */
	FNNAreaGeneratorData* RetrieveGeneratorData() { return AreaGeneratorData.Release(); }

protected:
	/** Gathers the geometry inside the AreaBounds */
	void GatherGeometry( bool bGeometryChanged);
	/** Gather geometry from a specified Navigation Data */
	void GatherNavigationDataGeometry(const TSharedRef<FNavigationRelevantData, ESPMode::ThreadSafe>& ElementData, UNavigationSystemV1& NavSys, const FNavDataConfig& OwnerNavDataConfig, bool bGeometryChanged);
	/** Appends specified geometry to the AreaGeneratorData */
	void AppendGeometry(const FNavigationRelevantData& DataRef, const FCompositeNavModifier& InModifier, const FNavDataPerInstanceTransformDelegate& InTransformsDelegate);

private:
	/** The bounds assigned to the AreaGenerator */
	FNavigationBounds AreaBounds;

	// TODO (ignacio) no idea how safe is to have a raw pointer here
	/** The generator owner */
	const FNNNavMeshGenerator* ParentGenerator;

	/** The resulting data */
	TUniquePtr<FNNAreaGeneratorData> AreaGeneratorData;
};
