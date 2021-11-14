#pragma once

// NN Includes
#include "NNNavMeshRenderingComp.h"

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

/** Represents a cell that collides with a polygon */
struct Span
{
	Span() {}
	~Span();

	uint8 MaxSpanHeight = 0;
	uint8 MinSpanHeight = 0;
	Span* NextSpan = nullptr;
	bool bWalkable = false;

	/** Returns a readable representation of this Span */
	FString ToString() const;
};

/** Container of spans */
struct FNNHeightField
{
	FNNHeightField(int32 InUnitsWidth, int32 InUnitsHeight, int32 InUnitsDepth);
	~FNNHeightField();

	/** How spans are contained in every axis */
	int32 UnitsWidth = 0; // X
	int32 UnitsHeight = 0; // Z
	int32 UnitsDepth = 0; // Y

	/** Bounds */
	FVector MinPoint;
	FVector MaxPoint;

	/** The span size */
	float CellSize = 0.0f;
	float CellHeight = 0.0f;

	TArray<Span*> Spans; // 2D array, UnitsWidth * UnitsDepth
};

/** The result of the FNNAreaGenerator */
struct FNNAreaGeneratorData
{
	~FNNAreaGeneratorData();
	// tile's geometry: without voxel cache
	TArray<FNNRawGeometryElement> RawGeometry;

	FNNHeightField* HeightField;

	/** BoxSpheres used for debugging */
	TArray<FBoxSphereBounds> TemporaryBoxSpheres;

	/** Texts used for debugging */
	TArray<FNNNavMeshSceneProxyData::FDebugText> TemporaryTexts;
};

/** Calculates the nav mesh for a specific FNavigationBounds */
class NACHONAVMESH_API FNNAreaGenerator : public FNoncopyable
{
public:
	FNNAreaGenerator(FNNNavMeshGenerator* InParentGenerator, const FNavigationBounds& Bounds);

	/** Gathers the geometry inside the AreaBounds */
	void DoWork();

	/** Returns the result of this FNNAreaGenerator */
	FNNAreaGeneratorData* GetAreaGeneratorData() const { return AreaGeneratorData; }

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
	FNNNavMeshGenerator* ParentGenerator;

	/** The resulting data */
	FNNAreaGeneratorData* AreaGeneratorData = nullptr;
};
