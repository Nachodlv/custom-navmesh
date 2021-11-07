﻿#pragma once

class UNavigationSystemV1;
class FNNNavMeshGenerator;

/** A geometry element representation */
struct FNNRawGeometryElement
{
	// Instance geometry
	TArray<float>		GeomCoords;
	TArray<int32>		GeomIndices;

	// Per instance transformations in unreal coords
	// When empty geometry is in world space
	TArray<FTransform>	PerInstanceTransform;
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
};

/** Calculates the nav mesh for a specific FNavigationBounds */
class NACHONAVMESH_API FNNAreaGenerator : public FNoncopyable
{
public:
	FNNAreaGenerator(FNNNavMeshGenerator* InParentGenerator, const FNavigationBounds& Bounds);

	/** Gathers the geometry inside the AreaBounds */
	void DoWork();

	/** Returns the result of this FNNAreaGenerator */
	const FNNAreaGeneratorData& GetAreaGeneratorData() const { return AreaGeneratorData; }

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
	FNNAreaGeneratorData AreaGeneratorData;
};