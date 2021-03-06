#pragma once

// UE Includes
#include "CoreMinimal.h"
#include "NavigationData.h"

#include "NNNavMesh.generated.h"

struct FNNNavMeshDebuggingInfo;
struct FNNPolygon;

class FNNNavMeshGenerator;

UCLASS()
class NACHONAVMESH_API ANNNavMesh : public ANavigationData
{
	GENERATED_BODY()

friend FNNNavMeshGenerator;

public:
	ANNNavMesh();

	/** Searches for a path for the given query */
	static FPathFindingResult FindPath(const FNavAgentProperties& AgentProperties, const FPathFindingQuery& Query);

	/** Searches for the nearest point in the navmesh inside the given Extent */
	virtual bool ProjectPoint(const FVector& Point, FNavLocation& OutLocation, const FVector& Extent, FSharedConstNavQueryFilter Filter, const UObject* Querier) const override;

	/** Retrieves the polygon containing the NavLocation. Returns whether it was found */
	bool GetPolygonFromNavLocation(const FNavLocation& NavLocation, FNNPolygon& OutPolygon) const;

	/**
	 * Constructs and sets the generator which generates the navmesh. This function is called at the start of the game
	 * or editor or whenever the generator needs to be refreshed. It is conditional because sometimes there is no
	 * need to spawn a generator, for example when runtime navigation generation is not supported it should not
	 * spawn a generator when not in the editor. The generator should be an object derived from FNavDataGenerator
	 * and is set to a variable called NavDataGenerator in ANavigationData.
	*/
	virtual void ConditionalConstructGenerator() override;

	/**
	 * Constructs a component that contains the debug rendering of the navmesh. This is called every time the
	 * rendering flag of the navigation system is set to dirty
	 */
	virtual UPrimitiveComponent* ConstructRenderingComponent() override;

	/** Returns the navigation bounds */
	const TSet<FNavigationBounds>& GetRegisteredBounds() const;

	/** Fills the debugging info with the nav mesh results */
	void GrabDebuggingInfo(FNNNavMeshDebuggingInfo& DebuggingInfo) const;

	/** Transform the registered bounds to a FBox */
	FBox GetNavMeshBounds() const;

	/** Whether it should draw the geometry indices */
	UPROPERTY(EditAnywhere, Category = "NN|Debug")
	bool bDrawGeometry = false;

	/** Whether it should draw points in the element vertex and lines between them */
	UPROPERTY(EditAnywhere, Category = "NN|Debug|Voxelization")
	bool bDrawPolygons = false;

	/** Whether it should draw the spans from the HeightField */
	UPROPERTY(EditAnywhere, Category = "NN|Debug|Voxelization")
	bool bDrawHeightField = false;

	/** Whether it should draw the spans from the OpenHeightField */
	UPROPERTY(EditAnywhere, Category = "NN|Debug|Voxelization")
	bool bDrawOpenHeightField = false;

	/** Whether it should draw the regions created with the open spans */
	UPROPERTY(EditAnywhere, Category = "NN|Debug|Region")
	bool bDrawRegions = false;

	/** Whether it should draw the contours around the regions */
	UPROPERTY(EditAnywhere, Category = "NN|Debug|Contour")
	bool bDrawContours = false;

	/** Whether it should draw the raw vertexes from the regions */
	UPROPERTY(EditAnywhere, Category = "NN|Debug|Contour")
	bool bDrawRawVertexesContour = false;

	/** Whether it should only draw the floor of the OpenHeightField's spans */
	UPROPERTY(EditAnywhere, Category = "NN|Debug|Voxelization", meta = (EditCondition = "bDrawOpenHeightField"))
	bool bDrawOpenHeightFieldFloor = false;

	/** Whether it should draw the polygon mesh */
	UPROPERTY(EditAnywhere, Category = "NN|Debug|PolyMesh")
	bool bDrawPolygonMesh = false;

	/** Whether it should draw the mesh without merging the triangles */
	UPROPERTY(EditAnywhere, Category = "NN|Debug|PolyMesh")
	bool bDrawPolyMeshTriangles = false;

	/** Size of the tallest agent that will path with this navmesh. */
	UPROPERTY(EditAnywhere, Category = "NN|Config")
	float AgentHeight;

	/** Radius of the smallest agent that will path with this navmesh */
	UPROPERTY(EditAnywhere, Category = "NN|Config")
	float AgentRadius = 30.0f;

	/** The X and Y size from the Spans in the HeightField */
	UPROPERTY(EditAnywhere, Category = "NN|Config")
	float CellSize = 20.0f;

	/** The Z size from the Spans in the HeightField */
	UPROPERTY(EditAnywhere, Category = "NN|Config")
	float CellHeight = 20.0f;

	/** The maximum possible angle the agent can climb */
	UPROPERTY(EditAnywhere, Category = "NN|Config")
	float WalkableSlopeDegrees = 45.0f;

	/** The maximum ledge an agent can walk */
	UPROPERTY(EditAnywhere, Category = "NN|Config")
	float MaxLedgeHeight = 50.0f;

	UPROPERTY(EditAnywhere, Category = "NN|Config")
	float MinRegionSize = 50.0f;

	/** The maximum distance the edge may deviate from the geometry */
	UPROPERTY(EditAnywhere, Category = "NN|Config|Contour")
	float ContourDeviationThreshold = 0.5f;

	/** The maximum length of polygon edges that represent the border of the navmesh */
	UPROPERTY(EditAnywhere, Category = "NN|Config|Contour")
	float MaxEdgeLength = 100.0f;

protected:
	virtual void PostEditChangeChainProperty(FPropertyChangedChainEvent& PropertyChangedEvent) override;
};
