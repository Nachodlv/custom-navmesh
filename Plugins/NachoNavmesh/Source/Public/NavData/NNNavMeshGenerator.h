#pragma once

// NN Includes
#include "NNAreaGenerator.h"
#include "NNNavMesh.h"

#include "AI/NavDataGenerator.h"

class NACHONAVMESH_API FNNNavMeshGenerator : public FNavDataGenerator
{
	friend ANNNavMesh;

public:
	FNNNavMeshGenerator(ANNNavMesh& InNavMesh);

	// ~ Begin FNavDataGenerator

	/** Processes the dirty areas */
	virtual void TickAsyncBuild(float DeltaSeconds) override;

	/** Marks all the nav bounds dirty */
	virtual bool RebuildAll() override;

	/** Marks the DirtyAreas dirty */
	virtual void RebuildDirtyAreas(const TArray<FNavigationDirtyArea>& DirtyAreas) override;

	/** Returns number of remaining tasks till build is complete */
	virtual int32 GetNumRemaningBuildTasks() const override { return WorkingTasks.Num(); }

	// ~ End FNavDataGenerator

	/** Returns a FBox with the sum of BBox, BBoxGrowth and the AgentHeight */
	FBox GrowBoundingBox(const FBox& BBox, bool bUseAgentHeight) const;

	/** Returns the NavMesh owner's world */
	UWorld* GetWorld() const { return GetOwner()->GetWorld(); }

	/** Returns the NavMesh owner */
	const TWeakObjectPtr<ANNNavMesh>& GetOwner() const { return NavMesh; }

	/** Fills the DebuggingInfo with the result of the FNNAreaGenerators */
	void GrabDebuggingInfo(FNNNavMeshDebuggingInfo& DebuggingInfo) const;

	/** Returns the quantity of tasks that are currently running */
	virtual int32 GetNumRunningBuildTasks() const override;

	/** Returns whether we are still calculating dirty areas */
	virtual bool IsBuildInProgressCheckDirty() const override;

protected:
	/** Creates a FNNAreaGenerator for every dirty area and makes them calculates it */
	void ProcessDirtyAreas();

	/** Returns a new FNNAreaGenerator */
	FNNAreaGenerator* CreateAreaGenerator(const FNavigationBounds& DirtyArea);

	/** Retrieves the results from the async tasks and tries to delete the ones canceled */
	void CheckAsyncTasks();

private:
	/** The NavMesh owner of this generator */
	TWeakObjectPtr<ANNNavMesh> NavMesh;

	/** The bounds assigned to this generator */
	const TSet<FNavigationBounds>& NavBounds;

	/** The areas that need to be calculated next tick */
	TArray<uint32> DirtyAreas;

	/** The saved data for each area */
	TMap<uint32, FNNAreaGeneratorData*> GeneratorsData;

	/** The tasks that are currently calculating the area bounds given by its key */
	TMap<uint32, FAsyncTask<FNNAreaGenerator>*> WorkingTasks;

	/** The tasks that need to be canceled and deleted */
	TArray<FAsyncTask<FNNAreaGenerator>*> CanceledTasks;

	/** Used to grow generic element bounds to match this generator's properties
	 *	(most notably Config.borderSize) */
	FVector BBoxGrowth;
};
