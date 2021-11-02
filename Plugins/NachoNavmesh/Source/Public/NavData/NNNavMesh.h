#pragma once

// UE Includes
#include "CoreMinimal.h"
#include "NavigationData.h"

#include "NNNavMesh.generated.h"

UCLASS()
class NACHONAVMESH_API ANNNavMesh : public ANavigationData
{
	GENERATED_BODY()

public:
	ANNNavMesh();

	static FPathFindingResult FindPath(const FNavAgentProperties& AgentProperties, const FPathFindingQuery& Query);

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
};
