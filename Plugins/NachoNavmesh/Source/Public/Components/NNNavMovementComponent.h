#pragma once

// UE Includes
#include "CoreMinimal.h"
#include "GameFramework/CharacterMovementComponent.h"

#include "NNNavMovementComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class NACHONAVMESH_API UNNNavMovementComponent : public UCharacterMovementComponent
{
	GENERATED_BODY()

public:
	/** Overrides the preferred NavData class from the NavAgentProps */
	UNNNavMovementComponent();

};
