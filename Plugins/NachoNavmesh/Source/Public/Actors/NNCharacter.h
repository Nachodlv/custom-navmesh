#pragma once

// UE Includes
#include "CoreMinimal.h"
#include "GameFramework/Character.h"

#include "NNCharacter.generated.h"

UCLASS()
class NACHONAVMESH_API ANNCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	/** Override the MovementComponent class */
	ANNCharacter(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());
};
