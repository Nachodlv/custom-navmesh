#include "Actors/NNCharacter.h"

// NN Includes
#include "Components/NNNavMovementComponent.h"


ANNCharacter::ANNCharacter(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer.SetDefaultSubobjectClass<UNNNavMovementComponent>(ACharacter::CharacterMovementComponentName))
{
}
