// Copyright Epic Games, Inc. All Rights Reserved.

#include "CustomNavmeshGameMode.h"
#include "CustomNavmeshCharacter.h"
#include "UObject/ConstructorHelpers.h"

ACustomNavmeshGameMode::ACustomNavmeshGameMode()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/ThirdPersonCPP/Blueprints/ThirdPersonCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}
