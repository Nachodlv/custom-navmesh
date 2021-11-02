#include "Components/NNNavMovementComponent.h"

// NN Includes
#include "NavData/NNNavMesh.h"


UNNNavMovementComponent::UNNNavMovementComponent()
{
	NavAgentProps.PreferredNavData = FSoftClassPath(ANNNavMesh::StaticClass());
}

