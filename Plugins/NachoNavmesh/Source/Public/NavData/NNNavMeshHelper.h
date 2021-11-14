#pragma once

// TODO (ignacio) this class probably will need to be deleted. I don't think it has usages.

class FNNNavMeshHelper
{
public:
	static bool CheckIfLineIntersectsWithBox(const FVector& B1, const FVector& B2, const FVector& L1, const FVector& L2, FVector& Hit);

protected:
	static bool GetIntersection(float fDst1, float fDst2, const FVector& P1, const FVector& P2, FVector& Hit);

	static bool InBox(const FVector& Hit, const FVector& B1, const FVector& B2, const int Axis);
};
