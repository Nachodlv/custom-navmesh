#pragma once

class FNNNavMeshHelper
{
public:
	static bool CheckIfLineIntersectsWithBox(const FVector& B1, const FVector& B2, const FVector& L1, const FVector& L2, FVector& Hit);

	/** Returns the normal of the polygon provided */
	static FVector CalculatePolygonNormal(const TArray<FVector>& Polygon);

protected:
	static bool GetIntersection(float fDst1, float fDst2, const FVector& P1, const FVector& P2, FVector& Hit);

	static bool InBox(const FVector& Hit, const FVector& B1, const FVector& B2, const int Axis);
};
