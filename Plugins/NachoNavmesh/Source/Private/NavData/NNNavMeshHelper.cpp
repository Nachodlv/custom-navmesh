#include "NavData/NNNavMeshHelper.h"

bool FNNNavMeshHelper::CheckIfLineIntersectsWithBox(const FVector& B1, const FVector& B2, const FVector& L1, const FVector& L2, FVector& Hit)
{
	if (L2.X < B1.X && L1.X < B1.X)
	{
		return false;
	}
	if (L2.X > B2.X && L1.X > B2.X)
	{
		return false;
	}
	if (L2.Y < B1.Y && L1.Y < B1.Y)
	{
		return false;
	}
	if (L2.Y > B2.Y && L1.Y > B2.Y)
	{
		return false;
	}
	if (L2.Z < B1.Z && L1.Z < B1.Z)
	{
		return false;
	}
	if (L2.Z > B2.Z && L1.Z > B2.Z)
	{
		return false;
	}
	if (L1.X > B1.X && L1.X < B2.X && L1.Y > B1.Y && L1.Y < B2.Y && L1.Z > B1.Z && L1.Z < B2.Z)
	{
		Hit = L1;
		return true;
	}
	if ( (GetIntersection( L1.X-B1.X, L2.X-B1.X, L1, L2, Hit) && InBox( Hit, B1, B2, 1 ))
	  || (GetIntersection( L1.Y-B1.Y, L2.Y-B1.Y, L1, L2, Hit) && InBox( Hit, B1, B2, 2 ))
	  || (GetIntersection( L1.Z-B1.Z, L2.Z-B1.Z, L1, L2, Hit) && InBox( Hit, B1, B2, 3 ))
	  || (GetIntersection( L1.X-B2.X, L2.X-B2.X, L1, L2, Hit) && InBox( Hit, B1, B2, 1 ))
	  || (GetIntersection( L1.Y-B2.Y, L2.Y-B2.Y, L1, L2, Hit) && InBox( Hit, B1, B2, 2 ))
	  || (GetIntersection( L1.Z-B2.Z, L2.Z-B2.Z, L1, L2, Hit) && InBox( Hit, B1, B2, 3 )))
	{
		return true;
	}

	return false;
}

bool FNNNavMeshHelper::GetIntersection(float fDst1, float fDst2, const FVector& P1, const FVector& P2, FVector& Hit)
{
	if ( (fDst1 * fDst2) >= 0.0f)
	{
		return false;
	}
	if ( fDst1 == fDst2)
	{
		return false;
	}
	Hit = P1 + (P2-P1) * (-fDst1/(fDst2-fDst1));
	return true;
}

bool FNNNavMeshHelper::InBox(const FVector& Hit, const FVector& B1, const FVector& B2, const int Axis)
{
	if ( Axis==1 && Hit.Z > B1.Z && Hit.Z < B2.Z && Hit.Y > B1.Y && Hit.Y < B2.Y)
	{
		return true;
	}
	if ( Axis==2 && Hit.Z > B1.Z && Hit.Z < B2.Z && Hit.X > B1.X && Hit.X < B2.X)
	{
		return true;
	}
	if ( Axis==3 && Hit.X > B1.X && Hit.X < B2.X && Hit.Y > B1.Y && Hit.Y < B2.Y)
	{
		return true;
	}
	return false;
}
