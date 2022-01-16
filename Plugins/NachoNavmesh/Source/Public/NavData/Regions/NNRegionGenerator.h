#pragma once

struct FNNOpenHeightField;
struct FNNOpenSpan;
struct FNNRegion;

class FNNRegionGenerator
{
public:
	void CreateRegions(FNNOpenHeightField& OpenHeightField, float MinRegionSize, int32 TraversableAreaBorderSize) const;

protected:
	void FilterSmallRegions(TArray<FNNRegion>& Regions, int32 MinSpansForRegions) const;

	bool FloodNewRegion(FNNOpenSpan* RootSpan, int32 FillToDistance, TArray<FNNOpenSpan*>& WorkingStack, FNNRegion& NewRegion) const;

	/** Tries to find the most appropriate regions to attach spans to. Any span successfully assigned a region will
	 * be set to null in the Spans array */
	void ExpandRegions(TMap<int32, FNNRegion>& RegionsByID, TArray<FNNOpenSpan*>& Spans, int32 MaxIterations) const;
};
