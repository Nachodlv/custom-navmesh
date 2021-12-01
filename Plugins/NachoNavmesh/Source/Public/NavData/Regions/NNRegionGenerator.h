#pragma once

struct FNNOpenHeightField;
struct FNNOpenSpan;
struct FNNRegion;

class FNNRegionGenerator
{
public:
	/** Initializes the regions of the open spans with a watershed algorithm */
	void CreateRegions(FNNOpenHeightField& OpenHeightField, float MinRegionSize) const;

protected:
	/** Sets the region to the CurrentSpan neighbours with the same WaterLevel */
	void FloodRegion(FNNOpenSpan* CurrentSpan, FNNRegion& Region, int32 CurrentWaterLevel) const;

	/** Grows the current regions created equally */
	void GrowRegions(FNNOpenHeightField& OpenHeightField, int32 CurrentWaterLevel) const;

	/** Deletes or combines regions with less than te MinSpansForRegions */
	void FilterSmallRegions(TArray<FNNRegion>& Regions, int32 MinSpansForRegions) const;
};
