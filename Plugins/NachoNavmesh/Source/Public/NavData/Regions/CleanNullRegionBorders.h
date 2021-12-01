#pragma once

struct FNNOpenHeightField;
struct FNNOpenSpan;
struct FNNRegion;

/**
* Implements three algorithms that clean up issues that can
* develop around null region boarders.
*
* First algorithm:
*	Detect and fix encompassed null regions:
*	If a null region is found that is fully encompassed by a single
*	region, then the region will be split into two regions at the
*	null region border.
*
* Second algorithm:
*	Detect and fix "short wrapping" of null regions
*	Regions can sometimes wrap slightly around the corner of a null region
*   in a manner that eventually results in the formation of self-intersecting
*	polygons
*
* Third algorithm:
*	Detect and fix incomplete null region connections
*	If a region touches null region only diagonally, then contour detection
*	algorithms may not properly detect the null region connection. This can
*	adversely effect other algorithms in the pipeline
*/

class FNNCleanNullRegionBorders
{
public:
	FNNCleanNullRegionBorders(FNNOpenHeightField& InOpenHeightField) : OpenHeightField(InOpenHeightField) {}

	/** Applies the three algorithms to fix the region borders.
	 * Expects that the OpenHEightField has the Regions created. */
	void CleanNullRegionBorders();

protected:
	static int32 GetNonNullBorderDirection(const FNNOpenSpan& OpenSpan);

	bool ProcessNullRegion(FNNOpenSpan& StartSpan, int32 StartDirection);

	/** Detects and fixes configuration issues in the vicinity of a obtuse (outer) null region corner.
	 * Returns true if more than one region connects to the null region in the vicinity of the corner */
	bool ProcessOuterCorner(FNNOpenSpan& CurrentSpan, int32 BorderDirection);

	static void PartialFloodRegion(FNNOpenSpan& StartSpan, int32 BorderDirection, FNNRegion& StartSpanRegion, FNNRegion& NewRegion);

	/** Checks if the ReferenceSpan should be reassigned to a new region */
	int32 SelectRegionID(const FNNOpenSpan& ReferenceSpan, int32 BorderDirection, int32 CornerDirection);

	void ChangeRegion(FNNOpenSpan& ReferenceSpan, int32 NewRegionID) const;

private:
	FNNOpenHeightField& OpenHeightField;
};
