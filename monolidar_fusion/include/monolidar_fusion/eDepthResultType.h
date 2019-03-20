
#pragma once

#include <map>
#include <string>

namespace Mono_Lidar {

enum DepthResultType {
    Unspecified = 0,
    Success = 1,
    RadiusSearchInsufficientPoints = 2,
    HistogramNoLocalMax = 3,
    TresholdDepthGlobalGreaterMax = 4,
    TresholdDepthGlobalSmallerMin = 5,
    TresholdDepthLocalGreaterMax = 6,
    TresholdDepthLocalSmallerMin = 7,
    TriangleNotPlanar = 8,
    TriangleNotPlanarInsufficientPoints = 9,
    CornerBehindCamera = 10,
    PlaneViewrayNotOrthogonal = 11,
    PcaIsPoint = 12,
    PcaIsLine = 13,
    PcaIsCubic = 14,
    InsufficientRoadPoints = 15,
    SuccessRoad = 16,
    RegionGrowingNearestSeedNotAvailable = 17,
    RegionGrowingSeedsOutOfRange = 18,
    RegionGrowingInsufficientPoints = 19,
    SuccessRegionGrowing = 20
};

static const char * DepthResultTypeStrings[] {"Unspecified",
    "Success",
    "RadiusSearchInsufficientPoints",
    "HistogramNoLocalMax",
    "TresholdDepthGlobalGreaterMax",
    "TresholdDepthGlobalSmallerMin",
    "TresholdDepthLocalGreaterMax",
    "TresholdDepthLocalSmallerMin",
    "TriangleNotPlanar",
    "TriangleNotPlanarInsufficientPoints",
    "CornerBehindCamera",
    "PlaneViewrayNotOrthogonal",
    "PcaIsPoint",
    "PcaIsLine",
    "PcaIsCubic",
    "InsufficientRoadPoints",
    "SuccessRoad",
    "RegionGrowingNearestSeedNotAvailable",
    "RegionGrowingSeedsOutOfRange",
    "RegionGrowingInsufficientPoints",
    "SuccessRegionGrowing"};

static const char * getTextForDepthResultType(int enumValue) { return DepthResultTypeStrings[enumValue]; }

}
