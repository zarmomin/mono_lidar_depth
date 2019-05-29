/*
 * TresholdDepth.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: wilczynski
 */

#include <monolidar_fusion/TresholdDepthGlobal.h>
#include <string>

namespace Mono_Lidar {
TresholdDepthGlobal::TresholdDepthGlobal(const eTresholdDepthMode& mode, const double minValue, const double maxValue)
        : _mode(mode), _minValue(minValue), _maxValue(maxValue) {
}

eTresholdResult TresholdDepthGlobal::CheckInDepth(double& depth) {
    if (depth < _minValue) {
        if (_mode == eTresholdDepthMode::Dispose) {
            depth = -1;
            return eTresholdResult::SmallerMin;
        } else if (_mode == eTresholdDepthMode::Adjust)
            depth = _minValue;
    } else if (depth > _maxValue) {
        if (_mode == eTresholdDepthMode::Dispose) {
            depth = -1;
            return eTresholdResult::GreaterMax;
        } else if (_mode == eTresholdDepthMode::Adjust)
            depth = _maxValue;
    }

    return eTresholdResult::InBounds;
}
}
