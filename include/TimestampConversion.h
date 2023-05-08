//
// Created by amanda on 10/18/22.
//

#ifndef ORB_SLAM3_TIMESTAMPCONVERSION_H
#define ORB_SLAM3_TIMESTAMPCONVERSION_H

#include <utility>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <Timestamp.h>

namespace ORB_SLAM3 {
    const static double kSecondsInNsec = 1e-9;
    const static double kNsecInSecond = 1e9;

    static inline double toDoubleInSeconds(const Timestamp &timestamp_pair) {
        return static_cast<double>(timestamp_pair.first) + kSecondsInNsec * static_cast<double>(timestamp_pair.second);
    }

    static inline Timestamp toTimestampPair(const double &timestamp_sec) {
        // Based on time conversion in ros (fromSec)
        int64_t sec64 = static_cast<int64_t>(floor(timestamp_sec));
        if ((sec64 < 0) || (sec64 > std::numeric_limits<uint32_t>::max())) {
            throw std::runtime_error("Time is out of dual 32-bit range");
        }
        uint32_t sec = static_cast<uint32_t>(sec64);
        uint32_t nsec = static_cast<uint32_t>((timestamp_sec - sec) * kNsecInSecond);
        // avoid rounding errors (per ROS header implementation)
        sec += (nsec / 1000000000ul);
        nsec %= 1000000000ul;
        return std::make_pair(sec, nsec);
    }

}

#endif //ORB_SLAM3_TIMESTAMPCONVERSION_H
