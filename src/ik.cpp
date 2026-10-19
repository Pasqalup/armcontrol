#include "ik.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace modeled_c {

IK::IK(double l1, double l2) : l1_(l1), l2_(l2) {
    if (l1 <= 0.0 || l2 <= 0.0) {
        throw std::invalid_argument("link lengths must be positive");
    }
}

IKResult IK::solve(double x, double y, double z) const {
    constexpr double pi = 3.14159265358979323846;
    const double baseAngle = std::atan2(y, x);
    const double projectedX = std::hypot(x, y);
    const double distance = std::hypot(projectedX, z);

    if (distance > l1_ + l2_) {
        return {-1, 0.0, 0.0, 0.0}; // Target point is unreachable
    }
    if (distance == 0.0) {
        return {-2, 0.0, 0.0, 0.0}; // Target point is at the origin
    }

    const double cosAngle1 = std::max(
        -1.0,
        std::min(
            1.0,
            (l1_ * l1_ + distance * distance - l2_ * l2_) /
                (2.0 * l1_ * distance)));
    const double cosAngle2 = std::max(
        -1.0,
        std::min(
            1.0,
            (l1_ * l1_ + l2_ * l2_ - distance * distance) /
                (2.0 * l1_ * l2_)));
    const double angle1 = std::acos(cosAngle1);
    const double angle2 = std::acos(cosAngle2);

    return {
        0,
        baseAngle,
        std::atan2(z, projectedX) + angle1,
        pi - angle2
    };
}

} // namespace modeled_c