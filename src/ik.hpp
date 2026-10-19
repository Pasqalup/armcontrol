#pragma once

#include <array>


struct IKResult {
    int success;
    double theta1;
    double theta2;
    double theta3;
};
namespace modeled_c {

class IK {
public:
    IK(double l1, double l2);

    IKResult solve(double x, double y, double z) const;

private:
    double l1_;
    double l2_;
};

} // namespace modeled_c
