#pragma once
#include "ik.hpp"
#include "object_tracking.hpp"

#include <array>

struct IKParameters {
    double l1;
    double l2;
};
struct TrackingParameters {
    double maxDistance;
    int maxMissedFrames;
    int confirmationHits;
    double velocityAlpha;
    double radiusWeight;
    double maxSpeed;
    bool useKalman;
};
struct CameraParameters {
    std::array<double, 3> position;
    std::array<double, 3> forward;
    std::array<double, 3> up;
    double focalLength;
    double principalX;
    double principalY;
};
struct PipelineParameters {
    double lookaheadTime;
    double trueRadius;
    double armBaseHeight;
    CameraParameters camera;
};

class Pipeline {
public:
    Pipeline(IKParameters ikParams, TrackingParameters trackingParams, PipelineParameters pipelineParams);
    IKResult update(std::vector<double> detections, double timestamp);
    void setTrueRadius(double radius);
private:
    PipelineParameters pipelineParams_;
    modeled_c::IK ik_;
    modeled_c::ObjectTracker tracker_;
};