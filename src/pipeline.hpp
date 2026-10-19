#pragma once
#include "ik.hpp"
#include "object_tracking.hpp"

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
struct PipelineParameters {
    double lookaheadTime;
    double pixelToMeterRatio;
};

class Pipeline {
public:
    Pipeline(IKParameters ikParams, TrackingParameters trackingParams, PipelineParameters pipelineParams);
    IKResult update(std::vector<double> detections, double timestamp);
private:
    PipelineParameters pipelineParams_;
    modeled_c::IK ik_;
    modeled_c::ObjectTracker tracker_;
};