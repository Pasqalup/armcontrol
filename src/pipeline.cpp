#include "ik.hpp"
#include "object_tracking.hpp"
#include "pipeline.hpp"

#include <cmath>
#include <stdexcept>

Pipeline::Pipeline(IKParameters ikParams, TrackingParameters trackingParams, PipelineParameters pipelineParams) : ik_(ikParams.l1, ikParams.l2),
      tracker_(trackingParams.maxDistance, trackingParams.maxMissedFrames,
               trackingParams.confirmationHits, trackingParams.velocityAlpha,
               trackingParams.radiusWeight, trackingParams.maxSpeed,
               trackingParams.useKalman) {
                if (!std::isfinite(pipelineParams.pixelToMeterRatio) ||
                    pipelineParams.pixelToMeterRatio <= 0.0) {
                    throw std::invalid_argument(
                        "pixel-to-meter ratio must be finite and positive");
                }
                pipelineParams_ = pipelineParams;
               }
IKResult Pipeline::update(std::vector<double> detections, double timestamp) {
    if (detections.size() % 3 != 0) {
        return {-3, 0.0, 0.0, 0.0}; // Invalid input size
    }
    std::vector<modeled_c::DetectedObject> detectedObjects;
    for (std::size_t i = 0; i < detections.size(); i += 3) {
        detectedObjects.push_back(
            {detections[i], detections[i + 1], detections[i + 2]});
    }
    auto trackedObjects = tracker_.update(detectedObjects, timestamp);
    if (trackedObjects.empty()) {
        return {-4, 0.0, 0.0, 0.0}; // No tracked objects available
    }
    const auto& firstTrackedObject = trackedObjects[0];
    const auto prediction = firstTrackedObject->predict(pipelineParams_.lookaheadTime);
    
    return ik_.solve((prediction[0] - 320 )/ pipelineParams_.pixelToMeterRatio, (prediction[1] - 240)/ pipelineParams_.pixelToMeterRatio, 0.05);
}