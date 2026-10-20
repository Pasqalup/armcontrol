#include "ik.hpp"
#include "object_tracking.hpp"
#include "pipeline.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace {

double dot(const std::array<double, 3>& left,
           const std::array<double, 3>& right) {
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

std::array<double, 3> cross(const std::array<double, 3>& left,
                            const std::array<double, 3>& right) {
    return {
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    };
}

double norm(const std::array<double, 3>& value) {
    return std::sqrt(dot(value, value));
}

std::array<double, 3> normalized(const std::array<double, 3>& value,
                                 const char* name) {
    const double length = norm(value);
    if (!std::isfinite(length) || length <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be nonzero");
    }
    return {value[0] / length, value[1] / length, value[2] / length};
}

} // namespace

Pipeline::Pipeline(IKParameters ikParams, TrackingParameters trackingParams, PipelineParameters pipelineParams) : ik_(ikParams.l1, ikParams.l2),
      tracker_(trackingParams.maxDistance, trackingParams.maxMissedFrames,
               trackingParams.confirmationHits, trackingParams.velocityAlpha,
               trackingParams.radiusWeight, trackingParams.maxSpeed,
               trackingParams.useKalman) {
                if (!std::isfinite(pipelineParams.lookaheadTime) ||
                    pipelineParams.lookaheadTime < 0.0) {
                    throw std::invalid_argument("lookahead time must be finite and non-negative");
                }
                if (!std::isfinite(pipelineParams.trueRadius) ||
                    pipelineParams.trueRadius <= 0.0) {
                    throw std::invalid_argument("true radius must be finite and positive");
                }
                if (!std::isfinite(pipelineParams.armBaseHeight)) {
                    throw std::invalid_argument("arm base height must be finite");
                }
                pipelineParams.camera.forward =
                    normalized(pipelineParams.camera.forward, "camera forward");
                pipelineParams.camera.up =
                    normalized(pipelineParams.camera.up, "camera up");
                const auto right = cross(pipelineParams.camera.up,
                                         pipelineParams.camera.forward);
                if (!std::isfinite(norm(right)) || norm(right) <= 0.0) {
                    throw std::invalid_argument("camera up and forward must not be parallel");
                }
                if (!std::isfinite(pipelineParams.camera.focalLength) ||
                    pipelineParams.camera.focalLength <= 0.0 ||
                    !std::isfinite(pipelineParams.camera.principalX) ||
                    !std::isfinite(pipelineParams.camera.principalY)) {
                    throw std::invalid_argument("invalid camera intrinsics");
                }
                pipelineParams_ = pipelineParams;
               }

void Pipeline::setTrueRadius(double radius) {
    if (!std::isfinite(radius) || radius <= 0.0) {
        throw std::invalid_argument("true radius must be finite and positive");
    }
    pipelineParams_.trueRadius = radius;
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
    tracker_.update(detectedObjects, timestamp);
    const auto confirmedTracks = tracker_.getConfirmedTracks();
    if (confirmedTracks.empty()) {
        return {-4, 0.0, 0.0, 0.0}; // No confirmed tracked objects available
    }
    const auto* firstTrackedObject = confirmedTracks[0];
    const auto prediction = firstTrackedObject->predict(pipelineParams_.lookaheadTime);
    const double pixelRadius = firstTrackedObject->radius;
    if (!std::isfinite(pixelRadius) || pixelRadius <= 0.0) {
        return {-5, 0.0, 0.0, 0.0}; // Invalid projected radius
    }

    const auto& camera = pipelineParams_.camera;
    const auto forward = camera.forward;
    const auto right = normalized(cross(forward, camera.up), "camera right");
    const auto up = normalized(cross(right, forward), "camera up");
    const double x = prediction[0] - camera.principalX;
    const double y = prediction[1] - camera.principalY;
    const std::array<double, 3> unnormalizedRay = {
        x * right[0] - y * up[0] + camera.focalLength * forward[0],
        x * right[1] - y * up[1] + camera.focalLength * forward[1],
        x * right[2] - y * up[2] + camera.focalLength * forward[2],
    };
    const auto ray = normalized(unnormalizedRay, "camera ray");
    const double focalToRadius = camera.focalLength / pixelRadius;
    const double cameraDistance = pipelineParams_.trueRadius *
                                  std::sqrt(1.0 + focalToRadius * focalToRadius);
    if (!std::isfinite(cameraDistance) || cameraDistance <= 0.0) {
        return {-6, 0.0, 0.0, 0.0}; // Invalid target distance
    }
    const std::array<double, 3> worldPosition = {
        camera.position[0] + cameraDistance * ray[0],
        camera.position[1] + cameraDistance * ray[1],
        camera.position[2] + cameraDistance * ray[2],
    };
    return ik_.solve(worldPosition[0], worldPosition[1],
                     worldPosition[2] - pipelineParams_.armBaseHeight);
}