#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

namespace modeled_c {

class KalmanFilter {
public:
    explicit KalmanFilter(double measurementNoise = 1.0,
                          double processNoise = 0.1);

    void reset(double x, double y, double dt);
    std::array<double, 4> predict(double dt);
    std::array<double, 4> update(double x, double y, double dt);
    std::array<double, 2> estimatePosition() const;
    std::array<double, 2> estimateVelocity() const;

private:
    std::array<double, 4> state_{};
    std::array<std::array<double, 4>, 4> covariance_{};
    double measurementNoise_;
    double processNoise_;
    bool initialized_ = false;
    double lastUpdateDt_ = 0.0;
};

struct DetectedObject {
    double x;
    double y;
    double radius;
};

std::vector<DetectedObject> parseCenters(const std::vector<double>& centers);

class TrackedObject {
public:
    TrackedObject(int objectId, const DetectedObject& detection,
                  double timestamp);
    virtual ~TrackedObject() = default;

    virtual std::array<double, 2> predict(double time) const;
    virtual void update(const DetectedObject& detection, double timestamp,
                        double velocityAlpha);
    void predictAt(double timestamp);

    int id;
    double x;
    double y;
    double radius;
    double vx = 0.0;
    double vy = 0.0;
    double lastSeenTime;
    int missedFrames = 0;
    int hits = 1;
    bool confirmed = false;
    double predictedX;
    double predictedY;
};

class KalmanTrackedObject final : public TrackedObject {
public:
    KalmanTrackedObject(int objectId, const DetectedObject& detection,
                        double timestamp,
                        std::unique_ptr<KalmanFilter> filter = nullptr);

    std::array<double, 2> predict(double time) const override;
    void update(const DetectedObject& detection, double timestamp,
                double velocityAlpha) override;

private:
    std::unique_ptr<KalmanFilter> filter_;
};

class ObjectTracker {
public:
    ObjectTracker(double maxDistance = 40.0, int maxMissedFrames = 5,
                  int confirmationHits = 2, double velocityAlpha = 0.4,
                  double radiusWeight = 0.25, double maxSpeed = 1000.0,
                  bool useKalman = false);

    std::vector<TrackedObject*> update(
        const std::vector<DetectedObject>& detections, double timestamp);
    std::vector<TrackedObject*> getConfirmedTracks() const;

private:
    std::vector<std::unique_ptr<TrackedObject>> tracks_;
    int nextId_ = 1;
    double maxDistance_;
    int maxMissedFrames_;
    int confirmationHits_;
    double velocityAlpha_;
    double radiusWeight_;
    double maxSpeed_;
    bool useKalman_;
    bool hasTimestamp_ = false;
    double lastTimestamp_ = 0.0;
};

} // namespace modeled_c
