#include "object_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace modeled_c {
namespace {

using Matrix4 = std::array<std::array<double, 4>, 4>;

Matrix4 identity4() {
    Matrix4 result{};
    for (std::size_t i = 0; i < 4; ++i) {
        result[i][i] = 1.0;
    }
    return result;
}

Matrix4 transition(double dt) {
    Matrix4 result = identity4();
    result[0][2] = dt;
    result[1][3] = dt;
    return result;
}

Matrix4 multiply(const Matrix4& left, const Matrix4& right) {
    Matrix4 result{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            for (std::size_t i = 0; i < 4; ++i) {
                result[row][column] += left[row][i] * right[i][column];
            }
        }
    }
    return result;
}

Matrix4 transpose(const Matrix4& matrix) {
    Matrix4 result{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            result[row][column] = matrix[column][row];
        }
    }
    return result;
}

} // namespace

KalmanFilter::KalmanFilter(double measurementNoise, double processNoise)
    : measurementNoise_(measurementNoise), processNoise_(processNoise),
      covariance_{} {
    if (measurementNoise < 0.0 || processNoise < 0.0) {
        throw std::invalid_argument("noise values must be non-negative");
    }
    covariance_ = identity4();
    for (auto& row : covariance_) {
        for (double& value : row) {
            value *= 10.0;
        }
    }
}

void KalmanFilter::reset(double x, double y, double dt) {
    state_ = {x, y, 0.0, 0.0};
    covariance_ = identity4();
    for (auto& row : covariance_) {
        for (double& value : row) {
            value *= 10.0;
        }
    }
    initialized_ = true;
    lastUpdateDt_ = dt;
}

std::array<double, 4> KalmanFilter::predict(double dt) {
    if (!initialized_) {
        return state_;
    }
    const Matrix4 f = transition(dt);
    std::array<double, 4> next{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t i = 0; i < 4; ++i) {
            next[row] += f[row][i] * state_[i];
        }
    }
    state_ = next;
    covariance_ = multiply(multiply(f, covariance_), transpose(f));
    for (std::size_t i = 0; i < 4; ++i) {
        covariance_[i][i] += processNoise_;
    }
    return state_;
}

std::array<double, 4> KalmanFilter::update(double x, double y, double dt) {
    lastUpdateDt_ = dt;
    predict(dt);

    const double innovationX = x - state_[0];
    const double innovationY = y - state_[1];
    const double s00 = covariance_[0][0] + measurementNoise_;
    const double s01 = covariance_[0][1];
    const double s10 = covariance_[1][0];
    const double s11 = covariance_[1][1] + measurementNoise_;
    const double determinant = s00 * s11 - s01 * s10;
    if (std::abs(determinant) <= std::numeric_limits<double>::epsilon()) {
        throw std::runtime_error("Kalman innovation covariance is singular");
    }

    const double inv00 = s11 / determinant;
    const double inv01 = -s01 / determinant;
    const double inv10 = -s10 / determinant;
    const double inv11 = s00 / determinant;
    std::array<std::array<double, 2>, 4> gain{};
    for (std::size_t row = 0; row < 4; ++row) {
        gain[row][0] = covariance_[row][0] * inv00 +
                       covariance_[row][1] * inv10;
        gain[row][1] = covariance_[row][0] * inv01 +
                       covariance_[row][1] * inv11;
    }
    for (std::size_t row = 0; row < 4; ++row) {
        state_[row] += gain[row][0] * innovationX +
                       gain[row][1] * innovationY;
    }

    Matrix4 updated = covariance_;
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            updated[row][column] -= gain[row][0] * covariance_[0][column] +
                                    gain[row][1] * covariance_[1][column];
        }
    }
    covariance_ = updated;
    return state_;
}

std::array<double, 2> KalmanFilter::estimatePosition() const {
    return {state_[0], state_[1]};
}

std::array<double, 2> KalmanFilter::estimateVelocity() const {
    return {state_[2], state_[3]};
}

ScalarKalmanFilter::ScalarKalmanFilter(double measurementNoise,
                                       double processNoise)
    : measurementNoise_(measurementNoise), processNoise_(processNoise) {
    if (measurementNoise < 0.0 || processNoise < 0.0) {
        throw std::invalid_argument("noise values must be non-negative");
    }
}

void ScalarKalmanFilter::reset(double value) {
    state_ = value;
    variance_ = 1.0;
    initialized_ = true;
}

double ScalarKalmanFilter::update(double measurement) {
    if (!initialized_) {
        reset(measurement);
        return state_;
    }
    variance_ += processNoise_;
    const double gain = variance_ / (variance_ + measurementNoise_);
    state_ += gain * (measurement - state_);
    variance_ *= 1.0 - gain;
    return state_;
}

double ScalarKalmanFilter::estimate() const {
    return state_;
}

std::vector<DetectedObject> parseCenters(const std::vector<double>& centers) {
    if (centers.size() % 3 != 0) {
        throw std::invalid_argument(
            "centers must contain x, y, radius triples");
    }
    std::vector<DetectedObject> result;
    result.reserve(centers.size() / 3);
    for (std::size_t i = 0; i < centers.size(); i += 3) {
        result.push_back({centers[i], centers[i + 1], centers[i + 2]});
    }
    return result;
}

TrackedObject::TrackedObject(int objectId, const DetectedObject& detection,
                             double timestamp)
    : id(objectId), x(detection.x), y(detection.y),
      radius(detection.radius), lastSeenTime(timestamp), predictedX(x),
      predictedY(y) {
    radiusFilter_.reset(detection.radius);
}

void TrackedObject::updateRadius(double measurement) {
    radius = radiusFilter_.update(measurement);
}

std::array<double, 2> TrackedObject::predict(double time) const {
    if (time < 0.0) {
        throw std::invalid_argument("time must be non-negative");
    }
    return {x + vx * time, y + vy * time};
}

void TrackedObject::predictAt(double timestamp) {
    const double dt = std::max(0.0, timestamp - lastSeenTime);
    const auto prediction = predict(dt);
    predictedX = prediction[0];
    predictedY = prediction[1];
}

void TrackedObject::update(const DetectedObject& detection,
                           double timestamp, double velocityAlpha) {
    const double dt = timestamp - lastSeenTime;
    if (dt > 0.0) {
        const double measuredVx = (detection.x - x) / dt;
        const double measuredVy = (detection.y - y) / dt;
        vx = velocityAlpha * measuredVx + (1.0 - velocityAlpha) * vx;
        vy = velocityAlpha * measuredVy + (1.0 - velocityAlpha) * vy;
    }
    x = detection.x;
    y = detection.y;
    updateRadius(detection.radius);
    lastSeenTime = timestamp;
    missedFrames = 0;
    ++hits;
}

KalmanTrackedObject::KalmanTrackedObject(
    int objectId, const DetectedObject& detection, double timestamp,
    std::unique_ptr<KalmanFilter> filter)
    : TrackedObject(objectId, detection, timestamp),
      filter_(filter ? std::move(filter) : std::make_unique<KalmanFilter>()) {
    filter_->reset(detection.x, detection.y, 0.0);
}

std::array<double, 2> KalmanTrackedObject::predict(double time) const {
    if (time < 0.0) {
        throw std::invalid_argument("time must be non-negative");
    }
    return {x + vx * time, y + vy * time};
}

void KalmanTrackedObject::update(const DetectedObject& detection,
                                 double timestamp, double) {
    const double dt = std::max(1e-9, timestamp - lastSeenTime);
    filter_->update(detection.x, detection.y, dt);
    const auto position = filter_->estimatePosition();
    const auto velocity = filter_->estimateVelocity();
    x = position[0];
    y = position[1];
    vx = velocity[0];
    vy = velocity[1];
    updateRadius(detection.radius);
    lastSeenTime = timestamp;
    missedFrames = 0;
    ++hits;
}

ObjectTracker::ObjectTracker(double maxDistance, int maxMissedFrames,
                             int confirmationHits, double velocityAlpha,
                             double radiusWeight, double maxSpeed,
                             bool useKalman)
    : maxDistance_(maxDistance), maxMissedFrames_(maxMissedFrames),
      confirmationHits_(confirmationHits), velocityAlpha_(velocityAlpha),
      radiusWeight_(radiusWeight), maxSpeed_(maxSpeed), useKalman_(useKalman) {
    if (maxDistance < 0.0 || maxMissedFrames < 0 || confirmationHits < 1 ||
        velocityAlpha < 0.0 || velocityAlpha > 1.0 || radiusWeight < 0.0 ||
        maxSpeed < 0.0) {
        throw std::invalid_argument("invalid tracker configuration");
    }
}

std::vector<TrackedObject*> ObjectTracker::update(
    const std::vector<DetectedObject>& detections, double timestamp) {
    if (hasTimestamp_ && timestamp < lastTimestamp_) {
        throw std::invalid_argument("timestamp must be monotonic");
    }
    hasTimestamp_ = true;
    lastTimestamp_ = timestamp;

    for (auto& track : tracks_) {
        track->predictAt(timestamp);
    }

    struct Candidate {
        double cost;
        std::size_t trackIndex;
        std::size_t detectionIndex;
    };
    std::vector<Candidate> candidates;
    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
        for (std::size_t di = 0; di < detections.size(); ++di) {
            const double dx = detections[di].x - tracks_[ti]->predictedX;
            const double dy = detections[di].y - tracks_[ti]->predictedY;
            const double distanceSquared = dx * dx + dy * dy;
            if (distanceSquared > maxDistance_ * maxDistance_) {
                continue;
            }
            const double radiusScale = std::max(tracks_[ti]->radius, 1.0);
            const double radiusDifference =
                (detections[di].radius - tracks_[ti]->radius) / radiusScale;
            const double cost = distanceSquared +
                radiusWeight_ * maxDistance_ * maxDistance_ *
                    radiusDifference * radiusDifference;
            candidates.push_back({cost, ti, di});
        }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& left, const Candidate& right) {
                  return left.cost < right.cost;
              });

    std::vector<bool> matchedTracks(tracks_.size(), false);
    std::vector<bool> matchedDetections(detections.size(), false);
    for (const auto& candidate : candidates) {
        if (matchedTracks[candidate.trackIndex] ||
            matchedDetections[candidate.detectionIndex]) {
            continue;
        }
        tracks_[candidate.trackIndex]->update(
            detections[candidate.detectionIndex], timestamp, velocityAlpha_);
        matchedTracks[candidate.trackIndex] = true;
        matchedDetections[candidate.detectionIndex] = true;
    }

    for (std::size_t i = 0; i < tracks_.size(); ++i) {
        if (!matchedTracks[i]) {
            ++tracks_[i]->missedFrames;
        }
    }
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [this](const auto& track) {
                           return track->missedFrames > maxMissedFrames_;
                       }),
        tracks_.end());

    for (std::size_t di = 0; di < detections.size(); ++di) {
        if (matchedDetections[di]) {
            continue;
        }
        std::unique_ptr<TrackedObject> track;
        if (useKalman_) {
            track = std::make_unique<KalmanTrackedObject>(
                nextId_, detections[di], timestamp);
        } else {
            track = std::make_unique<TrackedObject>(
                nextId_, detections[di], timestamp);
        }
        ++nextId_;
        if (confirmationHits_ <= 1) {
            track->confirmed = true;
        }
        tracks_.push_back(std::move(track));
    }

    for (auto& track : tracks_) {
        if (track->hits >= confirmationHits_) {
            track->confirmed = true;
        }
    }

    std::vector<TrackedObject*> result;
    result.reserve(tracks_.size());
    for (auto& track : tracks_) {
        result.push_back(track.get());
    }
    return result;
}

std::vector<TrackedObject*> ObjectTracker::getConfirmedTracks() const {
    std::vector<TrackedObject*> result;
    for (const auto& track : tracks_) {
        if (track->confirmed) {
            result.push_back(track.get());
        }
    }
    return result;
}

} // namespace modeled_c
