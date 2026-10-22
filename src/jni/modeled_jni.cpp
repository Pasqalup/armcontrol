#include "jni_helpers.hpp"
#include "main.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using modeled_jni::readDoubleArray;
using modeled_jni::readIntArray;
using modeled_jni::requireFinite;
using modeled_jni::requireSize;
using modeled_jni::writeDoubleArray;

constexpr std::size_t kIkCount = 2;
constexpr std::size_t kTrackingCount = 4;
constexpr std::size_t kTrackingIntCount = 3;
constexpr std::size_t kPipelineCount = 3;
constexpr std::size_t kCameraCount = 12;
constexpr jsize kOutputCount = 4;

PipelineHandle* handleFromValue(jlong value) {
    if (value == 0) {
        throw std::invalid_argument("pipeline handle must not be null");
    }
    return reinterpret_cast<PipelineHandle*>(static_cast<intptr_t>(value));
}

jlong valueFromHandle(PipelineHandle* handle) {
    return static_cast<jlong>(reinterpret_cast<intptr_t>(handle));
}

void requirePositive(double value, const char* name) {
    requireFinite(value, name);
    if (value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be positive");
    }
}

void requireNonNegative(double value, const char* name) {
    requireFinite(value, name);
    if (value < 0.0) {
        throw std::invalid_argument(std::string(name) +
                                    " must be non-negative");
    }
}

void requireUnitInterval(double value, const char* name) {
    requireFinite(value, name);
    if (value < 0.0 || value > 1.0) {
        throw std::invalid_argument(std::string(name) +
                                    " must be between 0 and 1");
    }
}

int requireIntInRange(double value, int minimum, int maximum,
                      const char* name) {
    requireFinite(value, name);
    if (value < static_cast<double>(minimum) ||
        value > static_cast<double>(maximum) ||
        std::floor(value) != value) {
        throw std::invalid_argument(std::string(name) +
                                    " must be an integer in range");
    }
    return static_cast<int>(value);
}

} // namespace

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_flamingphoenix_nativebridge_modeled_NativePipeline_nativeCreate(
    JNIEnv* env, jclass, jdoubleArray ikArray, jdoubleArray trackingArray,
    jintArray trackingIntsArray, jdoubleArray pipelineArray,
    jdoubleArray cameraArray) {
    try {
        const auto ik = readDoubleArray(env, ikArray, "ik");
        const auto tracking = readDoubleArray(env, trackingArray, "tracking");
        const auto trackingInts =
            readIntArray(env, trackingIntsArray, "trackingInts");
        const auto pipeline = readDoubleArray(env, pipelineArray, "pipeline");
        const auto camera = readDoubleArray(env, cameraArray, "camera");

        requireSize(ik, kIkCount, "ik");
        requireSize(tracking, kTrackingCount, "tracking");
        requireSize(trackingInts, kTrackingIntCount, "trackingInts");
        requireSize(pipeline, kPipelineCount, "pipeline");
        requireSize(camera, kCameraCount, "camera");

        requirePositive(ik[0], "ik.l1");
        requirePositive(ik[1], "ik.l2");

        requireNonNegative(tracking[0], "tracking.maxDistance");
        requireUnitInterval(tracking[1], "tracking.velocityAlpha");
        requireNonNegative(tracking[2], "tracking.radiusWeight");
        requireNonNegative(tracking[3], "tracking.maxSpeed");

        requireIntInRange(static_cast<double>(trackingInts[0]), 0,
                          std::numeric_limits<int>::max(),
                          "trackingInts.maxMissedFrames");
        requireIntInRange(static_cast<double>(trackingInts[1]), 1,
                          std::numeric_limits<int>::max(),
                          "trackingInts.confirmationHits");
        requireIntInRange(static_cast<double>(trackingInts[2]), 0, 1,
                          "trackingInts.useKalman");

        requireNonNegative(pipeline[0], "pipeline.lookaheadTime");
        requirePositive(pipeline[1], "pipeline.trueRadius");
        requireFinite(pipeline[2], "pipeline.armBaseHeight");

        for (std::size_t i = 0; i < 9; ++i) {
            requireFinite(camera[i], "camera vector");
        }
        requirePositive(camera[9], "camera.focalLength");
        requireFinite(camera[10], "camera.principalX");
        requireFinite(camera[11], "camera.principalY");

        const CIKParameters ikParams{ik[0], ik[1]};
        const CTrackingParameters trackingParams{
            tracking[0],
            trackingInts[0],
            trackingInts[1],
            tracking[1],
            tracking[2],
            tracking[3],
            trackingInts[2] != 0 ? 1 : 0,
        };
        CCameraParameters cameraParams{};
        for (std::size_t i = 0; i < 3; ++i) {
            cameraParams.position[i] = camera[i];
            cameraParams.forward[i] = camera[i + 3];
            cameraParams.up[i] = camera[i + 6];
        }
        cameraParams.focalLength = camera[9];
        cameraParams.principalX = camera[10];
        cameraParams.principalY = camera[11];

        const CPipelineParameters pipelineParams{
            pipeline[0],
            pipeline[1],
            pipeline[2],
            cameraParams,
        };

        PipelineHandle* handle =
            pipeline_create(ikParams, trackingParams, pipelineParams);
        if (handle == nullptr) {
            throw std::runtime_error(
                "pipeline_create returned a null handle");
        }
        return valueFromHandle(handle);
    } catch (const std::invalid_argument& error) {
        modeled_jni::throwIllegalArgument(env, error.what());
    } catch (const std::exception& error) {
        modeled_jni::throwIllegalState(env, error.what());
    } catch (...) {
        modeled_jni::throwIllegalState(env, "unknown native error");
    }
    return 0;
}

JNIEXPORT jint JNICALL
Java_com_flamingphoenix_nativebridge_modeled_NativePipeline_nativeUpdate(
    JNIEnv* env, jclass, jlong handleValue, jdoubleArray detectionsArray,
    jdouble timestamp, jdoubleArray outArray) {
    try {
        PipelineHandle* handle = handleFromValue(handleValue);
        const auto detections =
            readDoubleArray(env, detectionsArray, "detections");

        if (detections.size() % 3 != 0) {
            throw std::invalid_argument(
                "detections must contain x, y, radius triples");
        }
        if (detections.size() >
            static_cast<std::size_t>(std::numeric_limits<int32_t>::max())) {
            throw std::invalid_argument("detections is too large");
        }
        for (double value : detections) {
            requireFinite(value, "detection value");
        }
        requireFinite(timestamp, "timestamp");
        if (outArray == nullptr) {
            throw std::invalid_argument("out must not be null");
        }
        if (env->GetArrayLength(outArray) < kOutputCount) {
            throw std::invalid_argument(
                "out must contain at least 4 values");
        }

        CIKResult result{};
        const int32_t count = static_cast<int32_t>(detections.size());
        const double* data = detections.empty() ? nullptr : detections.data();
        const int32_t status =
            pipeline_update(handle, data, count, timestamp, &result);
        if (status != 0) {
            throw std::runtime_error("pipeline_update failed with status " +
                                     std::to_string(status));
        }

        const std::array<double, kOutputCount> values{
            static_cast<double>(result.errorCode),
            result.theta1,
            result.theta2,
            result.theta3,
        };
        writeDoubleArray(env, outArray, values.data(), kOutputCount, "out");
        return status;
    } catch (const std::invalid_argument& error) {
        modeled_jni::throwIllegalArgument(env, error.what());
    } catch (const std::exception& error) {
        modeled_jni::throwIllegalState(env, error.what());
    } catch (...) {
        modeled_jni::throwIllegalState(env, "unknown native error");
    }
    return -3;
}

JNIEXPORT jint JNICALL
Java_com_flamingphoenix_nativebridge_modeled_NativePipeline_nativeSetTrueRadius(
    JNIEnv* env, jclass, jlong handleValue, jdouble radius) {
    try {
        PipelineHandle* handle = handleFromValue(handleValue);
        requirePositive(radius, "radius");
        const int32_t status = pipeline_set_true_radius(handle, radius);
        if (status != 0) {
            throw std::runtime_error(
                "pipeline_set_true_radius failed with status " +
                std::to_string(status));
        }
        return status;
    } catch (const std::invalid_argument& error) {
        modeled_jni::throwIllegalArgument(env, error.what());
    } catch (const std::exception& error) {
        modeled_jni::throwIllegalState(env, error.what());
    } catch (...) {
        modeled_jni::throwIllegalState(env, "unknown native error");
    }
    return -3;
}

JNIEXPORT void JNICALL
Java_com_flamingphoenix_nativebridge_modeled_NativePipeline_nativeDestroy(
    JNIEnv*, jclass, jlong handleValue) {
    if (handleValue == 0) {
        return;
    }
    pipeline_destroy(reinterpret_cast<PipelineHandle*>(
        static_cast<intptr_t>(handleValue)));
}

} // extern "C"