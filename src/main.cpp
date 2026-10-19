
#include "main.hpp"
#include "pipeline.hpp"

#include <vector>

struct PipelineHandle {
    Pipeline* pipeline;

    PipelineHandle(
        CIKParameters ikParams,
        CTrackingParameters trackingParams,
        CPipelineParameters pipelineParams
    )
        : pipeline(new Pipeline(
            {ikParams.l1, ikParams.l2},
            {
                trackingParams.maxDistance,
                trackingParams.maxMissedFrames,
                trackingParams.confirmationHits,
                trackingParams.velocityAlpha,
                trackingParams.radiusWeight,
                trackingParams.maxSpeed,
                trackingParams.useKalman != 0
            },
            {
                pipelineParams.lookaheadTime,
                pipelineParams.pixelToMeterRatio
            }
        )) {}

    ~PipelineHandle() {
        delete pipeline;
    }
};

extern "C" {

int32_t pipeline_update(
    PipelineHandle* handle,
    const double* detections,
    int32_t count,
    double timestamp,
    CIKResult* output
) {
    if (!handle || !handle->pipeline || !output ||
        count < 0 || (count > 0 && !detections)) {
        return -1;
    }

    try {
        std::vector<double> detectionVec;

        if (count > 0) {
            detectionVec.assign(detections, detections + count);
        }

        IKResult result = handle->pipeline->update(detectionVec, timestamp);

        output->errorCode = result.success;
        output->theta1 = result.theta1;
        output->theta2 = result.theta2;
        output->theta3 = result.theta3;

        return 0;
    } catch (...) {
        return -2;
    }
}

PipelineHandle* pipeline_create(
    CIKParameters ikParams,
    CTrackingParameters trackingParams,
    CPipelineParameters pipelineParams
) {
    try {
        return new PipelineHandle(
            ikParams, trackingParams, pipelineParams
        );
    } catch (...) {
        return nullptr;
    }
}

void pipeline_destroy(PipelineHandle* handle) {
    delete handle;
}

}
