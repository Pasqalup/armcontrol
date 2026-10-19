#pragma once

#include "pipeline.hpp"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CIKParameters {
    double l1;
    double l2;
} CIKParameters;
typedef struct CTrackingParameters {
    double maxDistance;
    int maxMissedFrames;
    int confirmationHits;
    double velocityAlpha;
    double radiusWeight;
    double maxSpeed;
    bool useKalman;
} CTrackingParameters;
typedef struct CPipelineParameters {
    double lookaheadTime;
    double pixelToMeterRatio;
} CPipelineParameters;
typedef struct CIKResult {
    int errorCode;
    double theta1;
    double theta2;
    double theta3;
} CIKResult;

typedef struct PipelineHandle PipelineHandle;
PipelineHandle* pipeline_create(CIKParameters ikParams, CTrackingParameters trackingParams, CPipelineParameters pipelineParams);

void pipeline_destroy(PipelineHandle* handle);

int32_t pipeline_update(
    PipelineHandle* handle, 
    const double* detections, 
    int32_t count,
     double timestamp,
    CIKResult* output
);


#ifdef __cplusplus
}
#endif