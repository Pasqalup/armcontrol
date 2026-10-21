package org.firstinspires.ftc.teamcode.modeled;


/*
turn structs into arrays that pass through JNI
*/
public final class NativeStructs {
    class IKParams{
        double l1, l2;
        IKParams(double l1, double l2){
            this.l1 = l1;
            this.l2 = l2;
        }
        double[] getParams(){
            return new double[]{l1, l2};
        }
    }
    class TrackingParams{
        double maxDistance;
        int maxMissedFrames;
        int confirmationHits;
        double velocityAlpha;
        double radiusWeight;
        double maxSpeed;
        boolean useKalman;
        TrackingParams(double maxDistance, int maxMissedFrames, int confirmationHits, double velocityAlpha, double radiusWeight, double maxSpeed, boolean useKalman){
            this.maxDistance = maxDistance;
            this.maxMissedFrames = maxMissedFrames;
            this.confirmationHits = confirmationHits;
            this.velocityAlpha = velocityAlpha;
            this.radiusWeight = radiusWeight;
            this.maxSpeed = maxSpeed;
            this.useKalman = useKalman;
        }
        double[] getDoubleParams(){
            return new double[]{maxDistance, velocityAlpha, radiusWeight, maxSpeed};
        }
        int[] getIntParams(){
            return new int[]{maxMissedFrames, confirmationHits, useKalman ? 1 : 0};
        }
    }
    class CameraParams{
        double[] position;
        double[] forward;
        double[] up;
        double focalLength;
        double principalX;
        double principalY;
        CameraParams(double[] position, double[] forward, double[] up, double focalLength, double principalX, double principalY){
            this.position = position;
            this.forward = forward;
            this.up = up;
            this.focalLength = focalLength;
            this.principalX = principalX;
            this.principalY = principalY;
        }
        double[] getParams(){
            return new double[]{
                    position[0], position[1], position[2],
                    forward[0], forward[1], forward[2],
                    up[0], up[1], up[2],
                    focalLength, principalX, principalY
            };
        }
    }
    class PipelineParams{
        double lookaheadTime;
        double trueRadius;
        double armBaseHeight;
        CameraParams camera;
        PipelineParams(double lookaheadTime, double trueRadius, double armBaseHeight, CameraParams camera){
            this.lookaheadTime = lookaheadTime;
            this.trueRadius = trueRadius;
            this.armBaseHeight = armBaseHeight;
            this.camera = camera;
        }
        double[] getParams(){
            double[] cameraParams = camera.getParams();
            double[] params = new double[3 + cameraParams.length];
            params[0] = lookaheadTime;
            params[1] = trueRadius;
            params[2] = armBaseHeight;
            System.arraycopy(cameraParams, 0, params, 3, cameraParams.length);
            return params;
        }
    }

}

/*
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
*/