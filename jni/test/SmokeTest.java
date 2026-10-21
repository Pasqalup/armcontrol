import org.firstinspires.ftc.teamcode.modeled.NativePipeline;

public final class SmokeTest {
    public static void main(String[] args) {
        double[] ik = {0.4, 0.4};
        double[] tracking = {50.0, 1.0, 1.0, 1000.0};
        int[] trackingInts = {5, 2, 0};
        double[] pipeline = {0.5, 0.0762, 0.15};
        double[] camera = {
            0.0, 0.0, 1.0,
            0.0, 0.0, -1.0,
            0.0, 1.0, 0.0,
            415.8, 320.0, 240.0
        };
        double[] detections = {320.0, 240.0, 60.0};
        double[] out = new double[4];

        try (NativePipeline nativePipeline = NativePipeline.create(
                ik, tracking, trackingInts, pipeline, camera)) {
            nativePipeline.update(detections, 1.0, out);
            int errorCode = nativePipeline.update(detections, 1.1, out);
            if (errorCode != 0) {
                throw new AssertionError("expected error code 0, got " + errorCode);
            }
            if (!isFinite(out[1]) || !isFinite(out[2]) || !isFinite(out[3])) {
                throw new AssertionError("IK returned a non-finite angle");
            }

            nativePipeline.setTrueRadius(0.1);
            errorCode = nativePipeline.update(detections, 1.2, out);
            if (errorCode != 0) {
                throw new AssertionError(
                        "expected error code 0 after radius change, got " + errorCode);
            }
        }

        System.out.println(
                "JNI smoke test passed: "
                        + out[0] + " " + out[1] + " " + out[2] + " " + out[3]);
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
