package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * TODO
 * 
 */

public class VisionAssist {

    public enum ScoringPosition { // Could expand for L4, etc...
        LEFT, RIGHT;
    }

    private static final String CAMERA_NAME = "LimeLight1"; // this is the front camera
    private static final PhotonCamera CAMERA = new PhotonCamera(CAMERA_NAME);

    /**
     * We can simply transform laterly (relative to the target) unless we ever want
     * to stop short of the target or something. We could use the 3D parts of the
     * transform if we ever install the jetpack.
     */

    private static final double LEFT_OFFSET = -0.75; // meters? TODO verify and estimate
    private static final Transform3d LEFT_SCORING_TRANSFORM = new Transform3d(
        new Translation3d(LEFT_OFFSET, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, 0.0)
    );

    private static final double RIGHT_OFFSET = 0.75; // meters? TODO verify and estimate
    private static final Transform3d RIGHT_SCORING_TRANSFORM = new Transform3d(
        new Translation3d(RIGHT_OFFSET, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, 0.0)
    );

    /**
     * TODO document and complete
     */

    private Pose2d getScoringPose(ScoringPosition scoringPosition) {

        /**
         * First we call PhotonVision to get the pose of the best AprilTag
         * as a Transform3d from the camera.
         * 
         * TODO: Check the camera settings to see if it is sending offsets
         * for where and how it's mounted.
         */

        List<PhotonPipelineResult> results = CAMERA.getAllUnreadResults();

        if (results.isEmpty()) {
            return null; // no good target pose
        }

        // Is there a reason to use earlier results?
        PhotonPipelineResult result = results.get(results.size() - 1);

        PhotonTrackedTarget target = result.getBestTarget();

        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        // what is getAlternateCameraToTarget() for??

        // TODO throw out obviously bad results here by returning null?

        /**
         * Now we calculate our scoring pose, which is a constant offset from
         * the AprilTag RELATIVE TO THE APRILTAG. plus() transforms a transform
         * relative to the orientation of the transform.
         */

        Transform3d scoringTransform = bestCameraToTarget.plus(
            scoringPosition == ScoringPosition.LEFT ? LEFT_SCORING_TRANSFORM : RIGHT_SCORING_TRANSFORM
        );
        
        Translation3d translation3d = scoringTransform.getTranslation();
        Rotation3d rotation3d = scoringTransform.getRotation();

        Translation2d translation2d = new Translation2d(translation3d.getX(), translation3d.getY());
        Rotation2d rotation2d = new Rotation2d(rotation3d.getAngle());
        Pose2d scoringPose = new Pose2d(translation2d, rotation2d);

        // TODO: output to shuffleboard here and see if the outputs are rational in simulation

        return scoringPose;
    }

    private class TargetError {
        // positive error means we are left of target line
        public double lateralError = 0.0;

        // positive error means we need to rotate clockwise
        public double rotationError = 0.0; // radians
    }

    /**
     * TODO: document
     */
    
    private class ErrorList {
        private static final int SLIDING_WIDOW_SIZE = 15;

        private final TargetError[] errorMeasurements
            = new TargetError[SLIDING_WIDOW_SIZE];

        public ErrorList() {
            this.reset();
        }

        private void reset() { // set all to zero
            for (int i = 0; i < SLIDING_WIDOW_SIZE; i++) {
                errorMeasurements[i] = new TargetError();
            }
        }

        private void addMeasurement(TargetError newError) {
            for (int i = SLIDING_WIDOW_SIZE - 2; i >= 0; i--) {
                // yeah, this is cringeworthy, but it's fine for short lists
                errorMeasurements[i + 1] = errorMeasurements[i];
            }

            errorMeasurements[0] = newError;
        }

        private TargetError calculateAverage() {
            // returns a WEIGHTED average

            double totalWeight = 0; // I know Gauss figured this out, but hey...
            TargetError accumulator = new TargetError();

            for (int i = 0; i < SLIDING_WIDOW_SIZE; i++) {
                double weight = SLIDING_WIDOW_SIZE - i;
                accumulator.lateralError += weight*errorMeasurements[i].lateralError;
                accumulator.rotationError += weight*errorMeasurements[i].rotationError;
                totalWeight += weight;
            }

            accumulator.lateralError /= totalWeight;
            accumulator.rotationError /= totalWeight;

            return accumulator; // TODO: unit test
        }
    }

    private final ErrorList ERROR_LIST = new ErrorList();

    private TargetError calculateError(Pose2d targetPose) {
        /**
         * Determine lateralError and rotationError. Note that we add zeros
         * into the measurement if the targetPose is null (poor or invalid).
         * This causes the error to smoothly go to zero if we loose the target.
         */

        TargetError error = new TargetError();

        if (targetPose != null) {
            error.lateralError = targetPose.getX(); // TODO think about this more
            error.rotationError = targetPose.getRotation().getRadians(); // TODO think about this more
        }

        /**
         * Add the new error reading to the front of the list and drop the
         * back of the list the return the weighted average.
         */

        ERROR_LIST.addMeasurement(error);
        return ERROR_LIST.calculateAverage();
    }

    private class VisionOutputs {
        public double outputX = 0.0;
        public double outputRotation = 0.0;
    }

    /**
     * TODO: Look into if these need to be transformed for field-relative
     * drive or if we can send them to the drivetrain robot relative (hopefully).
     * 
     * Document better. Think through values. Output and test in sim.
     */

    private static final double P_LATERAL = 1.0;
    private static final double P_ROTATION = 1.0;

    private VisionOutputs getOutputs(TargetError error) {
        VisionOutputs outputs = new VisionOutputs();
        outputs.outputX = Math.max(-1.0, Math.min(P_LATERAL*error.lateralError, 1.0));
        outputs.outputRotation = Math.max(-1.0, Math.min(P_ROTATION*error.rotationError, 1.0));
        return outputs;
    }
}
