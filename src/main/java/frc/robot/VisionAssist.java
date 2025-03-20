package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.Struct;
import frc.robot.VisionAssist.ScoringPosition;

/**
 * TODO
 * And we probably need to make this a command.
 */

public class VisionAssist {

    public enum ScoringPosition { // Could expand for L4, etc...
        LEFT, RIGHT;
    }

    private static final String CAMERA_NAME = "photonvision"; // TODO confirm
    private static final PhotonCamera CAMERA = new PhotonCamera(CAMERA_NAME);

    private static final Transform2d LEFT_SCORING_TRANSFORM = null;
    private static final Transform2d RIGHT_SCORING_TRANSFORM = null;

    /**
     * TODO document and complete
     */

    private Pose2d getTargetPose(ScoringPosition scoringPosition) {

        /**
         * First we call PhotonVision to get the pose of the best AprilTag
         * as a Transform3d from the camera.
         * 
         * TODO: Check the camera settings to see if it is sending offsets
         * for where and how it's mounted.
         */

        var result = CAMERA.getLatestResult();

        if (!result.hasTargets()) {
            return null; // no good target pose
        }

        PhotonTrackedTarget target = result.getBestTarget();

        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        // what is getAlternateCameraToTarget() for??

        // TODO throw out obviously bad results here by returning null?

        /**
         * Now we calculate our target pose, which is a constant offset from
         * the AprilTag RELATIVE TO THE APRILTAG. This is the robot's scoring
         * position pose.
         */

        Pose2d scoringPose = null;

        if (scoringPosition == ScoringPosition.LEFT) {

        }
        else if (scoringPosition == ScoringPosition.RIGHT) {

        }

        return scoringPose;
    }

    private class TargetError {
        // positive error means we are left of target line
        public double lateralError = 0.0;

        // positive error means we need to rotate clockwise
        public double rotationError = 0.0;
    }

    /**
     * TODO: document
     */

    private TargetError calculateError(Pose2d targetPose) {
        /**
         * Determine lateralError and rotationError
         * 
         */

        /**
         * Add the new error reading to the front of the list and drop the
         * back of the list.
         */

        /**
         * Return the weighted average.
         */
    }

    private class VisionOutputs {
        double outputX = 0.0;
        double outputRotation = 0.0;
    }

    /**
     * TODO: Look into if these need to be transformed for field-relative
     * drive or if we can send them to the drivetrain robot relative.
     */

    private VisionOutputs getOutputs(TargetError error) {
        return null;
    }
}