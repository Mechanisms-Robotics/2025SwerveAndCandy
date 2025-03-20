package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * TODO
 */

public class VisionAssist {

    /**
     * TODO
     */

    private Object getTargetPose(leftOrRightEnum) {
        /**
         * First we call PhotonVision to get the pose of the best AprilTag.
         */

        var result = camera.getLatestResult();

        if (!result.hasTargets()) {
            return null;
        }

        PhotonTrackedTarget target = result.getBestTarget();

        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        // what is getAlternateCameraToTarget() for??

        // TODO throw out obviously bad results here by return null?

        /**
         * Now we calculate our target pose, which is a constant offset from
         * the AprilTag RELATIVE TO THE APRILTAG.
         */

        
    }

}