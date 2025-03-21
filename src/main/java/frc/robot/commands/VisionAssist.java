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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * NEXT STEPS
 * 
 *   Figure out how to inject the outputs and put that in the code
 *   Run it in simulation and document it
 *   Through out obviously bad results (see comment below)
 *     Maybe comment this out at first then introduce it later
 *   Finish test plan for TMS and test at TMS
 *   Create Walton test plan below when it's ready for that
 * 
 * SIMULATION TESTS
 * 
 *   Make sure that scoring pose is what I think it is (right and cc'wise positive)
 *     I ran this simulation and the rotation, X, and Y all make sense.
 *   Make sure that the error output is rational.
 *     Yes, the laterl and rotational errors are rational for the test numbers.
 *   Make sure that the control outputs are rational.
 *     Yes, they are as expected.
 * 
 * ON-ROBOT TEST PLAN FOR TMS
 * 
 *   Validate that the AprilTags show positive rotation when rotated cc'wise
 *     and positive lateral offset when moved to the right (from camera POV)
 *     and that the numbers seem rational (meters and radians, etc.). Do this
 *     with the robot enabled in test mode so I get outputs to the dashboard.
 *   Calibrate P_LATERAL and P_ROTATIONAL
 *   Test LEFT_OFFSET and RIGHT_OFFSET and adjust to approximate better
 * 
 * ON-ROBOT TEST PLAN FOR WALTON
 * 
 * 
 * NOTES ON CHANGING THE DRIVE
 * 
 * In RobotContainer.java, we create driveAngularVelocity, which is a
 * SwerveInputStream. It takes inputs from the driver controller. We then create
 * Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity
 * and set that command to the drive base's default command. That eventually
 * uses driveFieldOriented, but calls the implementation on SwerveDrive, not
 * our SwerveDriveSubsystem.
 * 
 * NOTE TO JOEL: SEE UPDATES TO RobotContianer. This works, but unfortunately I'll
 * need a safe way of applying this to modify the driver inputs because it will
 * indeed need to be field relative.
 * 
 * 
 */

/**
 * The VisionAssist command helps the driver to line up on the scoring position
 * by aligning the robot with the scoring position laterally and rotationally.
 * When activated, it uses a sliding averager to smooth the error between the
 * robot's vision-determined pose and the desired pose (ignoring distance to the
 * scoring position as the driver will control that). Here is the flow:
 * 
 * 1. The driver activates vision assistance.
 * 2. The averager sets all of its error values to zero, meaning for the first
 *    few iterations of the periodic loop there will be no outputs.
 * 3. Every iteration of the periodic loop, the following happens:
 *      - We get the pose of the AprilTag in front of us and use a transform
 *        to get the scoring pose (remember we only really care about the
 *        lateral and rotational part of the pose). This pose is relative to the
 *        camera, not field relative.
 *      - From the scoring pose, we calculate the error in rotation and in
 *        lateral translation. This is simply the difference between the
 *        current state and the desired state. This error is put into the
 *        averager and begins to pull the average error toward what we hope is
 *        the actual error.
 *      - From the error, we use PID to determine the magnitude of output to the
 *        drivetrain and send that output to the drivetrain.
 * 
 * The end effect is that the robot should smoothly rotate and translate onto
 * the "scoring line" as the driver drives towards the reef.
 */

public class VisionAssist extends Command {

    public enum ScoringPosition {
        // We could expand this for L4, etc. but that may require us to
        // consider the distance from the pose, not just lateral and rotational.
        LEFT, RIGHT;
    }

    private static final String CAMERA_NAME = "LimeLight1"; // front camera
    private static final PhotonCamera realCamera = new PhotonCamera(CAMERA_NAME);

    // see CameraWrapper for discussion
    private CameraWrapper wrappedCamera = new CameraWrapper(realCamera);

    /**
     * We can simply transform laterly (relative to the target) unless we ever want
     * to stop short of the target or something. We could use the 3D parts of the
     * transform if we ever install the jetpack.
     * 
     * These transforms will be added to the AprilTag's pose (which is relative
     * to the robot). So the transformation already takes into account a rotational
     * component of the AprilTag relative to the robot.
     */

    private static final double LEFT_OFFSET = -0.75; // meters
    private static final Transform3d LEFT_SCORING_TRANSFORM = new Transform3d(
        new Translation3d(LEFT_OFFSET, 0.0, 0.0), // merely left
        new Rotation3d(0.0, 0.0, 0.0) // same orientation as the AprilTag
    );

    private static final double RIGHT_OFFSET = 0.75; // meters
    private static final Transform3d RIGHT_SCORING_TRANSFORM = new Transform3d(
        new Translation3d(RIGHT_OFFSET, 0.0, 0.0), // merely right
        new Rotation3d(0.0, 0.0, 0.0)  // same orientation as the AprilTag
    );

    /**
     * These are the overrides for VisionAssist as a Command. The idea is that
     * the driver holds down a button while driving.
     */

    private final ScoringPosition currentScoringPosition;

    public VisionAssist(ScoringPosition scoringPosition) {
        this.currentScoringPosition = scoringPosition;
    }

    @Override
    public void initialize() {
        averager.reset(); // zeros all error so that we start without jerk
    }

    @Override
    public void execute() {
        /**
         * This is the main execution loop for the algorithm.
         */

        Pose2d scoringPose = getScoringPose(currentScoringPosition);

        SmartDashboard.putNumber("Vision Assist/Scoring Pose/X (m)",
            scoringPose.getTranslation().getX());
        SmartDashboard.putNumber("Vision Assist/Scoring Pose/Y (m)",
            scoringPose.getTranslation().getY());
        SmartDashboard.putNumber("Vision Assist/Scoring Pose/Rotation (rad)",
            scoringPose.getRotation().getRadians());
        SmartDashboard.putNumber("Vision Assist/Scoring Pose/Rotation (deg)",
            scoringPose.getRotation().getDegrees());

        TargetError error = calculateError(scoringPose);

        SmartDashboard.putNumber("Vision Assist/Error/Lateral",
            error.lateralError);
        SmartDashboard.putNumber("Vision Assist/Error/Rotational (rad)",
            error.rotationError);

        VisionOutputs outputs = getOutputs(error);

        SmartDashboard.putNumber("Vision Assist/Outputs/Lateral",
            outputs.outputX);
        SmartDashboard.putNumber("Vision Assist/Outputs/Rotational",
            outputs.outputRotation);

        /**
         * We inject the outputs here. Hopefully we don't have to transform
         * them to the field coordinate system.
         */

        //m_swerve.drive(new ChassisSpeeds(speed, 0, 0));
    }

    /**
     * This gives us our scoring pose, relative to the robot. Remember that
     * we don't really care about how far the pose is in front of the robot
     * since we only want to control side-to-side and rotational motion. So
     * the returned Pose2d is relative to the robot.
     */

    private Pose2d getScoringPose(ScoringPosition scoringPosition) {
        /**
         * Use the camera to get the transform from the robot
         * to the AprilTag.
         */

        Transform3d cameraToTarget = wrappedCamera.getCameraToTarget();

        /**
         * Now we calculate our scoring pose, which is a constant offset from
         * the AprilTag RELATIVE TO THE APRILTAG. plus() transforms a transform
         * relative to the orientation of the transform.
         */

        Transform3d scoringTransform = cameraToTarget.plus(
            scoringPosition == ScoringPosition.LEFT
            ? LEFT_SCORING_TRANSFORM : RIGHT_SCORING_TRANSFORM
        );

        /**
         * The earth is flat, so we rebuild the transform as a Pose2d.
         */
        
        Translation3d translation3d = scoringTransform.getTranslation();
        Rotation3d rotation3d = scoringTransform.getRotation();

        Translation2d translation2d
            = new Translation2d(translation3d.getX(), translation3d.getY());
        Rotation2d rotation2d = new Rotation2d(rotation3d.getAngle());
        Pose2d scoringPose = new Pose2d(translation2d, rotation2d);

        return scoringPose;
    }

    private class TargetError {
        // positive error means we are left of target line
        public double lateralError = 0.0;

        // positive error means we need to rotate clockwise
        public double rotationError = 0.0; // radians
    }

    /**
     * ErrorAverager is the averager. It performs a weighted average of the last
     * so many readings, weighting the recent readings heaviest. The way it
     * should work is you throw in a zero error if the reading is bad, which
     * will cause the controller that uses it to start to try less hard. At
     * some point, with enough zero readings, the controller will simply
     * "stay the course." The averager also smooths the control outputs.
     */
    
    private class ErrorAverager {
        private static final int SLIDING_WIDOW_SIZE = 15;

        private final TargetError[] errorMeasurements
            = new TargetError[SLIDING_WIDOW_SIZE];

        public ErrorAverager() {
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

            return accumulator;
        }
    }

    private final ErrorAverager averager = new ErrorAverager();

    private TargetError calculateError(Pose2d targetPose) {
        /**
         * Determine lateralError and rotationError. Note that we add zeros
         * into the measurement if the targetPose is null (poor or invalid).
         * This causes the error to smoothly go to zero if we loose the target.
         * 
         * Notice that if the target pose X offset is positive, the error is
         * negative because we are left of the target pose. If the target
         * rotation is positive, our error is negative because we want to
         * rotate counter clockwise (as viewed from above).
         */

        TargetError error = new TargetError();
        if (targetPose != null) {
            error.lateralError = -targetPose.getX();
            error.rotationError = -targetPose.getRotation().getRadians();
        }
        averager.addMeasurement(error); // never null, but may be zeros
        return averager.calculateAverage();
    }

    /**
     * The vision outputs are between -1.0 and 1.0, inclusive. I'm thinking
     * of this like joystick controller inputs.
     */

    private class VisionOutputs {
        public double outputX = 0.0;
        public double outputRotation = 0.0;
    }
    /**
     * This is a place where proportional control only should be fine, so
     * we just multiply by P and clamp the outputs to be between -1.0 and 1.0.
     */

    // Think of this as the desired control output if we're one meter off
    private static final double P_LATERAL = 0.25;

    // Think of this as the desired control output if we're one radian (57 degrees) off
    private static final double P_ROTATION = 0.25;

    private VisionOutputs getOutputs(TargetError error) {
        VisionOutputs outputs = new VisionOutputs();
        // [fox] why is this max and min needed? [joel] answering in comments above.
        outputs.outputX = Math.max(-1.0, Math.min(-P_LATERAL*error.lateralError, 1.0));
        outputs.outputRotation = Math.max(-1.0, Math.min(-P_ROTATION*error.rotationError, 1.0));
        return outputs;
    }

    /**
     * If I were a good person, I'd write real unit tests, but this series of tests
     * should at least help. These are meant to be run in simulation in test mode.
     * 
     * See Robot.java testInit().
     */

    public void startTestMode() {
        /**
         * To test getScoringPose we create a fake camera and then see if the
         * dashboard outputs are rational.
         */

        wrappedCamera = new CameraWrapper(); // creates a fake camera
    }

    /**
     * The camera wrapper is a class that wraps our camera. Its purpose is to allow
     * us to run tests against a "fake" camera.
     */

    private class CameraWrapper {
        private final PhotonCamera camera;

        public CameraWrapper() {
            this.camera = null; // fake camera for testing
        }

        public CameraWrapper(PhotonCamera camera) {
            this.camera = camera;
        }

        public Transform3d getCameraToTarget() {
            /**
             * If camera is null, we consider this a test scenario. The
             * magic numbers below are just for testing.
             */

            if (camera == null) {
                /**
                 * This is the simulated camera to target. Some of the "small"
                 * numbers are to simulate noise
                 */
                Transform3d xform = new Transform3d(
                    new Translation3d(0.5, 3.0, 0.4), // 3m in front and 0.5m right
                    new Rotation3d(0.03, -0.03, 0.7) // 0.7 radians (cc'wise)
                );
                return xform;
            }

            /**
             * First we call PhotonVision to get the pose of the best AprilTag
             * as a Transform3d from the camera.
             */

            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            if (results.isEmpty()) {
                return null; // no good target pose
            }

            // Is there a reason to use earlier results?
            // [fox] i could come up with scenarios, but i don't know how likely they are.
            //       for exmample, let's say we had already locked on a target in a prior
            //       loop iteration. and in this iteration, we get 20 new snapshots from photon
            //       to analyze. but in the last 3 snapshots, something happened and we don't see
            //       the target now. would we want to operate off our lastKnown snapshot from
            //       the prior loop? or would we want to get the latest info possible from these
            //       results before we lost view of the target?
            //  Maybe an alternative question would be: do we want to bail out as soon as
            //  we determine our most recent photonResult doesn't have the target in view? Or
            //  do we want to have some tolerance for such mishaps with a number of retry attempts?
            //  KISS principle would say just use last and hope for the best

            /**
             * [joel] That's fair. Part of the behavior of the averager is that if we lose sight
             * of the target for a brief time, the average error goes down a bit, meaning the
             * tendency of the robot is to stabilize at holding its course. So if the target is
             * noisy (maybe every third reading is lousy) it effectively tries less hard but
             * the good readings keep it geneally moving in the right direction.  If it
             * loses the target altogether and permanently (maybe the camera is blocked
             * when we're really close to the scoring position), the effect is the robot
             * will stop trying to rotate or slew, which is the desired behavior when
             * it's on the line and really close.
             * 
             * I guess we could average all unread results here, too, and use that
             * as the current result. If there is a timestamp on the result we could
             * only care about those younger than the periodic loop time, 20 ms.
             */
            
            PhotonPipelineResult result = results.get(results.size() - 1);

            if (!result.hasTargets()) {
                return null; // no target
            }

            // [fox] are we comfortable assuming that the bestTarget is the right one we want?
            //       do we need to check if it is different than the lastKnown bestTarget?
            //       for example: MetalMountain knocks us off course and even though we still
            //       see our target that we are locked on, it is no longer the BESTtarget at
            //       the current time. Do we want to change our goal to the newBestTarget?
            /**
             * [joel] That's a good thought. I'd say that this is one way we could
             * discriminate between a good reading and a bad reading, by the AprilTag
             * id. Maybe we store a list of the last so many target ids, regardless
             * of if good or bad and we assume the most frequent target id is what
             * we care about.
             */

            PhotonTrackedTarget target = result.getBestTarget();

            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            // what is getAlternateCameraToTarget() for??
            // [fox] I guess you could have one camera on each of the corners of the bot and improve
            //       accuracy / smooth error correction ??

            /**
             * This is where we could (should?) throw out obviously bad results.
             * See discussion above for some ideas. We should at least throw out
             * targets outside of, say a 30-degree cone extending 5 meters in front
             * of the robot. And maybe ids we know are not reef scoring positions.
             */

             return bestCameraToTarget;
        }
    }
}
