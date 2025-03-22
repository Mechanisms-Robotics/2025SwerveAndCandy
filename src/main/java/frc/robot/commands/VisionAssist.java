package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * NEXT STEPS
 * 
 *   Add buttons to trigger the command
 *   Create Walton test plan below when it's ready for that
 *   Through out obviously bad results (see comment below)
 *     Maybe comment this out at first then introduce it later
 * 
 * ON-ROBOT TEST PLAN FOR TMS
 * 
 *   Figure out why translational driving is broken
 * 
 *   Test lateral outputs and calibrate P_LATERAL
 * 
 *   Drive it to find bugs
 * 
 *   Enjoy a stiff drink of strong chocolate milk
 * 
 * ON-ROBOT TEST PLAN FOR WALTON
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
        //
        // [fox] if we are going to assume that driver parks against reef, we could
        //  update L4 command to bump swerve backwards before extending elevator
        LEFT, RIGHT;
    }

    private static final String CAMERA_NAME = "LimeLight1"; // front camera
    private static final PhotonCamera realCamera = new PhotonCamera(CAMERA_NAME);

    // see CameraWrapper for discussion
    private CameraWrapper wrappedCamera = new CameraWrapper(realCamera);

    private static final double LEFT_OFFSET = -0.42; // meters
    private static final double RIGHT_OFFSET = -0.05; // meters
    
    /**
     * These are the overrides for VisionAssist as a Command. The idea is that
     * the driver holds down a button while driving.
     */
    
    private final RobotContainer robotContainer;
    private final ScoringPosition currentScoringPosition;

    public VisionAssist(
        RobotContainer robotContainer, ScoringPosition scoringPosition)
    {
        this.robotContainer = robotContainer;
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

        if (scoringPose != null) {
            SmartDashboard.putNumber("Vision Assist/Scoring Pose/X (m)",
                scoringPose.getTranslation().getX());
            SmartDashboard.putNumber("Vision Assist/Scoring Pose/Y (m)",
                scoringPose.getTranslation().getY());
            SmartDashboard.putNumber("Vision Assist/Scoring Pose/Rotation (rad)",
                scoringPose.getRotation().getRadians());
            SmartDashboard.putNumber("Vision Assist/Scoring Pose/Rotation (deg)",
                scoringPose.getRotation().getDegrees());
        }

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
         * This puts the robot in driver override mode if it isn't already.
         * The override ends when the command ends (below).
         */

        robotContainer.overrideDriver(
            outputs.outputRotation, outputs.outputX);
    }

    @Override
    public void end(boolean interrupted) {
        /**
         * This absolutely has to happen or we are done for the match.
         */

        robotContainer.cancelOverride();
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

        if (cameraToTarget == null)
            return null; // no good target

        /**
         * As it turns out, PhotonVision uses a coordinate system that
         * I discovered we have to transform. Here's where that's done. 
         */

        cameraToTarget = new Transform3d(
            new Translation3d(
                -cameraToTarget.getY(),
                cameraToTarget.getX(),
                0.0),
            cameraToTarget.getRotation().plus(
                new Rotation3d(0.0, 0.0, Math.PI)
            )
        );

        SmartDashboard.putNumber("Vision Assist/Camera To Target/X",
            cameraToTarget.getX());
        SmartDashboard.putNumber("Vision Assist/Camera To Target/Y",
            cameraToTarget.getY());
        SmartDashboard.putNumber("Vision Assist/Camera To Target/Rotation (rad)",
            cameraToTarget.getRotation().toRotation2d().getRadians());

        /**
         * The earth is flat, so we project the transform onto the plane and
         * then we use math to find the relative scoring position. BUT as it
         * turns out math is hard and the change in scoring position based on
         * the rotation of the target is trivial and that error should go to
         * zero as we drive, so I'm throwing out that correction for now.u
         */
        
        double x = cameraToTarget.getX();
        double angle = cameraToTarget.getRotation().toRotation2d().getRadians();

        double offset = scoringPosition == ScoringPosition.RIGHT ? RIGHT_OFFSET : LEFT_OFFSET;
        x += offset; //*Math.cos(angle);

        return new Pose2d(x, 0.0, new Rotation2d(angle));
    }

    private class TargetError {
        // positive error means we are left of target line
        public double lateralError = 0.0; // meters

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
    private static final double P_ROTATION = 0.4;

    private VisionOutputs getOutputs(TargetError error) {
        VisionOutputs outputs = new VisionOutputs();

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
                 * numbers are to simulate noise.
                 * 
                 * NOTE: I don't think these are actually how the camera outputs
                 * so don't believe them. The camera coordinate system is different!
                 * See comments above about how to change thes to mimic the real
                 * camera.
                 */
                Transform3d xform = new Transform3d(
                    new Translation3d(0.5, 3.0, 0.4),
                    new Rotation3d(0.03, -0.03, 0.1)
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

             /** [fox] i think i read somewhere that the max results queue in photon is 20.
              *  But that was probably stupid late at night, so don't quote me. After digesting
              *  your average, I'd vote for KISS. Just analyze the most recent result from the
              *  PhotonPipeline as you've already coded below.
              *  [odom] I read 20 as well.
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

             /* [fox] i was thinking simpler, but along the lines of the state machine that
              * you've seen me asking Nolan to start on. 
              *     -- If not yet in vision-assist mode, then the
              *        bestTarget is the one we want.
              *     -- If already in vision-assist, then we compare bestTarget to
              *        lastKnown bestTarget. If they are different, enter 0 in averager.
              *
              * [odom] Something like that. My thought is to solve for target noise
              * if we discover that we have a problem with it. It may be unnecessary.
              * Keep in mind that I don't envision this running constantly, only when
              * the vision assist button is held, so the averager would start with all
              * zeros and populate over the course of about half a second, leading to an
              * initial smooth start.
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
