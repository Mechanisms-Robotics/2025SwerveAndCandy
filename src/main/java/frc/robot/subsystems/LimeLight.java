package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LimeLight extends SubsystemBase {
    private final PhotonCamera camera;
    private final String cameraName;
    private final Transform3d cameraToRobot;

    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonPoseEstimator poseEstimator;
    private Pose3d visionPose;
    private final ArrayList<ApriltagData> aprilTagDatas = new ArrayList<>();

    private final StructPublisher<Pose3d> visionLocalisationPublisher;
    

    private final SwerveSubsystem swerve;


    public LimeLight(String cameraName, Transform3d cameraToRobot, SwerveSubsystem swerve) {
        camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        this.cameraToRobot = cameraToRobot;
        this.swerve = swerve;

        aprilTagFieldLayout = AprilTagFieldLayout.loadField(Constants.Vision.FIELD);
        aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

        poseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            this.cameraToRobot
        );

        for (int i = 0; i < 22; i++) {
            aprilTagDatas.add(new ApriltagData(i+1));
        }

        // I am not using SmartDashboard.putData and am using NetworkTableInstance because I can give advantage scope a pose3d
        // if you want to see this pose3d, open up the 3D Field and drag this value into Poses
        visionLocalisationPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard")
            .getStructTopic(cameraName + "/Vision Localisation Pose3d", Pose3d.struct).publish();
    }

    public class ApriltagData {
        private final int id;
        private double yaw = 0.0;
        private double pitch = 0.0;
        private double area = 0.0;
        private Pose3d robotPoseBasedOnApriltag;
        public final Pose3d positionOnField;
        
        private boolean detected = false;
        private boolean updatedOnCycle = false;
        
        // true if the april tag is being used for robot position measurements
        private boolean goodForPositionMeasurements = false;
        private static boolean hasPose = false;
        private String tableName;

        private final StructPublisher<Pose3d> visionLocalisationPublisher;


        public ApriltagData(int id) {
            this.id = id;
            for (int tag : Constants.FieldConstants.GOOD_APRIL_TAGS) {
                if (id == tag) {
                    goodForPositionMeasurements = true;
                    break;
                }
            }
            positionOnField = aprilTagFieldLayout.getTagPose(id).get();

            tableName = cameraName + "/Apriltags/" + id + "/";

            if (goodForPositionMeasurements) {
                SmartDashboard.putString(tableName + "info", "this apriltag is used for robot position field localisation");
            } else {
                SmartDashboard.putString(tableName + "info", "this apriltag is not used for position localisation");
            }
            visionLocalisationPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard")
                .getStructTopic(tableName + "Vision Localisation Pose3d", Pose3d.struct).publish();

        }

        /**
         * 
         * @param yaw 
         * @param pitch
         * @param area
         * @param detected
         */
        public void update(double yaw, double pitch, double area, boolean detected, Pose3d robotPoseBasedOnApriltag) {
            this.yaw = yaw;
            this.pitch = pitch;
            this.area = area;
            this.detected = detected;
            updatedOnCycle = true;
            this.robotPoseBasedOnApriltag = robotPoseBasedOnApriltag;
            robotPoseBasedOnApriltag = aprilTagFieldLayout.getTagPose(id).get();


            if (detected && goodForPositionMeasurements) {
                hasPose = true;
            }
            
            SmartDashboard.putNumber(tableName + "Yaw", yaw);
            SmartDashboard.putNumber(tableName + "Pitch", pitch);
            SmartDashboard.putNumber(tableName + "Area", area);
            SmartDashboard.putBoolean(tableName + "Detected", detected);
            visionLocalisationPublisher.set(robotPoseBasedOnApriltag);
        }

        /**
         * Clears the data when not updated on the cycle, if it was updated it does litteraly nothing
         */
        public void resetData() {
            if (hasPose) {
                hasPose = false;
            }
            if (!updatedOnCycle) {
                update(0.0, 0.0, 0.0, false, getRobotBasedOnTag());
            }
            updatedOnCycle = false;
        }

        public boolean getDetected() {
            return detected;
        }
        public int getId() {
            return id;
        }
        public double getYaw() {
            return yaw;
        }

        public double getPitch() {
            return pitch;
        }

        public double getArea() {
            return area;
        }

        public Pose3d getRobotBasedOnTag() {
            return robotPoseBasedOnApriltag;
        }

        public Transform2d getRobotApriltagTransform() {
            return new Transform2d(
                // converting the x and y courdanites to robot relative
                robotPoseBasedOnApriltag.getX() - positionOnField.getX(),
                robotPoseBasedOnApriltag.getY() - positionOnField.getY(),
                // the angle the apriltag is pointing flipped around (plus PI radians) is facing the apriltag
                Rotation2d.fromRadians(positionOnField.getRotation().getZ() + Math.PI)
            );
        }

        public Transform2d getRobotApriltagTransformOffset(double xOffset) {
            // the yaw of the angle is the 3d rotations angle around the z coardanite
            double yaw = positionOnField.getRotation().getZ();
            // Since th desired x offset is apriltag relative and the one I need is robot relative, it needs to be converted
            double xFieldRelativeOffset = xOffset * Math.cos(yaw);
            double yFieldRelativeOffset = xOffset * Math.sin(yaw);

            return new Transform2d(
                // converting the x and y courdanites to robot relative, and then offseting it
                robotPoseBasedOnApriltag.getX() - positionOnField.getX() + xFieldRelativeOffset,
                robotPoseBasedOnApriltag.getY() - positionOnField.getY() + yFieldRelativeOffset,
                // the angle the apriltag is pointing flipped around (plus PI radians) is facing the apriltag
                Rotation2d.fromRadians(positionOnField.getRotation().getZ() + Math.PI)
            );        }


        /**
         * @return true if an apriltag has taken a valid position measurement
         */
        public static boolean validPositionMeasurement() {
            return hasPose;
        }
    }

    /**
     * Get the april tag data of the given id.
     * Note, this will get the id-1 element of the list because Apriltag 1 is the 0th element.
     * 
     * @param id of the apriltag
     * @return april tag data object
     */
    public ApriltagData getApriltag(int id) {
        return aprilTagDatas.get(id-1);
    }

    /**
     * Get the closest apriltag data (calculated by largest area taken up on the camera)
     * 
     * @param ids int list of the ids to look for
     * @return apriltag data if it finds one, otherwise Optional.empty()
     */
    public Optional<ApriltagData> getClosestAprilTag(int[] ids) {
        double largestArea = 0.0;
        int largestAreaID = 0;
        for (int i = 0; i < ids.length; i++) {
            ApriltagData tag = getApriltag(ids[i]);
            if (tag.getArea() > largestArea) {
                largestArea = tag.area;
                largestAreaID = tag.id;
            }
        }
        if (largestAreaID != 0) {
            return Optional.of(getApriltag(largestAreaID));
        }
            
        return Optional.empty();
    }

    /**
     * Get the closest apriltag data (calculated by largest area taken up on the camera).
     * Acounts for the side of the field
     * 
     * @param ids int list of the ids to look for
     * @return apriltag data if it finds one, otherwise Optional.empty()
     */
    public Optional<ApriltagData> getClosestReefApriltag() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            return getClosestAprilTag(Constants.FieldConstants.BLUE_REEF_APRIL_TAGS);
        return getClosestAprilTag(Constants.FieldConstants.RED_REEF_APRIL_TAGS);
    }

    @Override
    public void periodic() {
        var results = camera.getAllUnreadResults();
        SmartDashboard.putBoolean(cameraName + "/connected", camera.isConnected());
        SmartDashboard.putNumber(cameraName + "/Target Number", results.size());

        for (PhotonPipelineResult result : results) {
            SmartDashboard.putBoolean(cameraName + "/Apriltag detected", result.hasTargets());
            if (result.hasTargets()) {
                SmartDashboard.putBoolean(cameraName + "/Result Empty", result.getMultiTagResult().isEmpty());
                // Loop through all of the apriltag data
                for (PhotonTrackedTarget target : result.targets) {
                    ApriltagData tagData = getApriltag(target.getFiducialId());
                
                    // now we know the april tag was detected, updated all of the values of that tag id
                    tagData.update(target.getYaw(), target.getPitch(), target.getArea(), true, 
                    PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), tagData.positionOnField, cameraToRobot));
                }
            }
            // only updates the position if one of april tags was detected that is supposed to be used for vision
            if (ApriltagData.validPositionMeasurement()) {
                Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
                if (estimatedPose.isPresent()) {
                    visionPose = estimatedPose.get().estimatedPose;
                    visionLocalisationPublisher.set(visionPose);
                    swerve.addVisionMeasurement(visionPose.toPose2d(), estimatedPose.get().timestampSeconds);

                    SmartDashboard.putString(cameraName + "/Estimated Robot Position Components", visionPose.toString());
                }
            }
        }

        // if the apriltag was not updated, clear its data because otherwise the data from the last time it was detected stays
        // also updated has pose to make if false, i.e. assume false until proven, for the next cycle
        // * note please keep this as the final line if possible, moving it up it a breading ground for bugs
        for (int i = 0; i < aprilTagDatas.size(); i++) {
            aprilTagDatas.get(i).resetData();
        }
    }
}
