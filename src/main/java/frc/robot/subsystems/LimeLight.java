package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase {
    private final PhotonCamera camera;
    private final String cameraName;
    private final Transform3d cameraToRobot;

    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonPoseEstimator poseEstimator;
    private Pose3d visionPose;
    private final ArrayList<ApriltagData> aprilTagDatas = new ArrayList<>();


    public LimeLight(String cameraName, Transform3d cameraToRobot) {
        camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        this.cameraToRobot = cameraToRobot;

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
    }

    public class ApriltagData {
        private final int id;
        private double yaw = 0.0;
        private double pitch = 0.0;
        private double area = 0.0;
        
        private boolean detected = false;
        private boolean updatedOnCycle = false;
        
        // true if the april tag is being used for robot position measurements
        private boolean goodForPositionMeasurements = false;
        private static boolean hasPose = false;
        private String tableName;

        public ApriltagData(int id) {
            this.id = id;
            for (int tag : Constants.FieldConstants.GOOD_APRIL_TAGS) {
                if (id == tag) {
                    goodForPositionMeasurements = true;
                    break;
                }
            }

            tableName = cameraName + "/AprilTags/" + id + "/";

            if (goodForPositionMeasurements) {
                SmartDashboard.putString(tableName + "info", "this apriltag is used for robot position field localisation");
            } else {
                SmartDashboard.putString(tableName + "info", "this apriltag is note used for position localisation");
            }
        }

        /**
         * 
         * @param yaw 
         * @param pitch
         * @param area
         * @param detected
         */
        public void update(double yaw, double pitch, double area, boolean detected) {
            this.yaw = yaw;
            this.pitch = pitch;
            this.area = area;
            this.detected = detected;
            updatedOnCycle = true;

            if (detected && goodForPositionMeasurements) {
                hasPose = true;
            }
            
            SmartDashboard.putNumber(tableName + "Yaw", yaw);
            SmartDashboard.putNumber(tableName + "Pitch", pitch);
            SmartDashboard.putNumber(tableName + "Area", area);
            SmartDashboard.putBoolean(tableName + "Detected", detected);
        }

        /**
         * Clears the data when not updated on the cycle, if it was updated it does litteraly nothing
         */
        public void resetData() {
            if (hasPose) {
                hasPose = false;
            }
            if (!updatedOnCycle) {
                update(0.0, 0.0, 0.0, false);
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
    public final ApriltagData getApriltag(int id) {
        return aprilTagDatas.get(id-1);
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
                    tagData.update(target.getYaw(), target.getPitch(), target.getArea(), true);
                }
            }
            // only updates the position if one of april tags was detected that is supposed to be used for vision
            if (ApriltagData.validPositionMeasurement()) {
                Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
                if (estimatedPose.isPresent()) {
                    visionPose = estimatedPose.get().estimatedPose;
                    SmartDashboard.putString(cameraName + "/Estimated Robot Pose", visionPose.toString());
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
