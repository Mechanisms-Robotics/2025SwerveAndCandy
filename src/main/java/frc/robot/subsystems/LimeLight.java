package frc.robot.subsystems;

import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase {
    private final PhotonCamera camera;
    private final String cameraName;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    // private final PhotonPoseEstimator poseEstimator;
    private final ArrayList<ApriltagData> aprilTagDatas = new ArrayList<>();


    public LimeLight(String cameraName) {
        camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;

        aprilTagFieldLayout = AprilTagFieldLayout.loadField(Constants.Vision.FIELD);
        aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

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
        public boolean updatedOnCycle = false;

        public ApriltagData(int id) {
            this.id = id;
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
            String tableName = cameraName + "/AprilTags/" + id + "/";
            
            SmartDashboard.putNumber(tableName + "Yaw", yaw);
            SmartDashboard.putNumber(tableName + "Pitch", pitch);
            SmartDashboard.putNumber(tableName + "Area", area);
            SmartDashboard.putBoolean(tableName + "Detected", detected);

        }

        /**
         * Clears the data when not updated on the cycle, if it was updated it does litteraly nothing
         */
        public void clearWhenOutdated() {
            if (!updatedOnCycle) {
                update(0.0, 0.0, 0.0, false);
            }
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

        // assume the april tag was not updated on this cycle until detected
        for (int i = 0; i < aprilTagDatas.size(); i++) {
            // this is not refering to the april tag with the i id, because the 0th element has an id of 1
            aprilTagDatas.get(i).updatedOnCycle = false;
        }

        for (PhotonPipelineResult result : results) {
            SmartDashboard.putBoolean(cameraName + "/Apriltag detected", result.hasTargets());

            if (result.hasTargets()) {
                SmartDashboard.putBoolean(cameraName + "/Result Empty", result.getMultiTagResult().isEmpty());
                for (PhotonTrackedTarget target : result.targets) {
                    ApriltagData tagData = getApriltag(target.getFiducialId());
                    // now we know the april tag was detected, updated all of the values of that tag id
                    tagData.update(target.getYaw(), target.getPitch(), target.getArea(), true);
                }
            }
        }
        // if the apriltag was not updated, clear its data because otherwise the data from the last time it was detected stays
        for (int i = 0; i < aprilTagDatas.size(); i++) {
            aprilTagDatas.get(i).clearWhenOutdated();
        }
    }
}
