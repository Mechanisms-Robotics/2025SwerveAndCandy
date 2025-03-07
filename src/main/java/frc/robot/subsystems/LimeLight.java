package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
    PhotonCamera camera;
    private final String cameraName;

    public LimeLight(String cameraName) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
    }

    @Override
    public void periodic() {
        var results = camera.getAllUnreadResults();
        SmartDashboard.putBoolean(cameraName + "/connected", camera.isConnected());

        SmartDashboard.putNumber(cameraName + "/results", results.size());
        for (PhotonPipelineResult result : results) {
            SmartDashboard.putBoolean(cameraName + "/April tag detected", result.hasTargets());
        }
    }
}
