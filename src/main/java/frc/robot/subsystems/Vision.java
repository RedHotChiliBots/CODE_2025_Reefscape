package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.littletonrobotics.junction.Logger;


public class Vision extends SubsystemBase {
    private final PhotonCamera camera1;
    private final PhotonCamera camera2;
    private final PhotonCamera camera3;
    private final PhotonCamera camera4;

    public Vision(PhotonCamera camera1, PhotonCamera camera2, PhotonCamera camera3, PhotonCamera camera4) {
        this.camera1 = camera1;
        this.camera2 = camera2;
        this.camera3 = camera3;
        this.camera4 = camera4;
    }

    @Override
    public void execute() {
        // Process results from each camera
        processCameraResult(camera1, "Camera1");
        processCameraResult(camera2, "Camera2");
        processCameraResult(camera3, "Camera3");
        processCameraResult(camera4, "Camera4");
    }

    private void processCameraResult(PhotonCamera camera, String cameraName) {
        PhotonPipelineResult result = camera.getLatestResult();
    
        Logger.getInstance().recordOutput(cameraName + "/HasTargets", result.hasTargets());
        if (result.hasTargets()) {
            Logger.getInstance().recordOutput(cameraName + "/TargetYaw", result.getBestTarget().getYaw());
            Logger.getInstance().recordOutput(cameraName + "/TargetPitch", result.getBestTarget().getPitch());
        }
    }
}
