package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Map;

public class Vision extends SubsystemBase {
    private final PhotonCamera[] cameras; // Array of cameras
    private final Transform3d[] cameraToRobotTransforms; // Array of camera-to-robot transforms
    private final PhotonPoseEstimator[] poseEstimators; // Array of pose estimators
    private final SimpleWidget[] robotPoseXWidgets;
    private final SimpleWidget[] robotPoseYWidgets;
    private final SimpleWidget[] robotPoseZWidgets;
    private final SimpleWidget[] robotRotationWidgets;
    private final SimpleWidget[] numTargetsWidgets;
    

    public Vision(PhotonCamera camera1, PhotonCamera camera2, PhotonCamera camera3, PhotonCamera camera4) {
        this.cameras = new PhotonCamera[] { camera1, camera2, camera3, camera4 };

        // Define the camera-to-robot transforms for each camera (adjust for the robot)
        this.cameraToRobotTransforms = new Transform3d[] {
            new Transform3d(new Translation3d(0.2, -0.1, 0.5), new Rotation3d(0, -0.175, 0)), // Camera 1
            new Transform3d(new Translation3d(0.2, 0.1, 0.5), new Rotation3d(0, -0.175, 0)),  // Camera 2
            new Transform3d(new Translation3d(-0.2, -0.1, 0.5), new Rotation3d(0, -0.175, 0)), // Camera 3
            new Transform3d(new Translation3d(-0.2, 0.1, 0.5), new Rotation3d(0, -0.175, 0))   // Camera 4
        };

        // Initialize pose estimators for each camera
        this.poseEstimators = new PhotonPoseEstimator[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            poseEstimators[i] = new PhotonPoseEstimator(
                null, // Field layout is handled by PhotonVision on the Orange Pi 5
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraToRobotTransforms[i] // Uses the corresponding transform
                
            );
        }

        this.robotPoseXWidgets = new SimpleWidget[cameras.length];
        this.robotPoseYWidgets = new SimpleWidget[cameras.length];
        this.robotPoseZWidgets = new SimpleWidget[cameras.length];
        this.robotRotationWidgets = new SimpleWidget[cameras.length]; // ngl the dent bothers me
        this.numTargetsWidgets = new SimpleWidget[cameras.length];

        for (int i = 0; i < cameras.length; i++) {
            String cameraName = "Camera" + (i + 1);
            robotPoseXWidgets[i] = Shuffleboard.getTab(cameraName)
                .add("Robot Pose X", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -10, "max", 10)); // Adjust min/max as needed
            robotPoseYWidgets[i] = Shuffleboard.getTab(cameraName)
                .add("Robot Pose Y", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -10, "max", 10)); // Adjust min/max as needed
            robotPoseZWidgets[i] = Shuffleboard.getTab(cameraName)
                .add("Robot Pose Z", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -10, "max", 10)); // Adjust min/max as needed
            robotRotationWidgets[i] = Shuffleboard.getTab(cameraName)
                .add("Robot Rotation", "N/A")
                .withWidget(BuiltInWidgets.kTextView);
            numTargetsWidgets[i] = Shuffleboard.getTab(cameraName)
                .add("Num Targets", 0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 0, "max", 10));
        }
    }

    @Override
    public void periodic() {
        List<Pose3d> validPoses = new ArrayList<>();

        // Process data for all cameras
        for (int i = 0; i < cameras.length; i++) {
            Optional<Pose3d> pose = processCamera(cameras[i], poseEstimators[i], i + 1);
            if (pose.isPresent()) {
                validPoses.add(pose.get());
            }
        }

        // Average the poses if there are valid estimates
        if (!validPoses.isEmpty()) {
            Pose3d averagedPose = averagePoses(validPoses.toArray(new Pose3d[0]));
            // Log the averaged pose
            Logger.recordOutput("Vision/AveragedPose/X", averagedPose.getX());
            Logger.recordOutput("Vision/AveragedPose/Y", averagedPose.getY());
            Logger.recordOutput("Vision/AveragedPose/Z", averagedPose.getZ());
            Logger.recordOutput("Vision/AveragedPose/Rotation", averagedPose.getRotation().toString());
        } else {
            Logger.recordOutput("Vision/AveragedPose/Valid", false);
        }
    }

    private Optional<Pose3d> processCamera(PhotonCamera camera, PhotonPoseEstimator poseEstimator, int cameraNumber) {
        PhotonPipelineResult result = camera.getLatestResult();
        int widgetIndex = cameraNumber - 1; // Convert cameraNumber (1-4) to index (0-3)
    
        if (result.hasTargets()) {
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
            if (estimatedPose.isPresent()) {
                Pose3d robotPose3d = estimatedPose.get().estimatedPose;
    
                // Logging
                String cameraKey = "Vision/Camera" + cameraNumber + "/";
                Logger.recordOutput(cameraKey + "HasTargets", true);
                Logger.recordOutput(cameraKey + "NumTargets", result.getTargets().size());
                Logger.recordOutput(cameraKey + "Pose/X", robotPose3d.getX());
                Logger.recordOutput(cameraKey + "Pose/Y", robotPose3d.getY());
                Logger.recordOutput(cameraKey + "Pose/Z", robotPose3d.getZ());
                Logger.recordOutput(cameraKey + "Rotation", robotPose3d.getRotation().toString());
    
                // Update Shuffleboard
                robotPoseXWidgets[widgetIndex].getEntry().setDouble(robotPose3d.getX());
                robotPoseYWidgets[widgetIndex].getEntry().setDouble(robotPose3d.getY());
                robotPoseZWidgets[widgetIndex].getEntry().setDouble(robotPose3d.getZ());
                robotRotationWidgets[widgetIndex].getEntry().setString(robotPose3d.getRotation().toString());
                numTargetsWidgets[widgetIndex].getEntry().setDouble(result.getTargets().size());
    
                return Optional.of(robotPose3d);
            } else {
                // Pose estimation failed
                String cameraKey = "Vision/Camera" + cameraNumber + "/";
                Logger.recordOutput(cameraKey + "HasTargets", false);
                Logger.recordOutput(cameraKey + "PoseEstimationFailed", true);
    
                // Clear Shuffleboard data
                robotPoseXWidgets[widgetIndex].getEntry().setDouble(0.0);
                robotPoseYWidgets[widgetIndex].getEntry().setDouble(0.0);
                robotPoseZWidgets[widgetIndex].getEntry().setDouble(0.0);
                robotRotationWidgets[widgetIndex].getEntry().setString("N/A");
                numTargetsWidgets[widgetIndex].getEntry().setDouble(0);
            }
        } else {
            // No targets detected
            String cameraKey = "Vision/Camera" + cameraNumber + "/";
            Logger.recordOutput(cameraKey + "HasTargets", false);
    
            // Clear Shuffleboard data
            robotPoseXWidgets[widgetIndex].getEntry().setDouble(0.0);
            robotPoseYWidgets[widgetIndex].getEntry().setDouble(0.0);
            robotPoseZWidgets[widgetIndex].getEntry().setDouble(0.0);
            robotRotationWidgets[widgetIndex].getEntry().setString("N/A");
            numTargetsWidgets[widgetIndex].getEntry().setDouble(0);
        }
    
        return Optional.empty();
    }

    private Pose3d averagePoses(Pose3d[] poses) {
        if (poses.length == 0) {
            return null; // No valid poses to average
        }

        double totalX = 0, totalY = 0, totalZ = 0;
        double totalRoll = 0, totalPitch = 0, totalYaw = 0;

        for (Pose3d pose : poses) {
            totalX += pose.getX();
            totalY += pose.getY();
            totalZ += pose.getZ();

            Rotation3d rotation = pose.getRotation();
            totalRoll += rotation.getX();
            totalPitch += rotation.getY();
            totalYaw += rotation.getZ();
        }

        double avgX = totalX / poses.length;
        double avgY = totalY / poses.length;
        double avgZ = totalZ / poses.length;
        double avgRoll = totalRoll / poses.length;
        double avgPitch = totalPitch / poses.length;
        double avgYaw = totalYaw / poses.length;

        return new Pose3d(
            new Translation3d(avgX, avgY, avgZ),
            new Rotation3d(avgRoll, avgPitch, avgYaw)
        );
    }
}