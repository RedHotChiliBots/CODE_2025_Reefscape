package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Pair;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;
import java.util.Map;

public class Vision extends SubsystemBase {
    private Chassis chassis;
    private final PhotonCamera[] cameras; // Array of cameras
    private final Transform3d[] cameraToRobotTransforms; // Array of camera-to-robot transforms
    private final PhotonPoseEstimator[] poseEstimators; // Array of pose estimators
    private final SimpleWidget[] robotPoseXWidgets;
    private final SimpleWidget[] robotPoseYWidgets;
    private final SimpleWidget[] robotPoseZWidgets;
    private final SimpleWidget[] robotRotationWidgets;
    private final SimpleWidget[] numTargetsWidgets;

    public void setChassis(Chassis chassis) {
        this.chassis = chassis;
    }

    public Vision(PhotonCamera camera1, PhotonCamera camera2, PhotonCamera camera3, PhotonCamera camera4) {
        this.cameras = new PhotonCamera[] { camera1, camera2, camera3, camera4 };
        
        // Define the camera-to-robot transforms for each camera (adjust for the robot)
        this.cameraToRobotTransforms = new Transform3d[] {
                new Transform3d(new Translation3d( // camera 1  back right
                        Units.inchesToMeters(11.8251), // x
                        Units.inchesToMeters(-12.0819), // y
                        Units.inchesToMeters(8.5062)), // z
                        new Rotation3d(0.0, // roll
                                Units.degreesToRadians(35.0), // pitch
                                Units.degreesToRadians(180.0))), // yaw
                new Transform3d(new Translation3d( // camera 2  back left
                        Units.inchesToMeters(11.8251), // x
                        Units.inchesToMeters(-12.0819), // y
                        Units.inchesToMeters(8.5062)), // z
                        new Rotation3d(0.0, // roll
                                Units.degreesToRadians(35.0), // pitch
                                Units.degreesToRadians(180.0))), // yaw
                new Transform3d(new Translation3d( // camera 3  front left
                        Units.inchesToMeters(-11.8251), // x
                        Units.inchesToMeters(12.0819), // y
                        Units.inchesToMeters(8.5062)), // z
                        new Rotation3d(0.0, // roll
                                Units.degreesToRadians(35.0), // pitch
                                Units.degreesToRadians(-45.0))), // yaw
                new Transform3d(new Translation3d( // camera 4  front right
                        Units.inchesToMeters(11.8251), // x
                        Units.inchesToMeters(12.0819), // y
                        Units.inchesToMeters(8.5062)), // z
                        new Rotation3d(0.0, // roll
                                Units.degreesToRadians(35.0), // pitch
                                Units.degreesToRadians(45.0))) // yaw
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
                .withPosition(0, 0)
                .withSize(10, 3)
                .withProperties(Map.of("min", -10, "max", 10)); // Adjust min/max as needed
            robotPoseYWidgets[i] = Shuffleboard.getTab(cameraName)
                .add("Robot Pose Y", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withPosition(0, 3)
                .withSize(10, 3)
                .withProperties(Map.of("min", -10, "max", 10)); // Adjust min/max as needed
            robotPoseZWidgets[i] = Shuffleboard.getTab(cameraName)
                .add("Robot Pose Z", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withPosition(0, 6)
                .withSize(10, 3)
                .withProperties(Map.of("min", -10, "max", 10)); // Adjust min/max as needed
            robotRotationWidgets[i] = Shuffleboard.getTab(cameraName)
                .add("Robot Rotation", "N/A")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 9)
                .withSize(10, 3);
            numTargetsWidgets[i] = Shuffleboard.getTab(cameraName)
                .add("Num Targets", 0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withPosition(0, 12)
                .withSize(10, 3)
                .withProperties(Map.of("min", 0, "max", 10));
        }
    }

    @Override
    public void periodic() {
        //List<Pose3d> validPoses = new ArrayList<>();

        // Process data for all cameras
        for (int i = 0; i < cameras.length; i++) {
            Optional<Pair<Pose3d, Double>> poseAndTimestamp = processCamera(cameras[i], poseEstimators[i], i + 1);
            if (poseAndTimestamp.isPresent()) {
                Pose3d pose3d = poseAndTimestamp.get().getFirst();
                double timestamp = poseAndTimestamp.get().getSecond();
                
                Pose2d pose2d = getVisionPose2d(pose3d);
                chassis.addVisionMeasurement(pose2d, timestamp); // Use PhotonVision's timestamp
            }
        }   
    }

    private Optional<Pair<Pose3d, Double>> processCamera(PhotonCamera camera, PhotonPoseEstimator poseEstimator, int cameraNumber) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        int widgetIndex = cameraNumber - 1; // Convert cameraNumber (1-4) to index (0-3)

        if (results.isEmpty()) {
            // No new results; clear data and exit
            String cameraKey = "Vision/Camera" + cameraNumber + "/";
            Logger.recordOutput(cameraKey + "HasTargets", false);
            clearShuffleboardData(widgetIndex);
            return Optional.empty();
        }

        PhotonPipelineResult latestResult = getNewestResult(results);
    
        if (latestResult.hasTargets()) {
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(latestResult);
            if (estimatedPose.isPresent()) {
                Pose3d robotPose3d = estimatedPose.get().estimatedPose;
                logAndUpdateShuffleboard(cameraNumber, widgetIndex, robotPose3d, latestResult.getTargets().size());
                return Optional.of(new Pair<>(robotPose3d, latestResult.getTimestampSeconds())); // Return pose and timestamp

            } else {
                // Pose estimation failed
                String cameraKey = "Vision/Camera" + cameraNumber + "/";
                Logger.recordOutput(cameraKey + "PoseEstimationFailed", true);
                clearShuffleboardData(widgetIndex);
                return Optional.empty();
            }

        } else {
            // No targets detected
            String cameraKey = "Vision/Camera" + cameraNumber + "/";
            Logger.recordOutput(cameraKey + "HasTargets", false);
            clearShuffleboardData(widgetIndex);
        }
    
        return Optional.empty();
    }

    // Helper method to extract the latest result from a list
    private PhotonPipelineResult getNewestResult(List<PhotonPipelineResult> results) {
        PhotonPipelineResult latest = null;
        double latestTimestamp = 0;
        for (PhotonPipelineResult result : results) {
            double timestamp = result.getTimestampSeconds();
            if (timestamp > latestTimestamp) {
                latestTimestamp = timestamp;
                latest = result;
            }
        }
        return latest;
    }

    // Helper method to log and update Shuffleboard
    private void logAndUpdateShuffleboard(int cameraNumber, int widgetIndex, Pose3d pose, int numTargets) {
        String cameraKey = "Vision/Camera" + cameraNumber + "/";
        Logger.recordOutput(cameraKey + "HasTargets", true);
        Logger.recordOutput(cameraKey + "NumTargets", numTargets);
        Logger.recordOutput(cameraKey + "Pose/X", pose.getX());
        Logger.recordOutput(cameraKey + "Pose/Y", pose.getY());
        Logger.recordOutput(cameraKey + "Pose/Z", pose.getZ());
        Logger.recordOutput(cameraKey + "Rotation", pose.getRotation().toString());
    
        robotPoseXWidgets[widgetIndex].getEntry().setDouble(pose.getX());
        robotPoseYWidgets[widgetIndex].getEntry().setDouble(pose.getY());
        robotPoseZWidgets[widgetIndex].getEntry().setDouble(pose.getZ());
        robotRotationWidgets[widgetIndex].getEntry().setString(pose.getRotation().toString());
        numTargetsWidgets[widgetIndex].getEntry().setDouble(numTargets);
    }

    // Helper method to clear Shuffleboard data
    private void clearShuffleboardData(int widgetIndex) {
        robotPoseXWidgets[widgetIndex].getEntry().setDouble(0.0);
        robotPoseYWidgets[widgetIndex].getEntry().setDouble(0.0);
        robotPoseZWidgets[widgetIndex].getEntry().setDouble(0.0);
        robotRotationWidgets[widgetIndex].getEntry().setString("N/A");
        numTargetsWidgets[widgetIndex].getEntry().setDouble(0);
    }

    // Helper method for converting Pose3d to Pose2d
    public Pose2d getVisionPose2d(Pose3d pose3d) {
        return new Pose2d(
            pose3d.getX(), // X position
            pose3d.getY(), // Y position
            pose3d.getRotation().toRotation2d() // Yaw (rotation around Z-axis)
        );
    }
}

    // private Pose3d averagePoses(Pose3d[] poses) {
    //     if (poses.length == 0) {
    //         return null; // No valid poses to average
    //     }

    //     double totalX = 0, totalY = 0, totalZ = 0;
    //     double totalRoll = 0, totalPitch = 0, totalYaw = 0;

    //     for (Pose3d pose : poses) {
    //         totalX += pose.getX();
    //         totalY += pose.getY();
    //         totalZ += pose.getZ();

    //         Rotation3d rotation = pose.getRotation();
    //         totalRoll += rotation.getX();
    //         totalPitch += rotation.getY();
    //         totalYaw += rotation.getZ();
    //     }

    //     double avgX = totalX / poses.length;
    //     double avgY = totalY / poses.length;
    //     double avgZ = totalZ / poses.length;
    //     double avgRoll = totalRoll / poses.length;
    //     double avgPitch = totalPitch / poses.length;
    //     double avgYaw = totalYaw / poses.length;

    //     return new Pose3d(
    //         new Translation3d(avgX, avgY, avgZ),
    //         new Rotation3d(avgRoll, avgPitch, avgYaw)
    //     );

        
    // }
