package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
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

public class Vision extends SubsystemBase {
    private Chassis chassis;
    private final PhotonCamera[] cameras; // Array of cameras
    private final Transform3d[] cameraToRobotTransforms; // Array of camera-to-robot transforms
    private final PhotonPoseEstimator[] poseEstimators; // Array of pose estimators
    private final GenericEntry[] robotNameWidgets;
    private final GenericEntry[] robotPoseXWidgets;
    private final GenericEntry[] robotPoseYWidgets;
    private final GenericEntry[] robotPoseZWidgets;
    private final GenericEntry[] robotRotationWidgets;
    private final GenericEntry[] numTargetsWidgets;
    private final GenericEntry[] hasTargetsWidgets;

    public void setChassis(Chassis chassis) {
        this.chassis = chassis;
    }

    public Vision(PhotonCamera camera1, PhotonCamera camera2, PhotonCamera camera3, PhotonCamera camera4) {
        this.cameras = new PhotonCamera[] { camera1, camera2, camera3, camera4 };

        // Define the camera-to-robot transforms for each camera (adjust for the robot)
        this.cameraToRobotTransforms = new Transform3d[] {
                new Transform3d(new Translation3d( // camera 1 back right
                        Units.inchesToMeters(11.8251), // x
                        Units.inchesToMeters(-12.0819), // y
                        Units.inchesToMeters(8.5062)), // z
                        new Rotation3d(0.0, // roll
                                Units.degreesToRadians(35.0), // pitch
                                Units.degreesToRadians(170.0))), // yaw
                new Transform3d(new Translation3d( // camera 2 back left
                        Units.inchesToMeters(-11.8251), // x
                        Units.inchesToMeters(-12.0819), // y
                        Units.inchesToMeters(8.5062)), // z
                        new Rotation3d(0.0, // roll
                                Units.degreesToRadians(35.0), // pitch
                                Units.degreesToRadians(-170.0))), // yaw
                new Transform3d(new Translation3d( // camera 3 front left
                        Units.inchesToMeters(-11.8251), // x
                        Units.inchesToMeters(12.0819), // y
                        Units.inchesToMeters(8.5062)), // z
                        new Rotation3d(0.0, // roll
                                Units.degreesToRadians(15.0), // pitch
                                Units.degreesToRadians(-30.0))), // yaw
                new Transform3d(new Translation3d( // camera 4 front right
                        Units.inchesToMeters(11.8251), // x
                        Units.inchesToMeters(12.0819), // y
                        Units.inchesToMeters(8.5062)), // z
                        new Rotation3d(0.0, // roll
                                Units.degreesToRadians(15.0), // pitch
                                Units.degreesToRadians(30.0))) // yaw
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

        this.robotNameWidgets = new GenericEntry[cameras.length];
        this.robotPoseXWidgets = new GenericEntry[cameras.length];
        this.robotPoseYWidgets = new GenericEntry[cameras.length];
        this.robotPoseZWidgets = new GenericEntry[cameras.length];
        this.robotRotationWidgets = new GenericEntry[cameras.length]; // ngl the dent bothers me
        this.numTargetsWidgets = new GenericEntry[cameras.length];
        this.hasTargetsWidgets = new GenericEntry[cameras.length];

        for (int i = 0; i < cameras.length; i++) {
            String cameraName = "Cameras";
            robotNameWidgets[i] = Shuffleboard.getTab(cameraName)
                    .add("Name " + i, "")
                    .withWidget(BuiltInWidgets.kTextView)
                    .withPosition(2 * i, 0)
                    .withSize(2, 2)
                    .getEntry();
            // .withProperties(Map.of("min", -10, "max", 10)); // Adjust min/max as needed
            hasTargetsWidgets[i] = Shuffleboard.getTab(cameraName)
                    .add("Has Tgts " + i, false)
                    .withWidget(BuiltInWidgets.kBooleanBox)
                    .withPosition(2 * i, 2)
                    .withSize(2, 2)
                    .getEntry();
            numTargetsWidgets[i] = Shuffleboard.getTab(cameraName)
                    .add("Num Tgts " + i, 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withPosition(2 * i, 4)
                    .withSize(2, 2)
                    .getEntry();
            // .withProperties(Map.of("min", 0, "max", 10));
            // .withProperties(Map.of("min", -10, "max", 10)); // Adjust min/max as needed
            robotPoseXWidgets[i] = Shuffleboard.getTab(cameraName)
                    .add("Pose X " + i, 0.0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withPosition(2 * i, 6)
                    .withSize(2, 2)
                    .getEntry();
            // .withProperties(Map.of("min", -10, "max", 10)); // Adjust min/max as needed
            robotPoseYWidgets[i] = Shuffleboard.getTab(cameraName)
                    .add("Pose Y " + i, 0.0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withPosition(2 * i, 8)
                    .withSize(2, 2)
                    .getEntry();
            // .withProperties(Map.of("min", -10, "max", 10)); // Adjust min/max as needed
            robotPoseZWidgets[i] = Shuffleboard.getTab(cameraName)
                    .add("Pose Z " + i, 0.0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withPosition(2 * i, 10)
                    .withSize(2, 2)
                    .getEntry();
            // .withProperties(Map.of("min", -10, "max", 10)); // Adjust min/max as needed
            robotRotationWidgets[i] = Shuffleboard.getTab(cameraName)
                    .add("Rotation " + i, "N/A")
                    .withWidget(BuiltInWidgets.kTextView)
                    .withPosition(2 * i, 12)
                    .withSize(2, 2)
                    .getEntry();
        }
    }

    @Override
    public void periodic() {
        // List<Pose3d> validPoses = new ArrayList<>();

        // Process data for all cameras
        for (int i = 0; i < cameras.length; i++) {
            Optional<Pair<Pose3d, Double>> poseAndTimestamp = processCamera(cameras[i], poseEstimators[i], i + 1);
            if (poseAndTimestamp.isPresent()) {
                Pose3d pose3d = poseAndTimestamp.get().getFirst();
                double timestamp = poseAndTimestamp.get().getSecond();

                this.robotNameWidgets[i].setString(cameras[i].getName());
                this.robotPoseXWidgets[i].setDouble(pose3d.getX());
                this.robotPoseYWidgets[i].setDouble(pose3d.getY());
                this.robotPoseZWidgets[i].setDouble(pose3d.getZ());
                this.robotRotationWidgets[i].setDouble(pose3d.getRotation().getAngle());

                List<PhotonPipelineResult> results = cameras[i].getAllUnreadResults();
                PhotonPipelineResult latestResult = getNewestResult(results);
                this.numTargetsWidgets[i].setDouble(latestResult.getTargets().size());
                this.hasTargetsWidgets[i].setBoolean(latestResult.hasTargets());

                Pose2d pose2d = getVisionPose2d(pose3d);
                chassis.addVisionMeasurement(pose2d, timestamp); // Use PhotonVision's timestamp
            }
        }
    }

    private Optional<Pair<Pose3d, Double>> processCamera(PhotonCamera camera, PhotonPoseEstimator poseEstimator,
            int cameraNumber) {
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
                return Optional.of(new Pair<>(robotPose3d, latestResult.getTimestampSeconds())); // Return pose and
                                                                                                 // timestamp

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
        Logger.recordOutput(cameraKey + "Rotation", pose.getRotation().getAngle());

        // robotPoseXWidgets[widgetIndex].setDouble(pose.getX());
        // robotPoseYWidgets[widgetIndex].setDouble(pose.getY());
        // robotPoseZWidgets[widgetIndex].setDouble(pose.getZ());
        // robotRotationWidgets[widgetIndex].setString(pose.getRotation().toString());
        // numTargetsWidgets[widgetIndex].setDouble(numTargets);
    }

    // Helper method to clear Shuffleboard data
    private void clearShuffleboardData(int widgetIndex) {
        robotPoseXWidgets[widgetIndex].setDouble(0.0);
        robotPoseYWidgets[widgetIndex].setDouble(0.0);
        robotPoseZWidgets[widgetIndex].setDouble(0.0);
        robotRotationWidgets[widgetIndex].setDouble(-99.0);
        numTargetsWidgets[widgetIndex].setDouble(0);
    }

    // Helper method for converting Pose3d to Pose2d
    public Pose2d getVisionPose2d(Pose3d pose3d) {
        return new Pose2d(
                new Translation2d(pose3d.getX(), // X position
                        pose3d.getY()), // Y position
                new Rotation2d(pose3d.getRotation().toRotation2d().getDegrees()) // Yaw (rotation around Z-axis)
        );
    }
}

// private Pose3d averagePoses(Pose3d[] poses) {
// if (poses.length == 0) {
// return null; // No valid poses to average
// }

// double totalX = 0, totalY = 0, totalZ = 0;
// double totalRoll = 0, totalPitch = 0, totalYaw = 0;

// for (Pose3d pose : poses) {
// totalX += pose.getX();
// totalY += pose.getY();
// totalZ += pose.getZ();

// Rotation3d rotation = pose.getRotation();
// totalRoll += rotation.getX();
// totalPitch += rotation.getY();
// totalYaw += rotation.getZ();
// }

// double avgX = totalX / poses.length;
// double avgY = totalY / poses.length;
// double avgZ = totalZ / poses.length;
// double avgRoll = totalRoll / poses.length;
// double avgPitch = totalPitch / poses.length;
// double avgYaw = totalYaw / poses.length;

// return new Pose3d(
// new Translation3d(avgX, avgY, avgZ),
// new Rotation3d(avgRoll, avgPitch, avgYaw)
// );

// }
