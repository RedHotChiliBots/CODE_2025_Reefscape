
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.WPIUtilJNI;
//import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeft = new MAXSwerveModule(
      CANIdConstants.kFrontLeftDrivingCanId,
      CANIdConstants.kFrontLeftTurningCanId,
      ChassisConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRight = new MAXSwerveModule(
      CANIdConstants.kFrontRightDrivingCanId,
      CANIdConstants.kFrontRightTurningCanId,
      ChassisConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule rearLeft = new MAXSwerveModule(
      CANIdConstants.kRearLeftDrivingCanId,
      CANIdConstants.kRearLeftTurningCanId,
      ChassisConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule rearRight = new MAXSwerveModule(
      CANIdConstants.kRearRightDrivingCanId,
      CANIdConstants.kRearRightTurningCanId,
      ChassisConstants.kBackRightChassisAngularOffset);

  // ==============================================================
  // Initialize NavX AHRS board
  // Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
  private final AHRS ahrs = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private Pose2d robotPose = null;

  // The gyro sensor
  // private final ADIS16470_IMU ahrs = new ADIS16470_IMU();

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(ChassisConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(ChassisConstants.kRotationalSlewRate);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      ChassisConstants.kDriveKinematics,
      // Rotation2d.fromDegrees(getAngle()),
      getRotation2d(),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
      },
      new Pose2d(0.0, 0.0, new Rotation2d(Math.PI/2.0))); //TODO verify starting position

  // ==============================================================
  // Define Shuffleboard data - Chassis Tab
  private final ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");
  private final GenericEntry sbAngle = chassisTab.addPersistent("Angle", 0)
      .withWidget("Text View").withPosition(5, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbHeading = chassisTab.addPersistent("Heading", 0)
      .withWidget("Text View").withPosition(5, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbFusedHeading = chassisTab.addPersistent("FusedHeading", 0)
      .withWidget("Text View").withPosition(5, 2).withSize(2, 1).getEntry();
  private final GenericEntry sbPitch = chassisTab.addPersistent("Pitch", 0)
      .withWidget("Text View").withPosition(4, 0).withSize(1, 1).getEntry();
  private final GenericEntry sbRoll = chassisTab.addPersistent("Roll", 0)
      .withWidget("Text View").withPosition(4, 1).withSize(1, 1).getEntry();
  private final GenericEntry sbYaw = chassisTab.addPersistent("Yaw", 0)
      .withWidget("Text View").withPosition(4, 2).withSize(1, 1).getEntry();
  private final GenericEntry sbRotation2d = chassisTab.addPersistent("Rotation2D Angle", 0)
      .withWidget("Text View").withPosition(4, 3).withSize(2, 1).getEntry();

  /** Creates a new DriveSubsystem. */
  public Chassis() {
    System.out.println("+++++ Starting Chassis Constructor +++++");
    zeroHeading();
    System.out.println("----- Ending Chassis Constructor -----");
  }

  @Override
  public void periodic() {
    sbAngle.setDouble(getAngle());
    sbHeading.setDouble(getHeading());
    sbYaw.setDouble(getYaw());
    sbFusedHeading.setDouble(getFusedHeading());
    sbPitch.setDouble(getPitch());
    sbRoll.setDouble(getRoll());
    sbRotation2d.setDouble(getRotation2d().getDegrees());

    // Update the odometry in the periodic block
    robotPose = odometry.update(
//        Rotation2d.fromDegrees(getAngle()),
        getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });

    // Add Swerve Drive to SmartDashboard
    SwerveModuleState[] currentStates = new SwerveModuleState[4];

    currentStates[0] = frontLeft.getState();
    currentStates[1] = frontRight.getState();
    currentStates[2] = rearLeft.getState();
    currentStates[3] = rearRight.getState();

    double[] stateAdv = new double[8];

    for (int i = 0; i < 4; i++) {
      stateAdv[2 * i] = currentStates[i].angle.getDegrees();
      stateAdv[2 * i + 1] = currentStates[i].speedMetersPerSecond;
    }

    SmartDashboard.putNumberArray("swerve/status", stateAdv);
  }

    /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digiktal sensor.
    return false;
  }

    /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public Rotation2d getRotation2d() {
    // Negating the angle because WPILib gyros are CW positive.
    return ahrs.getRotation2d();
  }

  public double getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return -ahrs.getAngle();
  }

  public double getYaw() {
    // Negating the angle because WPILib gyros are CW positive.
    return -ahrs.getYaw();
  }

  public double getPitch() {
    // Negating the angle because WPILib gyros are CW positive.
    return -ahrs.getPitch();
  }

  public double getRoll() {
    // Negating the angle because WPILib gyros are CW positive.
    return -ahrs.getRoll();
  }

  public double getFusedHeading() {
    // Negating the angle because WPILib gyros are CW positive.
    return -ahrs.getFusedHeading();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        getRotation2d(),
//        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
  }

  /**
   * 
   * @param xSpeed
   * @param ySpeed
   * @param rot
   * @param fieldRelative
   * @param rateLimit
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Convert module states to chassis speeds
    ChassisSpeeds chassisSpeeds = ChassisConstants.kDriveKinematics.toChassisSpeeds(
        frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState());

    return chassisSpeeds;
  }

  /**
   * 
   * @param xSpeed
   * @param ySpeed
   * @param rot
   * @param fieldRelative
   * @param rateLimit
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates = ChassisConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, ChassisConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  private boolean flipPath = false;

  public void setFlipPath(boolean fp) {
    flipPath = fp;
  }

  public boolean getFlipPath() {
    return flipPath;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(ChassisConstants.kDirectionSlewRate / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        } else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      } else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;

      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * ChassisConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * ChassisConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * ChassisConstants.kMaxAngularSpeed;

    var swerveModuleStates = ChassisConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, ChassisConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void stopChassis() {
    frontLeft.setDesiredState(new SwerveModuleState(0, frontLeft.getState().angle));
    frontRight.setDesiredState(new SwerveModuleState(0, frontRight.getState().angle));
    rearLeft.setDesiredState(new SwerveModuleState(0, rearLeft.getState().angle));
    rearRight.setDesiredState(new SwerveModuleState(0, rearRight.getState().angle));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, ChassisConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    ahrs.reset();
    ahrs.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -ahrs.getRate() * (ChassisConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
