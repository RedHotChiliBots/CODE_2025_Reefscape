
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      CANIdConstants.kFrontLeftDrivingCanId,
      CANIdConstants.kFrontLeftTurningCanId,
      ChassisConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      CANIdConstants.kFrontRightDrivingCanId,
      CANIdConstants.kFrontRightTurningCanId,
      ChassisConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      CANIdConstants.kRearLeftDrivingCanId,
      CANIdConstants.kRearLeftTurningCanId,
      ChassisConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
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
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(ChassisConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(ChassisConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      ChassisConstants.kDriveKinematics,
      // Rotation2d.fromDegrees(getAngle()),
      getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
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
    robotPose = m_odometry.update(
//        Rotation2d.fromDegrees(getAngle()),
        getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Add Swerve Drive to SmartDashboard
    SwerveModuleState[] currentStates = new SwerveModuleState[4];

    currentStates[0] = m_frontLeft.getState();
    currentStates[1] = m_frontRight.getState();
    currentStates[2] = m_rearLeft.getState();
    currentStates[3] = m_rearRight.getState();

    double[] stateAdv = new double[8];

    for (int i = 0; i < 4; i++) {
      stateAdv[2 * i] = currentStates[i].angle.getDegrees();
      stateAdv[2 * i + 1] = currentStates[i].speedMetersPerSecond;
    }

    SmartDashboard.putNumberArray("swerve/status", stateAdv);
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
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        getRotation2d(),
//        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
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
        m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());

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
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
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
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(ChassisConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * ChassisConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * ChassisConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * ChassisConstants.kMaxAngularSpeed;

    var swerveModuleStates = ChassisConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, ChassisConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void stopChassis() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, m_frontLeft.getState().angle));
    m_frontRight.setDesiredState(new SwerveModuleState(0, m_frontRight.getState().angle));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, m_rearLeft.getState().angle));
    m_rearRight.setDesiredState(new SwerveModuleState(0, m_rearRight.getState().angle));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, ChassisConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
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
