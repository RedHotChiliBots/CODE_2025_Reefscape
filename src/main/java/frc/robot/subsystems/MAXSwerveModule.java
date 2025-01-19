// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveModuleConstants;

public class MAXSwerveModule {
  private final SparkMax drivingSparkMax;
  private final SparkMax turningSparkMax;

  private final SparkMaxConfig drivingConfig = new SparkMaxConfig();
  private final SparkMaxConfig turningConfig = new SparkMaxConfig();

  private final SparkClosedLoopController drivingController;
  private final SparkClosedLoopController turningController;
  
  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    drivingConfig
    .inverted(SwerveModuleConstants.kDrivingMotorInverted)
    .idleMode(SwerveModuleConstants.kDrivingMotorIdleMode)
    .smartCurrentLimit(SwerveModuleConstants.kDrivingMotorCurrentLimit);
    drivingConfig.encoder
    .positionConversionFactor(SwerveModuleConstants.kDrivingEncoderPositionFactor)
    .velocityConversionFactor(SwerveModuleConstants.kDrivingEncoderVelocityFactor);
    drivingConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(SwerveModuleConstants.kDrivingP, 
    SwerveModuleConstants.kDrivingI, 
    SwerveModuleConstants.kDrivingD, 
    SwerveModuleConstants.kDrivingFF)
    .outputRange(SwerveModuleConstants.kDrivingMinOutput, SwerveModuleConstants.kDrivingMaxOutput)
    .positionWrappingEnabled(SwerveModuleConstants.kDrivingEncodeWrapping);
    
    drivingController = drivingSparkMax.getClosedLoopController();
    drivingController.setReference(0.0, SparkBase.ControlType.kMAXMotionVelocityControl);

    drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turningConfig
    .inverted(SwerveModuleConstants.kTurningMotorInverted)
    .idleMode(SwerveModuleConstants.kTurningMotorIdleMode)
    .smartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit);
    turningConfig.encoder
    .positionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor)
    .velocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor);
    turningConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(SwerveModuleConstants.kTurningP, 
    SwerveModuleConstants.kTurningI, 
    SwerveModuleConstants.kTurningD, 
    SwerveModuleConstants.kTurningFF)
    .outputRange(SwerveModuleConstants.kTurningMinOutput, SwerveModuleConstants.kTurningMaxOutput)
    .positionWrappingEnabled(SwerveModuleConstants.kTurningEncodeWrapping)
    .positionWrappingInputRange(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput, SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);
    
    turningController = turningSparkMax.getClosedLoopController();
    turningController.setReference(0.0, ControlType.kMAXMotionPositionControl);

    turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    drivingEncoder = drivingSparkMax.getEncoder();
    turningEncoder = turningSparkMax.getAbsoluteEncoder();  //    (Type.kDutyCycle);

    // drivingPIDController = drivingSparkMax.getPIDController();
    // turningPIDController = turningSparkMax.getPIDController();

    // drivingPIDController.setFeedbackDevice(drivingEncoder);
    // turningPIDController.setFeedbackDevice(turningEncoder);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    // drivingSparkMax.burnFlash();
    // turningSparkMax.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    this.desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    this.drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(drivingEncoder.getVelocity(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        drivingEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    this.drivingController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    this.turningController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    this.drivingEncoder.setPosition(0);
  }
}
