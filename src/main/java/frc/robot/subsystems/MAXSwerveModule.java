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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveModuleConstants;

public class MAXSwerveModule {
	private final SparkFlex m_drivingSparkFlex;
	private final SparkMax m_turningSparkMax;

	private int m_drivingCANId = 0;
	private int m_turningCANId = 0;

	private final SparkMaxConfig m_drivingConfig = new SparkMaxConfig();
	private final SparkMaxConfig m_turningConfig = new SparkMaxConfig();

	private final SparkClosedLoopController m_drivingController;
	private final SparkClosedLoopController m_turningController;

	private final RelativeEncoder m_drivingEncoder;
	private final AbsoluteEncoder m_turningEncoder;

	private double m_chassisAngularOffset = 0;
	private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

	/**
	 * Constructs a MAXSwerveModule and configures the driving and turning motor,
	 * encoder, and PID controller. This configuration is specific to the REV
	 * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
	 * Encoder.
	 */
	public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
		m_drivingCANId = drivingCANId;
		m_turningCANId = turningCANId;

		m_drivingSparkFlex = new SparkFlex(drivingCANId, MotorType.kBrushless);
		m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

		m_drivingConfig
				.inverted(SwerveModuleConstants.kDrivingMotorInverted)
				.idleMode(SwerveModuleConstants.kDrivingMotorIdleMode)
				.smartCurrentLimit(SwerveModuleConstants.kDrivingMotorCurrentLimit);
		m_drivingConfig.encoder
				.positionConversionFactor(SwerveModuleConstants.kDrivingEncoderPositionFactor)
				.velocityConversionFactor(SwerveModuleConstants.kDrivingEncoderVelocityFactor);
		m_drivingConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pidf(SwerveModuleConstants.kDrivingP,
						SwerveModuleConstants.kDrivingI,
						SwerveModuleConstants.kDrivingD,
						SwerveModuleConstants.kDrivingFF)
				.outputRange(SwerveModuleConstants.kDrivingMinOutput, SwerveModuleConstants.kDrivingMaxOutput)
				.positionWrappingEnabled(SwerveModuleConstants.kDrivingEncodeWrapping);

		m_drivingController = m_drivingSparkFlex.getClosedLoopController();
		m_drivingController.setReference(0.0, SparkBase.ControlType.kMAXMotionVelocityControl);

		m_drivingSparkFlex.configure(m_drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		m_turningConfig
				.inverted(SwerveModuleConstants.kTurningMotorInverted)
				.idleMode(SwerveModuleConstants.kTurningMotorIdleMode)
				.smartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit);
		m_turningConfig.encoder
				.positionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor)
				.velocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor);
		m_turningConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pidf(SwerveModuleConstants.kTurningP,
						SwerveModuleConstants.kTurningI,
						SwerveModuleConstants.kTurningD,
						SwerveModuleConstants.kTurningFF)
				.outputRange(SwerveModuleConstants.kTurningMinOutput, SwerveModuleConstants.kTurningMaxOutput)
				.positionWrappingEnabled(SwerveModuleConstants.kTurningEncodeWrapping)
				.positionWrappingInputRange(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput,
						SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

		m_turningController = m_turningSparkMax.getClosedLoopController();
		m_turningController.setReference(0.0, ControlType.kMAXMotionPositionControl);

		m_turningSparkMax.configure(m_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Setup encoders and PID controllers for the driving and turning SPARKS MAX.
		m_drivingEncoder = m_drivingSparkFlex.getEncoder();
		m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(); // (Type.kDutyCycle);

		// drivingPIDController = drivingSparkMax.getPIDController();
		// turningPIDController = turningSparkMax.getPIDController();

		// drivingPIDController.setFeedbackDevice(drivingEncoder);
		// turningPIDController.setFeedbackDevice(turningEncoder);

		// Save the SPARK MAX configurations. If a SPARK MAX browns out during
		// operation, it will maintain the above configurations.
		// drivingSparkMax.burnFlash();
		// turningSparkMax.burnFlash();

		m_chassisAngularOffset = chassisAngularOffset;
		m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
		m_drivingEncoder.setPosition(0);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModuleState(m_drivingEncoder.getVelocity(),
				new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
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
				m_drivingEncoder.getPosition(),
				new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
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
		correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

		// Optimize the reference state to avoid spinning further than 90 degrees.
		correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

		// Command driving and turning SPARKS MAX towards their respective setpoints.
		m_drivingController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
		m_turningController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

		m_desiredState = desiredState;
	}

	/** Zeroes all the SwerveModule encoders. */
	public void resetEncoders() {
		this.m_drivingEncoder.setPosition(0);
	}

	public double getDriveVel() {
		return m_drivingEncoder.getVelocity();
	}

	public double getTurnPos() {
		return m_turningEncoder.getPosition();
	}

	public int getDrivingCANId() {
		return m_drivingCANId;
	}

	public int getTurningCANId() {
		return m_turningCANId;
	}

	// public double getDrivePosFactor() {
	// return m_drivingEncoder.getPositionConversionFactor();
	// }

	// public double getDriveVelFactor() {
	// return m_drivingEncoder.getVelocityConversionFactor();
	// }

	// ----- Simulation //TODO Add Simulation

	// public void simulationUpdate(
	// double driveEncoderDist,
	// double driveEncoderRate,
	// double driveCurrent,
	// double steerEncoderDist,
	// double steerEncoderRate,
	// double steerCurrent) {
	// driveEncoderSim.setDistance(driveEncoderDist);
	// driveEncoderSim.setRate(driveEncoderRate);
	// this.driveCurrentSim = driveCurrent;
	// steerEncoderSim.setDistance(steerEncoderDist);
	// steerEncoderSim.setRate(steerEncoderRate);
	// this.steerCurrentSim = steerCurrent;
	// }

	// public double getDriveCurrentSim() {
	// return driveCurrentSim;
	// }

	// public double getSteerCurrentSim() {
	// return steerCurrentSim;
	// }
}
