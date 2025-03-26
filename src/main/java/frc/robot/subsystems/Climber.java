package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.utils.Library;

public class Climber extends SubsystemBase {

	// Define Intake Motors
	private final SparkMax leftClimber = new SparkMax(
			Constants.CANId.kClimberLeftCanId, MotorType.kBrushless);
	private final SparkMax rightClimber = new SparkMax(
			Constants.CANId.kClimberRightCanId, MotorType.kBrushless);

	private final SparkMaxConfig leftConfig = new SparkMaxConfig();
	private final SparkMaxConfig rightConfig = new SparkMaxConfig();

	private SparkClosedLoopController leftController = leftClimber.getClosedLoopController();

	private AbsoluteEncoder leftEncoder = leftClimber.getAbsoluteEncoder();

	private SparkLimitSwitch isLimitSwitch = leftClimber.getForwardLimitSwitch();

	public enum ClimberSP {
		STOW(90.0), // degrees - up and out of way
		READY(23.0), // degrees - touch cage but don't climb
		ZERO(0.0), // degrees - for zeroing absolute encoder
		CLIMB(-23.0); // degrees - full climb

		private final double sp;

		ClimberSP(final double sp) {
			this.sp = sp;
		}

		public double getValue() {
			return sp;
		}
	}

	private ClimberSP climberSP = Climber.ClimberSP.STOW;

	private Library lib = new Library();


	/**************************************************************
	 * Initialize Shuffleboard entries
	 **************************************************************/
	private final ShuffleboardTab cmdTab = Shuffleboard.getTab("Commands");
	private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

	private final GenericEntry sbOnTgt = compTab.addPersistent("Climber OnTgt", false)
			.withWidget("Boolean Box").withPosition(7, 1)
			.withSize(2, 1).getEntry();
	private final GenericEntry sbTxtSP = compTab.addPersistent("Climber SP", "")
			.withWidget("Text View").withPosition(7, 2)
			.withSize(2, 1).getEntry();
	private final GenericEntry sbDblSP = compTab.addPersistent("Climber SP (deg)", 0)
			.withWidget("Text View").withPosition(7, 3)
			.withSize(2, 1).getEntry();
	private final GenericEntry sbLeftPos = compTab.addPersistent("Climber Pos", 0)
			.withWidget("Text View").withPosition(7, 4)
			.withSize(2, 2).getEntry();
	private final SimpleWidget sbMovingWidget = compTab.addPersistent("Climb Moving", "")
			.withWidget("Single Color View")
			.withPosition(7, 0)
			.withSize(2, 1);
	private final GenericEntry sbMoving = sbMovingWidget.getEntry();


	private final ShuffleboardLayout climberCommands = cmdTab
			.getLayout("Climber", BuiltInLayouts.kList)
			.withSize(3, 6)
			.withPosition(13, 1)
			.withProperties(Map.of("Label position", "Hidden"));


	/**************************************************************
	 * Constructor
	 **************************************************************/
	public Climber() {
		System.out.println("+++++ Starting Climber Constructor +++++");

		compTab.add("Climber Current", this)
				.withWidget("Subsystem")
				.withPosition(22, 10)
				.withSize(4, 2);

		// Configure Left Intake motor
		leftConfig
				.inverted(Constants.Climber.kLeftMotorInverted)
				.idleMode(Constants.Climber.kLeftIdleMode)
				.smartCurrentLimit(Constants.Climber.kLeftCurrentLimit);
		leftConfig.absoluteEncoder
				.zeroOffset(Constants.Climber.kLeftZeroOffset)
				.zeroCentered(Constants.Climber.kLeftZeroCentered)
				.inverted(Constants.Climber.kLeftEncoderInverted)
				.positionConversionFactor(Constants.Climber.kTiltPositionFactor)
				.velocityConversionFactor(Constants.Climber.kTiltVelocityFactor);
		leftConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
				.p(Constants.Climber.kPosP)
				.i(Constants.Climber.kPosI)
				.d(Constants.Climber.kPosD)
				.outputRange(Constants.Climber.kPosMinOutput, Constants.Climber.kPosMaxOutput)
				.positionWrappingEnabled(Constants.Climber.kLeftEncodeWrapping);
		// leftConfig.closedLoop.maxMotion
		// 		.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
		// 		.maxVelocity(Constants.Climber.kPosMaxVel)
		// 		.maxAcceleration(Constants.Climber.kPosMaxAccel)
		// 		.allowedClosedLoopError(Constants.Climber.kPosAllowedErr);

		leftClimber.configure(leftConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Right Intake motor
		rightConfig
				.follow(leftClimber, true)
				.inverted(Constants.Climber.kRightMotorInverted)
				.idleMode(Constants.Climber.kRightIdleMode)
				.smartCurrentLimit(Constants.Climber.kRightCurrentLimit);
		rightConfig.absoluteEncoder
				.zeroOffset(Constants.Climber.kRightZeroOffset)
				.zeroCentered(Constants.Climber.kRightZeroCentered)
				.inverted(Constants.Climber.kRightEncoderInverted)
				.positionConversionFactor(Constants.Climber.kTiltPositionFactor)
				.velocityConversionFactor(Constants.Climber.kTiltVelocityFactor);
		rightConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
				.p(Constants.Climber.kPosP)
				.i(Constants.Climber.kPosI)
				.d(Constants.Climber.kPosD)
				.outputRange(Constants.Climber.kPosMinOutput, Constants.Climber.kPosMaxOutput)
				.positionWrappingEnabled(Constants.Climber.kRightEncodeWrapping);
		rightConfig.closedLoop.maxMotion
				.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
				.maxVelocity(Constants.Climber.kPosMaxVel)
				.maxAcceleration(Constants.Climber.kPosMaxAccel)
				.allowedClosedLoopError(Constants.Climber.kPosAllowedErr);

		rightClimber.configure(rightConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		climberCommands.add("Climb", this.climb)
				.withProperties(Map.of("show type", false));
		climberCommands.add("Ready", this.ready)
				.withProperties(Map.of("show type", false));
		climberCommands.add("Zero", this.zero)
				.withProperties(Map.of("show type", false));
		climberCommands.add("Stow", this.stow)
				.withProperties(Map.of("show type", false));

		// Initialize intake start positions
		setClimberSP(climberSP);

		System.out.println("----- Ending Climber Constructor -----");
	}

	/**************************************************************
	 * Periodic
	 **************************************************************/
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		sbTxtSP.setString(getClimberSP().toString());
		sbDblSP.setDouble(getClimberSP().getValue());
		// sbLeftPos.setString(String.format("%.2f", getClimberPos()));
		sbLeftPos.setDouble(getClimberPos());

		sbOnTgt.setBoolean(onTarget());

		if (onTarget()) {
			sbMoving.setString(Constants.ColorConstants.OnTarget);
		} else {
			if (lib.isMoving(getClimberPos(), getClimberSP().getValue())) {
				sbMoving.setString(Constants.ColorConstants.Moving);
			} else {
				sbMoving.setString(Constants.ColorConstants.Stopped);
			}
		}

		// sbLimit.setBoolean(getLimitSwitch());
	}

	/**************************************************************
	 * Commands
	 **************************************************************/

	public Command stow = new InstantCommand(() -> setClimberPos(ClimberSP.STOW), this);
	public Command zero = new InstantCommand(() -> setClimberPos(ClimberSP.ZERO), this);
	public Command ready = new InstantCommand(() -> setClimberPos(ClimberSP.READY), this);
	public Command climb = new InstantCommand(() -> setClimberPos(ClimberSP.CLIMB), this);

	/**************************************************************
	 * Methods
	 **************************************************************/

	// private double sp = getClimberPos();
	// public void moveClimber(double pos) {
	// 	sp = sp + (pos / 2.5);
	// 	leftController.setReference(sp,
	// 			SparkBase.ControlType.kMAXMotionPositionControl);
	// }

	public double getClimberPos() {
		return leftEncoder.getPosition();
	}

	public double getClimberVel() {
		return leftEncoder.getVelocity();
	}

	public void setClimberPos(ClimberSP pos) {
		setClimberSP(pos);
		// leftController.setReference(pos.getValue(),
		// 		SparkBase.ControlType.kMAXMotionPositionControl);
		leftController.setReference(pos.getValue(),
				SparkBase.ControlType.kPosition);
	}

	public void setClimberPos() {
		setClimberPos(getClimberSP());
	}

	public boolean onTarget() {
		return Math.abs(getClimberPos() - getClimberSP().getValue()) < Constants.Climber.kTollerance;
	}

	public void setClimberSP(ClimberSP sp) {
		climberSP = sp;
	}

	public ClimberSP getClimberSP() {
		return climberSP;
	}

	public boolean getLimitSwitch() {
		return isLimitSwitch.isPressed();
	}
}
