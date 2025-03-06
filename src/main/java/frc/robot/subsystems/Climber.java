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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

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
		CLIMB(-14.0); // degrees - full climb

		private final double sp;

		ClimberSP(final double sp) {
			this.sp = sp;
		}

		public double getValue() {
			return sp;
		}
	}

	private ClimberSP climberSP = Climber.ClimberSP.STOW;

	/**************************************************************
	 * Initialize Shuffleboard entries
	 **************************************************************/
	private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
	private final ShuffleboardTab cmdTab = Shuffleboard.getTab("Commands");
	private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

	private final GenericEntry sbTxtSP = climberTab.addPersistent("Climber tSP", "")
			.withWidget("Text View").withPosition(1, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbDblSP = climberTab.addPersistent("Climber dSP", 0)
			.withWidget("Text View").withPosition(2, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbLeftPos = climberTab.addPersistent("Climber Pos", 0)
			.withWidget("Text View").withPosition(3, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbOnTgt = climberTab.addPersistent("On Tgt", false)
			.withWidget("Boolean Box").withPosition(4, 0).withSize(1, 1).getEntry();

	private final GenericEntry sbLimit = climberTab.addPersistent("Climber Limit", false)
			.withWidget("Boolean Box").withPosition(6, 0)
			.withSize(1, 1).getEntry();

	private final ShuffleboardLayout climberCommands = cmdTab
			.getLayout("Climber", BuiltInLayouts.kList)
			.withSize(2, 2)
			.withPosition(4, 1)
			.withProperties(Map.of("Label position", "Hidden"));

			private final ShuffleboardLayout climberData = compTab
			.getLayout("Climber", BuiltInLayouts.kList)
			.withSize(2, 5)
			.withPosition(5, 1)
			.withProperties(Map.of("Label position", "Top"));

	/**************************************************************
	 * Constructor
	 **************************************************************/
	public Climber() {
		System.out.println("+++++ Starting Climber Constructor +++++");

		cmdTab.add("Climber", this)
				.withWidget("Subsystem")
				.withPosition(9, 4)
				.withSize(2, 1);

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
		leftConfig.closedLoop.maxMotion
				.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
				.maxVelocity(Constants.Climber.kPosMaxVel)
				.maxAcceleration(Constants.Climber.kPosMaxAccel)
				.allowedClosedLoopError(Constants.Climber.kPosAllowedErr);

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

		climberCommands.add("Climb", this.climb);
		climberCommands.add("Ready", this.ready);
		climberCommands.add("Zero", this.zero);
		climberCommands.add("Stow", this.stow);

		climberData.add("Txt SP", this.climberSP.toString());
		climberData.add("Dbl SP", this.climberSP.getValue());
		climberData.add("Position", this.getClimberPos());
		climberData.add("On Target", this.onTarget());

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
		sbLeftPos.setDouble(getClimberPos());
		sbTxtSP.setString(getClimberSP().toString());
		sbDblSP.setDouble(getClimberSP().getValue());

		sbOnTgt.setBoolean(onTarget());

		sbLimit.setBoolean(getLimitSwitch());
	}

	/**************************************************************
	 * Commands
	 **************************************************************/

	public Command stow = new InstantCommand(() -> setClimberPos(ClimberSP.STOW));
	public Command zero = new InstantCommand(() -> setClimberPos(ClimberSP.ZERO));
	public Command ready = new InstantCommand(() -> setClimberPos(ClimberSP.READY));
	public Command climb = new InstantCommand(() -> setClimberPos(ClimberSP.CLIMB));

	/**************************************************************
	 * Methods
	 **************************************************************/
	private double sp = getClimberPos();

	public void moveClimber(double pos) {
		sp = sp + (pos / 2.5);
		leftController.setReference(sp,
				SparkBase.ControlType.kMAXMotionPositionControl);
	}

	public double getClimberPos() {
		return leftEncoder.getPosition();
	}

	public void setClimberPos(ClimberSP pos) {
		setClimberSP(pos);
		leftController.setReference(pos.getValue(),
				SparkBase.ControlType.kMAXMotionPositionControl);
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
