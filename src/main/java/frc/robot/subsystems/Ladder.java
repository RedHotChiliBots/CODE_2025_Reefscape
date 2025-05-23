package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
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

public class Ladder extends SubsystemBase {

	// Define Intake Motors
	private final SparkMax leftLadder = new SparkMax(
			Constants.CANId.kLadderLeftCanId, MotorType.kBrushless);
	private final SparkMax rightLadder = new SparkMax(
			Constants.CANId.kLadderRightCanId, MotorType.kBrushless);

	private final SparkMaxConfig leftConfig = new SparkMaxConfig();
	private final SparkMaxConfig rightConfig = new SparkMaxConfig();

	private SparkClosedLoopController leftController = leftLadder.getClosedLoopController();

	private RelativeEncoder leftEncoder = leftLadder.getEncoder();

	private SparkLimitSwitch limitSwitch = leftLadder.getForwardLimitSwitch();

	// define ladder positions
	public enum LadderSP {
		BARGE((-8.0 * 12.0) + 1.0), // 93.5" Max Height for Ladder
		L4((-6.0 * 12.0) + 3.5),
		L35((-5.0 * 12.0) - 3.375 + 2.5),
		L3((-3.0 * 12.0) - 11.625 + 6.5),
		L2((-2.0 * 12.0) - 7.825 + 7.5),
		L1((-1.0 * 12.0) - 6.0 - 1.5),
		STATION((-3.0 * 12.0) - 1.5 + 18.0),
		PROCESSOR(0.0 - 9.0 - 3.0),
		FLOOR((-0.25 * 12.0)),
		STOW(-0.5 * 12.0),
		START(0.0);

		private double sp;

		LadderSP(double sp) {
			this.sp = sp;
		}

		public double getValue() {
			return sp;
		}
	}

	private LadderSP ladderSP = LadderSP.START;

	private boolean firstPeriod = true;
	private boolean zeroingLadder = false;
	private boolean ladderZeroed = false;

	private Library lib = new Library();

	/**************************************************************
	 * Initialize Shuffleboard entries
	 **************************************************************/
	private final ShuffleboardTab cmdTab = Shuffleboard.getTab("Commands");
	private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

	private final GenericEntry sbOnTgt = compTab.addPersistent("Ladder OnTgt", false)
			.withWidget("Boolean Box").withPosition(13, 1)
			.withSize(2, 1).getEntry();
	private final GenericEntry sbTxtSP = compTab.addPersistent("Ladder SP", "")
			.withWidget("Text View").withPosition(13, 2)
			.withSize(2, 1).getEntry();
	private final GenericEntry sbDblSP = compTab.addPersistent("Ladder SP (deg)", 0)
			.withWidget("Text View").withPosition(13, 3)
			.withSize(2, 1).getEntry();
	private final GenericEntry sbLadderPos = compTab.addPersistent("Ladder Pos", 0)
			.withWidget("Text View").withPosition(13, 4)
			.withSize(2, 2).getEntry();

	private final GenericEntry sbLimit = compTab.addPersistent("Ladder Limit", false)
			.withWidget("Boolean Box").withPosition(13, 6)
			.withSize(2, 1).getEntry();
	private final GenericEntry sbFirst = compTab.addPersistent("First", false)
			.withWidget("Boolean Box").withPosition(13, 7)
			.withSize(2, 1).getEntry();
	private final GenericEntry sbZeroing = compTab.addPersistent("Zeroing", false)
			.withWidget("Boolean Box").withPosition(13, 8)
			.withSize(2, 1).getEntry();

	private final SimpleWidget sbMovingWidget = compTab.addPersistent("Ladder Moving", "")
			.withWidget("Single Color View")
			.withPosition(13, 0)
			.withSize(2, 1);
	private final GenericEntry sbMoving = sbMovingWidget.getEntry();

	private final ShuffleboardLayout ladderCommands = cmdTab
			.getLayout("Ladder", BuiltInLayouts.kList)
			.withSize(3, 12)
			.withPosition(9, 1)
			.withProperties(Map.of("Label position", "Hidden"));

	/**************************************************************
	 * Constructor
	 **************************************************************/
	public Ladder() {
		System.out.println("+++++ Starting Ladder Constructor +++++");

		compTab.add("Ladder Current", this)
				.withWidget("Subsystem")
				.withPosition(22, 8)
				.withSize(4, 2);

		// Configure Left Intake motor
		leftConfig
				.inverted(Constants.Ladder.kLeftMotorInverted)
				.idleMode(Constants.Ladder.kLeftIdleMode)
				.smartCurrentLimit(Constants.Ladder.kLeftCurrentLimit);
		leftConfig.encoder
				.positionConversionFactor(Constants.Ladder.kLiftPostionFactor)
				.velocityConversionFactor(Constants.Ladder.kLiftVelocityFactor);
		leftConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Ladder.kPosP)
				.i(Constants.Ladder.kPosI)
				.d(Constants.Ladder.kPosD)
				.outputRange(Constants.Ladder.kPosMinOutput, Constants.Ladder.kPosMaxOutput)
				.positionWrappingEnabled(Constants.Ladder.kLeftEncodeWrapping);
		leftConfig.closedLoop.maxMotion
				.maxVelocity(Constants.Ladder.kMaxVel)
				.maxAcceleration(Constants.Ladder.kMaxAccel)
				.allowedClosedLoopError(Constants.Ladder.kAllowedErr);

		leftLadder.configure(leftConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Right Intake motor
		rightConfig
				.follow(leftLadder, true)
				.inverted(Constants.Ladder.kRightMotorInverted)
				.idleMode(Constants.Ladder.kRightIdleMode)
				.smartCurrentLimit(Constants.Ladder.kRightCurrentLimit);

		rightLadder.configure(rightConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		ladderCommands.add("Barge", this.barge)
				.withProperties(Map.of("show type", false));
		ladderCommands.add("L4", this.l4)
				.withProperties(Map.of("show type", false));
		ladderCommands.add("L35", this.l35)
				.withProperties(Map.of("show type", false));
		ladderCommands.add("L3", this.l3)
				.withProperties(Map.of("show type", false));
		ladderCommands.add("L2", this.l2)
				.withProperties(Map.of("show type", false));
		ladderCommands.add("L1", this.l1)
				.withProperties(Map.of("show type", false));
		ladderCommands.add("Station", this.station)
				.withProperties(Map.of("show type", false));
		ladderCommands.add("Processor", this.processor)
				.withProperties(Map.of("show type", false));
		ladderCommands.add("Floor", this.floor)
				.withProperties(Map.of("show type", false));
		ladderCommands.add("Stow", this.stow)
				.withProperties(Map.of("show type", false));

		// Initialize intake start positions
		// leftEncoder.setPosition(ladderSP.getValue());
		// setLadderPos(ladderSP);

		System.out.println("----- Ending Ladder Constructor -----");
	}

	/**************************************************************
	 * Periodic
	 **************************************************************/
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		sbTxtSP.setString(getLadderSP().toString());
		sbDblSP.setDouble(getLadderSP().getValue());
		sbLadderPos.setDouble(getLeftPos());

		sbOnTgt.setBoolean(onTarget());

		sbLimit.setBoolean(isLimit());
		sbFirst.setBoolean(firstPeriod);
		sbZeroing.setBoolean(zeroingLadder);

		if (firstPeriod || zeroingLadder) {
			zeroLadder();
		}

		if (onTarget()) {
			sbMoving.setString(Constants.ColorConstants.OnTarget);
		} else {
			if (lib.isMoving(getLeftPos(), getLadderSP().getValue())) {
				sbMoving.setString(Constants.ColorConstants.Moving);
			} else {
				sbMoving.setString(Constants.ColorConstants.Stopped);
			}
		}
	}

	/**************************************************************
	 * Commands
	 **************************************************************/
	public Command barge = new InstantCommand(() -> setLadderPos(LadderSP.BARGE), this);
	public Command l4 = new InstantCommand(() -> setLadderPos(LadderSP.L4), this);
	public Command l35 = new InstantCommand(() -> setLadderPos(LadderSP.L35), this);
	public Command l3 = new InstantCommand(() -> setLadderPos(LadderSP.L3), this);
	public Command l2 = new InstantCommand(() -> setLadderPos(LadderSP.L2), this);
	public Command l1 = new InstantCommand(() -> setLadderPos(LadderSP.L1), this);
	public Command station = new InstantCommand(() -> setLadderPos(LadderSP.STATION), this);
	public Command processor = new InstantCommand(() -> setLadderPos(LadderSP.PROCESSOR), this);
	public Command floor = new InstantCommand(() -> setLadderPos(LadderSP.FLOOR), this);
	public Command stow = new InstantCommand(() -> setLadderPos(LadderSP.STOW), this);
	// new InstantCommand(() -> setZeroingLadder())
	// 		.until(() -> isLadderZeroed())
	// 		.andThen(new InstantCommand(() -> setLadderPos(LadderSP.STOW), this));

	/**************************************************************
	 * Methods
	 **************************************************************/

	private void zeroLadder() {
		if (firstPeriod) {
			leftLadder.set(Constants.Ladder.DOWN);
			firstPeriod = false;
			zeroingLadder = true;
		}

		if (isLimit()) {
			leftLadder.set(Constants.Ladder.STOP);
			leftEncoder.setPosition(LadderSP.START.getValue());
			setLadderPos(LadderSP.STOW);
			zeroingLadder = false;
			ladderZeroed = true;
		}
	}

	public boolean isLadderZeroed() {
		return ladderZeroed;
	}

	public void setZeroingLadder() {
		zeroingLadder = true;
	}

	public boolean onTarget() {
		return Math.abs(getLeftPos() - getLadderSP().getValue()) < Constants.Ladder.kTollerance;
	}

	public double getLeftPos() {
		return leftEncoder.getPosition();
	}

	public void setLadderPos(LadderSP sp) {
		setLadderSP(sp);
		leftController.setReference(sp.getValue(),
		SparkBase.ControlType.kMAXMotionPositionControl);
		// leftController.setReference(sp.getValue(),
		// 		SparkBase.ControlType.kPosition);
	}

	public void setLadderPos() {
		setLadderPos(getLadderSP());
	}

	public void setLadderSP(LadderSP sp) {
		ladderSP = sp;
	}

	public LadderSP getLadderSP() {
		return ladderSP;
	}

	public boolean isLimit() {
		return limitSwitch.isPressed();
	}
}
