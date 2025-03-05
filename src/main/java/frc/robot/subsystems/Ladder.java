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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

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

	private SparkLimitSwitch limitSwitch = leftLadder.getForwardLimitSwitch(); // leftLadder.getReverseLimitSwitch();

	// define ladder positions
	public enum LadderSP {
		BARGE((-8.0 * 12.0) - 5.0),
		L4(-6.0 * 12.0),
		L3((-3.0 * 12.0) - 11.625),
		L2((-2.0 * 12.0) - 7.825),
		L1((-1.0 * 12.0) - 2.0),
		STATION((-3.0 * 12.0) - 1.5),
		PROCESSOR(0.0 - 7.0),
		FLOOR(-0.5 * 12.0),
		STOW(-0.5 * 12.0),
		START(0.0);

		private final double sp;

		LadderSP(final double sp) {
			this.sp = sp;
		}

		public double getValue() {
			return sp;
		}
	}

	private LadderSP ladderSP = LadderSP.START;

	private boolean firstPeriod = true;
	private boolean zeroingLadder = false;

	/**************************************************************
	 * Initialize Shuffleboard entries
	 **************************************************************/
	private final ShuffleboardTab ladderTab = Shuffleboard.getTab("Ladder");
	private final ShuffleboardTab cmdTab = Shuffleboard.getTab("Commands");

	private final GenericEntry sbTxtSP = ladderTab.addPersistent("Ladder tSP", "")
			.withWidget("Text View").withPosition(1, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbDblSP = ladderTab.addPersistent("Ladder dSP", 0)
			.withWidget("Text View").withPosition(2, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbLadderPos = ladderTab.addPersistent("Ladder Pos", 0)
			.withWidget("Text View").withPosition(3, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbOnTgt = ladderTab.addPersistent("On Tgt", false)
			.withWidget("Boolean Box").withPosition(4, 0)
			.withSize(1, 1).getEntry();

	private final GenericEntry sbLimit = ladderTab.addPersistent("Ladder Limit", false)
			.withWidget("Boolean Box").withPosition(6, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbFirst = ladderTab.addPersistent("First", false)
			.withWidget("Boolean Box").withPosition(6, 1)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbZeroing = ladderTab.addPersistent("Zeroing", false)
			.withWidget("Boolean Box").withPosition(6, 2)
			.withSize(1, 1).getEntry();

	ShuffleboardLayout ladderCommands = cmdTab
			.getLayout("Ladder", BuiltInLayouts.kList)
			.withSize(2, 5)
			.withPosition(6, 1)
			.withProperties(Map.of("Label position", "HIDE"));

	/**************************************************************
	 * Constructor
	 **************************************************************/
	public Ladder() {
		System.out.println("+++++ Starting Ladder Constructor +++++");

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

		ladderCommands.add("Barge", this.barge);
		ladderCommands.add("L4", this.l4);
		ladderCommands.add("L3", this.l3);
		ladderCommands.add("L2", this.l2);
		ladderCommands.add("L1", this.l1);
		ladderCommands.add("Station", this.station);
		ladderCommands.add("Processor", this.processor);
		ladderCommands.add("Floor", this.floor);
		ladderCommands.add("Stow", this.stow);

		// Initialize intake start positions
		leftEncoder.setPosition(ladderSP.getValue());
		setLadderPos(ladderSP);

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
	}

	/**************************************************************
	 * Commands
	 **************************************************************/
	public Command barge = new InstantCommand(() -> setLadderPos(LadderSP.BARGE));
	public Command l4 = new InstantCommand(() -> setLadderPos(LadderSP.L3));
	public Command l3 = new InstantCommand(() -> setLadderPos(LadderSP.L2));
	public Command l2 = new InstantCommand(() -> setLadderPos(LadderSP.L3));
	public Command l1 = new InstantCommand(() -> setLadderPos(LadderSP.L2));
	public Command station = new InstantCommand(() -> setLadderPos(LadderSP.PROCESSOR));
	public Command processor = new InstantCommand(() -> setLadderPos(LadderSP.PROCESSOR));
	public Command floor = new InstantCommand(() -> setLadderPos(LadderSP.FLOOR));
	public Command stow = new InstantCommand(() -> setLadderPos(LadderSP.STOW));

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
		}
	}

	// /**
	//  * setTiltCmd - command factory method to update the Tilt pos
	//  * 
	//  * @return a command
	//  */
	// public Command setLadderSPCmd(LadderSP sp) {
	// 	// Subsystem::RunOnce implicitly requires `this` subsystem.
	// 	return runOnce(() -> {
	// 		setLadderSP(sp);
	// 	});
	// }

	// /**
	//  * setTiltCmd - command factory method to update the Tilt pos
	//  * 
	//  * @return a command
	//  */
	// public Command setLadderPosCmd(LadderSP pos) {
	// 	// Subsystem::RunOnce implicitly requires `this` subsystem.
	// 	return runOnce(() -> {
	// 		setLadderPos(pos);
	// 	});
	// }

	// public Command setLadderPosCmd() {
	// 	return setLadderPosCmd(getLadderSP());
	// }

	public boolean onTarget() {
		return Math.abs(getLeftPos() - getLadderSP().getValue()) < Constants.Ladder.kTollerance;
	}

	public double getLeftPos() {
		return leftEncoder.getPosition();
	}

	public void setLadderPos(LadderSP pos) {
		setLadderSP(pos);
		leftController.setReference(pos.getValue(),
				SparkBase.ControlType.kMAXMotionPositionControl);
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
		return limitSwitch.isPressed(); // || rightForLimitSwitch.isPressed();
	}

	// public Command ladderBarge() {
	// 	return this.runOnce(() -> setLadderPos(Ladder.LadderSP.BARGE));
	// }

	// public Command ladderL4() {
	// 	return this.runOnce(() -> setLadderPos(Ladder.LadderSP.L4));
	// }

	// public Command ladderL3() {
	// 	return this.runOnce(() -> setLadderPos(Ladder.LadderSP.L3));
	// }

	// public Command ladderL2() {
	// 	return this.runOnce(() -> setLadderPos(Ladder.LadderSP.L2));
	// }

	// public Command ladderL1() {
	// 	return this.runOnce(() -> setLadderPos(Ladder.LadderSP.L1));
	// }

	// public Command ladderStation() {
	// 	return this.runOnce(() -> setLadderPos(Ladder.LadderSP.STATION));
	// }

	// public Command ladderProcessor() {
	// 	return this.runOnce(() -> setLadderPos(Ladder.LadderSP.PROCESSOR));
	// }

	// public Command ladderFloor() {
	// 	return this.runOnce(() -> setLadderPos(Ladder.LadderSP.FLOOR));
	// }

	// public Command ladderStow() {
	// 	return this.runOnce(() -> setLadderPos(Ladder.LadderSP.STOW));
	// }

}
