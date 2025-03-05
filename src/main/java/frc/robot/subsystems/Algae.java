package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;

public class Algae extends SubsystemBase {

	// Define Intake Motors
	private final SparkMax leftIntake = new SparkMax(
			Constants.CANId.kAlgaeLeftIntakeCanId, MotorType.kBrushless);
	private final SparkMax rightIntake = new SparkMax(
			Constants.CANId.kAlgaeRightIntakeCanId, MotorType.kBrushless);
	private final SparkMax tilt = new SparkMax(
			Constants.CANId.kAlgaeTiltCanId, MotorType.kBrushless);

	private final SparkMaxConfig leftConfig = new SparkMaxConfig();
	private final SparkMaxConfig rightConfig = new SparkMaxConfig();
	private final SparkMaxConfig tiltConfig = new SparkMaxConfig();

	private SparkClosedLoopController tiltController = tilt.getClosedLoopController();
	private SparkClosedLoopController intakeController = leftIntake.getClosedLoopController();

	private RelativeEncoder intakeEncoder = leftIntake.getEncoder();
	private AbsoluteEncoder tiltEncoder = tilt.getAbsoluteEncoder();

	private SparkLimitSwitch isLimitSwitch = leftIntake.getForwardLimitSwitch();

	public enum AlgaeSP {
		STOW(109.0),
		ZERO(0.0),
		PROCESSOR(45.0),
		BARGE(-20.0),
		L2(-35.0),
		L3(-35.0),
		FLOOR(0.0);

		private final double sp;

		AlgaeSP(final double sp) {
			this.sp = sp;
		}

		public double getValue() {
			return sp;
		}
	}

	private Ladder ladder = null;

	private AlgaeSP tiltSP = AlgaeSP.ZERO;
	private double intakeSP = Constants.Algae.STOP;

	private boolean extractAlgae = false;

	// private boolean intakeActive = false;

	// private int intakeCntr = 0;

	/**************************************************************
	 * Initialize Shuffleboard entries
	 **************************************************************/
	private final ShuffleboardTab algaeTab = Shuffleboard.getTab("Algae");
	private final ShuffleboardTab cmdTab = Shuffleboard.getTab("Commands");

	private final GenericEntry sbIntakeVel = algaeTab.addPersistent("Intake Vel", 0)
			.withWidget("Text View").withPosition(2, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbIntakeSP = algaeTab.addPersistent("Intake SP", 0)
			.withWidget("Text View").withPosition(3, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbIntakeOnTgt = algaeTab.addPersistent("Intake On Tgt", false)
			.withWidget("Boolean Box").withPosition(4, 0).withSize(1, 1).getEntry();

	private final GenericEntry sbTxtTiltSP = algaeTab.addPersistent("Tilt tSP", "")
			.withWidget("Text View").withPosition(1, 1).withSize(1, 1).getEntry();
	private final GenericEntry sbDblTiltSP = algaeTab.addPersistent("Tilt dSP", 0)
			.withWidget("Text View").withPosition(2, 1).withSize(1, 1).getEntry();
	private final GenericEntry sbTiltPos = algaeTab.addPersistent("Tilt Pos", 0)
			.withWidget("Text View").withPosition(3, 1).withSize(1, 1).getEntry();
	private final GenericEntry sbTiltOnTgt = algaeTab.addPersistent("Tilt On Tgt", false)
			.withWidget("Boolean Box").withPosition(4, 1).withSize(1, 1).getEntry();

	private final GenericEntry sbLimit = algaeTab.addPersistent("Limit", false)
			.withWidget("Boolean Box").withPosition(6, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbExtract = algaeTab.addPersistent("Expel", false)
			.withWidget("Boolean Box").withPosition(6, 1).withSize(1, 1).getEntry();
	// private final GenericEntry sbEIntake = algaeTab.addPersistent("Intake",
	// false)
	// .withWidget("Boolean Box").withPosition(6, 2).withSize(1, 1).getEntry();
	// private final GenericEntry sbEIntakeCtr = algaeTab.addPersistent("Intake
	// Ctr", 0)
	// .withWidget("Text View").withPosition(7, 2).withSize(1, 1).getEntry();

	ShuffleboardLayout algaeCommands = cmdTab
				.getLayout("Algae Commands", BuiltInLayouts.kList)
				.withSize(2, 5)
				.withPosition(0, 1)
				.withProperties(Map.of("Label position", "HIDE"));

	/**************************************************************
	 * Constructor
	 **************************************************************/
	public Algae(Ladder ladder) {
		System.out.println("+++++ Starting Algae Constructor +++++");

		this.ladder = ladder;

		algaeTab.add("Algae", this);

		// Configure Left Intake motor
		leftConfig
				.inverted(Constants.Algae.kLeftMotorInverted)
				.idleMode(Constants.Algae.kLeftIdleMode)
				.smartCurrentLimit(Constants.Algae.kLeftCurrentLimit);
		leftConfig.encoder
				.positionConversionFactor(Constants.Algae.kIntakePositionFactor)
				.velocityConversionFactor(Constants.Algae.kIntakeVelocityFactor);
		leftConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Algae.kIntakeVelP)
				.i(Constants.Algae.kIntakeVelI)
				.d(Constants.Algae.kIntakeVelD)
				.velocityFF(Constants.Algae.kIntakeVelFF)
				.outputRange(Constants.Algae.kIntakeVelMinOutput, Constants.Algae.kIntakeVelMaxOutput)
				.positionWrappingEnabled(Constants.Algae.kIntakeEncodeWrapping);
		leftConfig.closedLoop.maxMotion
				.maxVelocity(Constants.Algae.kIntakeVelMaxVel)
				.maxAcceleration(Constants.Algae.kIntakeVelMaxAccel)
				.allowedClosedLoopError(Constants.Algae.kIntakeVelAllowedErr);

		leftIntake.configure(leftConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Right Intake motor
		rightConfig
				.follow(leftIntake, true)
				.inverted(Constants.Algae.kRightMotorInverted)
				.idleMode(Constants.Algae.kRightIdleMode)
				.smartCurrentLimit(Constants.Algae.kRightCurrentLimit);

		rightIntake.configure(rightConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Tilt motor
		tiltConfig
				.inverted(Constants.Algae.kTiltMotorInverted)
				.idleMode(Constants.Algae.kTiltIdleMode)
				.smartCurrentLimit(Constants.Algae.kTiltCurrentLimit);
		tiltConfig.absoluteEncoder
				.zeroOffset(Constants.Algae.kTiltZeroOffset)
				.zeroCentered(Constants.Algae.kTiltZeroCentered)
				.inverted(Constants.Algae.kTiltEncoderInverted)
				.positionConversionFactor(Constants.Algae.kTiltPositionFactor)
				.velocityConversionFactor(Constants.Algae.kTiltVelocityFactor);
		tiltConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
				.p(Constants.Algae.kTiltPosP)
				.i(Constants.Algae.kTiltPosI)
				.d(Constants.Algae.kTiltPosD)
				.outputRange(Constants.Algae.kTiltPosMinOutput, Constants.Algae.kTiltPosMaxOutput)
				.positionWrappingEnabled(Constants.Algae.kIntakeEncodeWrapping);
		tiltConfig.closedLoop.maxMotion
				.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
				.maxVelocity(Constants.Algae.kTiltPosMaxVel)
				.maxAcceleration(Constants.Algae.kTiltPosMaxAccel)
				.allowedClosedLoopError(Constants.Algae.kTiltPosAllowedErr);

		tilt.configure(tiltConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		algaeCommands.add("Barge", this.barge);
		algaeCommands.add("L3", this.l3);
		algaeCommands.add("L2", this.l2);
		algaeCommands.add("Processor", this.processor);
		algaeCommands.add("Floor", this.floor);
		algaeCommands.add("Stow", this.stow);
		algaeCommands.add("Intake", this.intake);
		algaeCommands.add("Eject", this.eject);

		setIntakeVel(intakeSP);
		setTiltPos(tiltSP);

		System.out.println("----- Ending Algae Constructor -----");
	}

	/**************************************************************
	 * Periodic
	 **************************************************************/
	@Override
	public void periodic() {
		// This method will be called once per scheduler run SuffleBoard
		// setTiltSP(sbTiltSP.getDouble(0.0));
		// setIntakeSP(sbIntakeSP.getDouble(0.0));

		sbIntakeVel.setDouble(getIntakeVel());
		sbIntakeSP.setDouble(getIntakeSP());
		sbTiltPos.setDouble(getTiltPos());
		sbTxtTiltSP.setString(getTiltSP().toString());
		sbDblTiltSP.setDouble(getTiltSP().getValue());

		sbTiltOnTgt.setBoolean(onTiltTarget());
		sbIntakeOnTgt.setBoolean(onIntakeTarget());

		sbLimit.setBoolean(isLimit());
		sbExtract.setBoolean(getExtract());
		// sbEIntake.setBoolean(intakeActive);
		// sbEIntakeCtr.setDouble(intakeCntr);

		// if (intakeActive) {
		// if (((int) getIntakeSP() == (int) Constants.Algae.INTAKE) &&
		// (isLimit())) {
		// setIntakeVel(Constants.Algae.STOP);
		// intakeActive = false;

		// } else if (((int) getIntakeSP() == (int) Constants.Algae.EJECT) &&
		// (intakeCntr++ > 25)) {
		// setIntakeVel(Constants.Algae.STOP);
		// intakeActive = false;

		// } else {
		// DriverStation.reportWarning("Algae Intake Active but not commanded.", false);
		// intakeActive = false;
		// }
		// }
	}

	/**************************************************************
	 * Commands
	 **************************************************************/
	public Command barge = new InstantCommand(() -> setTiltPos(AlgaeSP.BARGE));
	public Command l3 = new InstantCommand(() -> setTiltPos(AlgaeSP.L3));
	public Command l2 = new InstantCommand(() -> setTiltPos(AlgaeSP.L2));
	public Command processor = new InstantCommand(() -> setTiltPos(AlgaeSP.PROCESSOR));
	public Command floor = new InstantCommand(() -> setTiltPos(AlgaeSP.FLOOR));
	public Command zero = new InstantCommand(() -> setTiltPos(AlgaeSP.ZERO));
	public Command stow = new InstantCommand(() -> setTiltPos(AlgaeSP.STOW));

	public Command toggleExtract = new InstantCommand(() -> toggleExtract());

	public Command intake = new InstantCommand(() -> setIntakeVel(Constants.Algae.INTAKE))
			.until(() -> isLimit())
			// .andThen(() -> setTiltPos(AlgaeSP.PROCESSOR))
			.andThen(() -> setIntakeVel(Constants.Algae.HOLD));
	public Command eject = new InstantCommand(() -> setIntakeVel(Constants.Algae.EJECT))
			.andThen(new WaitCommand(0.5))
			.andThen(() -> setIntakeVel(Constants.Algae.STOP));

	/**************************************************************
	 * Methods
	 **************************************************************/

	// public void moveTilt(double pos) {
	// tiltController.setReference(getTiltPos() + pos,
	// SparkBase.ControlType.kMAXMotionPositionControl);
	// }

	// Getting the position of the encoders
	public double getIntakeVel() {
		return intakeEncoder.getVelocity();
	}

	public double getTiltPos() {
		return tiltEncoder.getPosition();
	}

	// Sets the position of the encoders
	public void setTiltPos(AlgaeSP pos) {
		setTiltSP(pos);
		tiltController.setReference(pos.getValue(), SparkBase.ControlType.kMAXMotionPositionControl);
	}

	// Sets the position of the encoders
	public void setTiltPos() {
		setTiltPos(getTiltSP());
	}

	public void setIntakeVel(double vel) {
		// intakeActive = true;
		// intakeCntr = 0;
		setIntakeSP(vel);
		intakeController.setReference(vel, SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public void setIntakeVel() {
		setIntakeVel(getIntakeSP());
	}

	public boolean onTiltTarget() {
		return Math.abs(getTiltPos() - getTiltSP().getValue()) < Constants.Algae.kTiltTollerance;
	}

	public boolean onIntakeTarget() {
		return Math.abs(getIntakeVel() - getIntakeSP()) < Constants.Algae.kIntakeTollerance;
	}

	public void setIntakeSP(double sp) {
		intakeSP = sp;
	}

	public double getIntakeSP() {
		return intakeSP;
	}

	public void setTiltSP(AlgaeSP sp) {
		tiltSP = sp;
	}

	public AlgaeSP getTiltSP() {
		return tiltSP;
	}

	public boolean isLimit() {
		return isLimitSwitch.isPressed(); // || rightForLimitSwitch.isPressed();
	}

	public void toggleExtract() {
		extractAlgae = !extractAlgae;
	}

	public boolean getExtract() {
		return extractAlgae;
	}

	/**
	 * doAction - performs Algea action based on Ladder position
	 */
	public void doAction() {
		double vel = 0.0;

		switch (ladder.getLadderSP()) {
			case BARGE:
			case PROCESSOR:
				vel = Constants.Algae.EJECT;
				break;
			case L3:
			case L2:
				if (extractAlgae) {
					vel = Constants.Algae.INTAKE;
				}
				break;
			case FLOOR:
				vel = Constants.Algae.INTAKE;
				break;
			default:
		}

		setIntakeVel(vel);
	}
}