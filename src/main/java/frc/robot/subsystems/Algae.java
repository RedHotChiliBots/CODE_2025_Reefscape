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
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.AlgaeEject;
import frc.robot.commands.AlgaeIntake;
import frc.robot.subsystems.Ladder.LadderSP;
import frc.robot.utils.Library;

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

	private SparkLimitSwitch limitSwitch = leftIntake.getForwardLimitSwitch();

	public enum IntakeEject {
		INTAKE,
		EJECT,
		STOP;
	}

	public enum AlgaeSP {
		STOWUP(80.0, Constants.Algae.STOP),
		STOWDN(-75.0, Constants.Algae.STOP),
		ZERO(0.0, Constants.Algae.STOP),
		PROCESSOR(10.0, Constants.Algae.EJECT),
		BARGE(30.0, Constants.Algae.EJECT),
		L2(-75.0, Constants.Algae.STOP),
		L3(-35.0, Constants.Algae.INTAKE),
		L35(-35.0, Constants.Algae.INTAKE),
		L4(-35.0, Constants.Algae.INTAKE),
		FLOOR(0.0, Constants.Algae.INTAKE);

		private double tilt;
		private double intake;

		AlgaeSP(double tilt, double intake) {
			this.tilt = tilt;
			this.intake = intake;
		}

		public double getTilt() {
			return tilt;
		}

		public double getIntake() {
			return intake;
		}
	}

	IntakeEject intakeEject = IntakeEject.STOP;

	private Ladder ladder = null;

	private AlgaeSP algaeSP = AlgaeSP.ZERO;

	private boolean extractAlgae = false;

	private Library lib = new Library();

	/**************************************************************
	 * Initialize Shuffleboard entries
	 **************************************************************/
	private final ShuffleboardTab cmdTab = Shuffleboard.getTab("Commands");
	private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

	private final GenericEntry sbTiltOnTgt = compTab.addPersistent("Algae OnTgt", false)
			.withWidget("Boolean Box").withPosition(9, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbTxtTiltSP = compTab.addPersistent("Algae Tilt SP", "")
			.withWidget("Text View").withPosition(9, 2).withSize(2, 1).getEntry();
	private final GenericEntry sbDblTiltSP = compTab.addPersistent("Algae Tilt SP (deg)", 0)
			.withWidget("Text View").withPosition(9, 3).withSize(2, 1).getEntry();
	private final GenericEntry sbTiltPos = compTab.addPersistent("Algae Tilt Pos (deg)", 0)
			.withWidget("Text View").withPosition(9, 4).withSize(2, 2).getEntry();

	private final GenericEntry sbIntakeOnTgt = compTab.addPersistent("Algae Intake OnTgt", false)
			.withWidget("Boolean Box").withPosition(9, 6).withSize(2, 1).getEntry();
	private final GenericEntry sbIntakeSP = compTab.addPersistent("Algae Intake SP", 0)
			.withWidget("Text View").withPosition(9, 7).withSize(2, 1).getEntry();
	private final GenericEntry sbIntakeVel = compTab.addPersistent("Algae Intake (vel))", 0)
			.withWidget("Text View").withPosition(9, 8).withSize(2, 1).getEntry();

	private final GenericEntry sbExtract = compTab.addPersistent("Extract", false)
			.withWidget("Boolean Box").withPosition(9, 10).withSize(2, 1).getEntry();
	private final GenericEntry sbLimit = compTab.addPersistent("Limit", false)
			.withWidget("Boolean Box").withPosition(9, 11).withSize(2, 1).getEntry();

	private final GenericEntry sbInEj = compTab.addPersistent("Algae In-Ej", "")
			.withWidget("Text View").withPosition(11, 14).withSize(2, 1).getEntry();

	private final SimpleWidget sbMovingWidget = compTab.addPersistent("Algae Moving", "")
			.withWidget("Single Color View")
			.withPosition(9, 0)
			.withSize(2, 1);
	private final GenericEntry sbMoving = sbMovingWidget.getEntry();

	private final ShuffleboardLayout algaeCommands = cmdTab
			.getLayout("Algae", BuiltInLayouts.kList)
			.withSize(3, 12)
			.withPosition(0, 1)
			.withProperties(Map.of("Label position", "Hidden"));

	/**************************************************************
	 * Constructor
	 **************************************************************/
	public Algae(Ladder ladder) {
		System.out.println("+++++ Starting Algae Constructor +++++");

		this.ladder = ladder;

		compTab.add("Algae Current", this)
				.withWidget("Subsystem")
				.withPosition(22, 6)
				.withSize(4, 2);

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

		algaeCommands.add("Barge", this.barge)
				.withProperties(Map.of("show type", false));
		algaeCommands.add("L35", this.l35)
				.withProperties(Map.of("show type", false));
		algaeCommands.add("L3", this.l3)
				.withProperties(Map.of("show type", false));
		algaeCommands.add("L2", this.l2)
				.withProperties(Map.of("show type", false));
		algaeCommands.add("Processor", this.processor)
				.withProperties(Map.of("show type", false));
		algaeCommands.add("Floor", this.floor)
				.withProperties(Map.of("show type", false));
		algaeCommands.add("StowUP", this.stowup)
				.withProperties(Map.of("show type", false));
		algaeCommands.add("StowDN", this.stowdn)
				.withProperties(Map.of("show type", false));
		algaeCommands.add("Intake", this.intake)
				.withProperties(Map.of("show type", false));
		algaeCommands.add("Eject", this.eject)
				.withProperties(Map.of("show type", false));
		algaeCommands.add("Zero", this.zero)
				.withProperties(Map.of("show type", false));

		setAlgaeSP(AlgaeSP.STOWUP);
		setIntakeVel(getAlgaeSP());
		setTiltPos(getAlgaeSP());

		System.out.println("----- Ending Algae Constructor -----");
	}

	/**************************************************************
	 * Periodic
	 **************************************************************/
	@Override
	public void periodic() {

		sbIntakeVel.setDouble(getIntakeVel());
		sbIntakeSP.setDouble(getIntakeSP());
		sbTiltPos.setDouble(getTiltPos());
		sbTxtTiltSP.setString(getAlgaeSP().toString());
		sbDblTiltSP.setDouble(getTiltSP());

		sbTiltOnTgt.setBoolean(onTiltTarget());
		sbIntakeOnTgt.setBoolean(onIntakeTarget());

		sbLimit.setBoolean(isLimit());
		sbExtract.setBoolean(getExtract());

		sbInEj.setString(getIntakeEject().toString());

		if (onTiltTarget()) {
			sbMoving.setString(Constants.ColorConstants.OnTarget);
		} else {
			if (lib.isMoving(getTiltPos(), getTiltSP())) {
				sbMoving.setString(Constants.ColorConstants.Moving);
			} else {
				sbMoving.setString(Constants.ColorConstants.Stopped);
			}
		}
	}

	/**************************************************************
	 * Commands
	 **************************************************************/
	public Command barge = new InstantCommand(() -> setTiltPos(AlgaeSP.BARGE), this);
	public Command l35 = new InstantCommand(() -> setTiltPos(AlgaeSP.L35), this);
	public Command l3 = new InstantCommand(() -> setTiltPos(AlgaeSP.L3), this);
	public Command l2 = new InstantCommand(() -> setTiltPos(AlgaeSP.L2), this);
	public Command processor = new InstantCommand(() -> setTiltPos(AlgaeSP.PROCESSOR), this);
	public Command floor = new InstantCommand(() -> setTiltPos(AlgaeSP.FLOOR), this);
	public Command zero = new InstantCommand(() -> setTiltPos(AlgaeSP.ZERO), this);
	public Command stowup = new InstantCommand(() -> setTiltPos(AlgaeSP.STOWUP), this);
	public Command stowdn = new InstantCommand(() -> setTiltPos(AlgaeSP.STOWDN), this);

	public Command toggleExtract = new InstantCommand(() -> toggleExtract());

	public Command intake = new AlgaeIntake(this);

	public Command eject = new AlgaeEject(this);

	/**
	 * doAction - performs Algea action based on Ladder position
	 */
	// public Command doAction() {
	// Command cmd = null;
	// LadderSP sp = ladder.getLadderSP();

	// System.out.println("Algae Ladder SP: " + sp.toString());

	// switch (sp) {
	// case BARGE:
	// case PROCESSOR:
	// cmd = this.eject;
	// break;
	// case L3:
	// case L2:
	// if (extractAlgae) {
	// cmd = this.intake;
	// }
	// break;
	// case FLOOR:
	// cmd = this.intake;
	// break;
	// default:
	// cmd = new WaitCommand(0.25);
	// }

	// return cmd;
	// }

	// new InstantCommand(() -> setIntakeVel(algaeSP))
	// .andThen(new WaitCommand(0.5))
	// .andThen(() -> setIntakeVel(Constants.Algae.STOP));

	/**************************************************************
	 * Methods
	 **************************************************************/

	// public void moveTilt(double pos) {
	// tiltController.setReference(getTiltPos() + pos,
	// SparkBase.ControlType.kMAXMotionPositionControl);
	// }

	public void doAction() {
		LadderSP sp = ladder.getLadderSP();

		System.out.println("Algae Ladder SP: " + sp.toString());

		switch (sp) {
			case BARGE:
			case PROCESSOR:
				setIntakeEject(Algae.IntakeEject.EJECT);
				break;
			case L3:
				if (extractAlgae) {
					setIntakeEject(Algae.IntakeEject.INTAKE);
				}
				break;
			case L35:
				setIntakeEject(Algae.IntakeEject.INTAKE);
				break;
			case FLOOR:
				setIntakeEject(Algae.IntakeEject.INTAKE);
				break;
			default:
				setIntakeEject(Algae.IntakeEject.STOP);
		}
	}

	public IntakeEject getIntakeEject() {
		return intakeEject;
	}

	public void setIntakeEject(IntakeEject state) {
		intakeEject = state;
	}

	public void holdIntakePos() {
		leftIntake.set(0.0);
		double pos = intakeEncoder.getPosition();
		intakeController.setReference(pos, SparkBase.ControlType.kMAXMotionPositionControl);
	}

	// Getting the position of the encoders
	public double getIntakeVel() {
		return intakeEncoder.getVelocity();
	}

	public double getTiltPos() {
		return tiltEncoder.getPosition();
	}

	public void setAlgaeSP(AlgaeSP sp) {
		algaeSP = sp;
	}

	public AlgaeSP getAlgaeSP() {
		return algaeSP;
	}

	public double getTiltSP() {
		return algaeSP.getTilt();
	}

	public double getIntakeSP() {
		return algaeSP.getIntake();
	}

	// Sets the position of the encoders
	public void setTiltPos(AlgaeSP sp) {
		setAlgaeSP(sp);
		tiltController.setReference(sp.getTilt(), SparkBase.ControlType.kMAXMotionPositionControl);
	}

	// Sets the position of the encoders
	public void setTiltPos() {
		setTiltPos(getAlgaeSP());
	}

	public void setIntakeVel(double vel) {
		intakeController.setReference(vel, SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public void setIntakeVel(AlgaeSP sp) {
		setAlgaeSP(sp);
		intakeController.setReference(sp.getIntake(), SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public void setIntakeVel() {
		setIntakeVel(getAlgaeSP());
	}

	public boolean onTiltTarget() {
		return Math.abs(getTiltPos() - getTiltSP()) < Constants.Algae.kTiltTollerance;
	}

	public boolean onIntakeTarget() {
		return Math.abs(getIntakeVel() - getIntakeSP()) < Constants.Algae.kIntakeTollerance;
	}

	public boolean isLimit() {
		return limitSwitch.isPressed();
	}

	public void toggleExtract() {
		extractAlgae = !extractAlgae;
	}

	public boolean getExtract() {
		return extractAlgae;
	}
}