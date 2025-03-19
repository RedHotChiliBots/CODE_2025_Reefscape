package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.CoralEject;
import frc.robot.commands.CoralIntake;
import frc.robot.utils.Library;

public class Coral extends SubsystemBase {

	// Define Intake Motors
	private final SparkMax leftIntake = new SparkMax(
			Constants.CANId.kCoralLeftIntakeCanId, MotorType.kBrushless);
	private final SparkMax rightIntake = new SparkMax(
			Constants.CANId.kCoralRightIntakeCanId, MotorType.kBrushless);
	private final SparkMax tilt = new SparkMax(
			Constants.CANId.kCoralTiltCanId, MotorType.kBrushless);

	private final SparkMaxConfig leftConfig = new SparkMaxConfig();
	private final SparkMaxConfig rightConfig = new SparkMaxConfig();
	private final SparkMaxConfig tiltConfig = new SparkMaxConfig();

	private SparkClosedLoopController leftController = leftIntake.getClosedLoopController();
	private SparkClosedLoopController rightController = rightIntake.getClosedLoopController();
	private SparkClosedLoopController tiltController = tilt.getClosedLoopController();

	private RelativeEncoder leftEncoder = leftIntake.getEncoder();
	private RelativeEncoder rightEncoder = rightIntake.getEncoder();
	private AbsoluteEncoder tiltEncoder = tilt.getAbsoluteEncoder();

	public enum CoralSP {
		STOW(90.0, Constants.Coral.STOP), // degrees
		STATION(55.0, Constants.Coral.INTAKE), // degrees
		ZERO(0.0, Constants.Coral.STOP),
		L1(0.0, Constants.Coral.EJECT), // degrees
		L2(-35.0, Constants.Coral.EJECT), // degrees
		L3(-35.0, Constants.Coral.EJECT), // degrees
		L4(-40.0, Constants.Coral.EJECT); // degrees

		private final double tilt;
		private final double intake;

		CoralSP(double tilt, double intake) {
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

	private Ladder ladder = null;
	private Algae algae = null;

	private PowerDistribution pdh = null;
	private CoralSP coralSP = CoralSP.ZERO;

	private boolean leftCoral = true;

	private Library lib = new Library();

	/**************************************************************
	 * Initialize Shuffleboard entries
	 **************************************************************/
	private final ShuffleboardTab cmdTab = Shuffleboard.getTab("Commands");
	private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

	private final GenericEntry sbTiltOnTgt = compTab.addPersistent("Coral Tilt OnTgt", false)
			.withWidget("Boolean Box").withPosition(11, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbTxtTiltSP = compTab.addPersistent("Coral Tilt SP", "")
			.withWidget("Text View").withPosition(11, 2).withSize(2, 1).getEntry();
	private final GenericEntry sbDblTiltSP = compTab.addPersistent("Coral Tilt SP (deg)", 0)
			.withWidget("Text View").withPosition(11, 3).withSize(2, 1).getEntry();
	private final GenericEntry sbTiltPos = compTab.addPersistent("Coral Tilt Pos", 0)
			.withWidget("Text View").withPosition(11, 4).withSize(2, 2).getEntry();

	private final GenericEntry sbLeftInOnTgt = compTab.addPersistent("Coral Intake OnTgt", false)
			.withWidget("Boolean Box").withPosition(11, 6).withSize(2, 1).getEntry();
	private final GenericEntry sbLeftIntakeSP = compTab.addPersistent("Coral Intake SP", 0)
			.withWidget("Text View").withPosition(11, 7).withSize(2, 1).getEntry();
	private final GenericEntry sbLeftIntakeVel = compTab.addPersistent("Coral Intake Vel", 0)
			.withWidget("Text View").withPosition(11, 8).withSize(2, 1).getEntry();
	private final SimpleWidget sbMovingWidget = compTab.addPersistent("Coral Moving", "")
			.withWidget("Single Color View")
			.withPosition(11, 0)
			.withSize(2, 1);
	private final GenericEntry sbMoving = sbMovingWidget.getEntry();

	private final GenericEntry sbSide = compTab.addPersistent("Side", "")
			.withWidget("Text View").withPosition(11, 10).withSize(2, 1).getEntry();

	private final ShuffleboardLayout coralCommands = cmdTab
			.getLayout("Coral", BuiltInLayouts.kList)
			.withSize(3, 12)
			.withPosition(4, 1)
			.withProperties(Map.of("Label position", "Hidden"));

	/**************************************************************
	 * Constructor
	 **************************************************************/
	public Coral(Chassis chassis, Ladder ladder, Algae algae) {
		System.out.println("+++++ Starting Coral Constructor +++++");

		this.ladder = ladder;
		this.algae = algae;
		this.pdh = chassis.getPDH();

		compTab.add("Coral Current", this)
				.withWidget("Subsystem")
				.withPosition(22, 4)
				.withSize(4, 2);

		// Configure Left Intake motor
		leftConfig
				.inverted(Constants.Coral.kLeftMotorInverted)
				.idleMode(Constants.Coral.kIntakeIdleMode)
				.smartCurrentLimit(Constants.Coral.kLeftCurrentLimit);
		leftConfig.encoder
				.positionConversionFactor(Constants.Coral.kIntakePositionFactor)
				.velocityConversionFactor(Constants.Coral.kIntakeVelocityFactor);
		leftConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Coral.kIntakeVelP)
				.i(Constants.Coral.kIntakeVelI)
				.d(Constants.Coral.kIntakeVelD)
				.velocityFF(Constants.Coral.kIntakeVelFF)
				.outputRange(Constants.Coral.kIntakeVelMinOutput, Constants.Coral.kIntakeVelMaxOutput)
				.positionWrappingEnabled(Constants.Coral.kIntakeEncodeWrapping);
		leftConfig.closedLoop.maxMotion
				.maxVelocity(Constants.Coral.kIntakeVelMaxVel)
				.maxAcceleration(Constants.Coral.kIntakeVelMaxAccel)
				.allowedClosedLoopError(Constants.Coral.kIntakeVelAllowedErr);

		leftIntake.configure(
				leftConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Left Intake motor
		rightConfig
				.inverted(Constants.Coral.kLeftMotorInverted)
				.idleMode(Constants.Coral.kIntakeIdleMode)
				.smartCurrentLimit(Constants.Coral.kLeftCurrentLimit);
		rightConfig.encoder
				.positionConversionFactor(Constants.Coral.kIntakePositionFactor)
				.velocityConversionFactor(Constants.Coral.kIntakeVelocityFactor);
		rightConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Coral.kIntakeVelP)
				.i(Constants.Coral.kIntakeVelI)
				.d(Constants.Coral.kIntakeVelD)
				.velocityFF(Constants.Coral.kIntakeVelFF)
				.outputRange(Constants.Coral.kIntakeVelMinOutput, Constants.Coral.kIntakeVelMaxOutput)
				.positionWrappingEnabled(Constants.Coral.kIntakeEncodeWrapping);
		rightConfig.closedLoop.maxMotion
				.maxVelocity(Constants.Coral.kIntakeVelMaxVel)
				.maxAcceleration(Constants.Coral.kIntakeVelMaxAccel)
				.allowedClosedLoopError(Constants.Coral.kIntakeVelAllowedErr);

		rightIntake.configure(
				rightConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Tilt motor
		tiltConfig
				.inverted(Constants.Coral.kTiltMotorInverted)
				.idleMode(Constants.Coral.kTiltIdleMode)
				.smartCurrentLimit(Constants.Coral.kTiltCurrentLimit);
		tiltConfig.absoluteEncoder
				.zeroOffset(Constants.Coral.kTiltZeroOffset)
				.zeroCentered(Constants.Coral.kTiltZeroCentered)
				.inverted(Constants.Coral.kTiltEncoderInverted)
				.positionConversionFactor(Constants.Coral.kTiltPositionFactor)
				.velocityConversionFactor(Constants.Coral.kTiltVelocityFactor);
		tiltConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
				.p(Constants.Coral.kTiltPosP)
				.i(Constants.Coral.kTiltPosI)
				.d(Constants.Coral.kTiltPosD)
				.outputRange(Constants.Coral.kTiltPosMinOutput, Constants.Coral.kTiltPosMaxOutput)
				.positionWrappingEnabled(Constants.Coral.kTiltEncodeWrapping);
		tiltConfig.closedLoop.maxMotion
				.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
				.maxVelocity(Constants.Coral.kTiltPosMaxVel)
				.maxAcceleration(Constants.Coral.kTiltPosMaxAccel)
				.allowedClosedLoopError(Constants.Coral.kTiltPosAllowedErr);

		tilt.configure(tiltConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		coralCommands.add("L4", this.l4)
				.withProperties(Map.of("show type", false));
		coralCommands.add("L3", this.l3)
				.withProperties(Map.of("show type", false));
		coralCommands.add("L2", this.l2)
				.withProperties(Map.of("show type", false));
		coralCommands.add("L1", this.l1)
				.withProperties(Map.of("show type", false));
		coralCommands.add("Station", this.station)
				.withProperties(Map.of("show type", false));
		coralCommands.add("Stow", this.stow)
				.withProperties(Map.of("show type", false));
		coralCommands.add("Intake", this.intake)
				.withProperties(Map.of("show type", false));
		coralCommands.add("Eject", this.eject)
				.withProperties(Map.of("show type", false));
		coralCommands.add("Zero", this.zero)
				.withProperties(Map.of("show type", false));

		setCoralSP(CoralSP.ZERO);
		setLeftIntakeVel(getCoralSP());
		setRightIntakeVel(getCoralSP());
		setTiltPos(getCoralSP());

		// setTiltAfterAlgaePos(tiltSP);

		System.out.println("----- Ending Coral Constructor -----");
	}

	/**************************************************************
	 * Periodic
	 **************************************************************/
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		sbLeftIntakeVel.setDouble(getIntakeVel());
		sbLeftIntakeSP.setDouble(getIntakeSP());
		// sbRightIntakeVel.setDouble(getRightIntakeVel());
		// sbRightIntakeSP.setDouble(getRightIntakeSP());

		sbTxtTiltSP.setString(getCoralSP().toString());
		sbDblTiltSP.setDouble(getCoralSP().getTilt());
		sbTiltPos.setDouble(getTiltPos());

		sbLeftInOnTgt.setBoolean(onIntakeTarget());
		// sbRightInOnTgt.setBoolean(onRightIntakeTarget());
		sbTiltOnTgt.setBoolean(onTiltTarget());

		if (leftCoral) {
			sbSide.setString("Left");
		} else {
			sbSide.setString("Right");
		}

		// sbLeftLimit.setBoolean(isLeftLimit());
		// sbRightLimit.setBoolean(isRightLimit());

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
	public Command l4 = new InstantCommand(() -> setTiltPos(CoralSP.L4), this);
	public Command l3 = new InstantCommand(() -> setTiltPos(CoralSP.L3), this);
	public Command l2 = new InstantCommand(() -> setTiltPos(CoralSP.L2), this);
	public Command l1 = new InstantCommand(() -> setTiltPos(CoralSP.L1), this);
	public Command station = new InstantCommand(() -> setTiltPos(CoralSP.STATION), this);
	public Command zero = new InstantCommand(() -> setTiltPos(CoralSP.ZERO), this);
	public Command stow = new InstantCommand(() -> setTiltPos(CoralSP.STOW), this);

	public Command toggleSide = new InstantCommand(() -> toggleSide());

	public Command intake = new CoralIntake(this);

	public Command eject = new CoralEject(this);
	// new InstantCommand(() -> setIntakeVel(coralSP))
	// .andThen(new WaitCommand(0.5))
	// .andThen(() -> setIntakeVel(Constants.Coral.STOP));

	/**************************************************************
	 * Methods
	 **************************************************************/

	// public void moveTilt(double pos) {
	// tiltController.setReference(getTiltPos() + pos,
	// SparkBase.ControlType.kMAXMotionPositionControl);
	// }

	public void holdIntakePos() {
		if (leftCoral) {
			leftIntake.set(0.0);
			double pos = leftEncoder.getPosition();
			leftController.setReference(pos, SparkBase.ControlType.kMAXMotionPositionControl);

		} else {
			rightIntake.set(0.0);
			double pos = rightEncoder.getPosition();
			rightController.setReference(pos, SparkBase.ControlType.kMAXMotionPositionControl);
		}
	}

	// Getting the position of the encoders
	public double getLeftIntakeVel() {
		return leftEncoder.getVelocity();
	}

	public double getRightIntakeVel() {
		return rightEncoder.getVelocity();
	}

	public double getIntakeVel() {
		if (leftCoral) {
			return getLeftIntakeVel();
		} else {
			return getRightIntakeVel();
		}
	}

	public double getTiltPos() {
		return tiltEncoder.getPosition();
	}

	public void setCoralSP(CoralSP sp) {
		coralSP = sp;
	}

	public CoralSP getCoralSP() {
		return coralSP;
	}

	public double getTiltSP() {
		return coralSP.getTilt();
	}

	public double getIntakeSP() {
		return coralSP.getIntake();
	}

	public void setTiltPos(CoralSP sp) {
		setCoralSP(sp);
		tiltController.setReference(sp.getTilt(), SparkBase.ControlType.kMAXMotionPositionControl);
	}

	public void setTiltPos() {
		setTiltPos(getCoralSP());
	}

	public void setTiltAfterAlgaePos(CoralSP sp) {
		if (algae.isLimit()) {
			setCoralSP(sp);
			tiltController.setReference(sp.getTilt(), SparkBase.ControlType.kMAXMotionPositionControl);
		}
	}

	public void setIntakeVel(CoralSP sp) {
		setCoralSP(sp);
		if (leftCoral) {
			setLeftIntakeVel(sp);
		} else {
			setRightIntakeVel(sp);
		}
	}

	public void setIntakeVel() {
		setIntakeVel(getCoralSP());
	}

	public void setLeftIntakeVel(CoralSP sp) {
		setCoralSP(sp);
		leftController.setReference(sp.getIntake(), SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public void setRightIntakeVel(CoralSP sp) {
		setCoralSP(sp);
		rightController.setReference(sp.getIntake(), SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public void setIntakeVel(double sp) {
		if (leftCoral) {
			setLeftIntakeVel(sp);
		} else {
			setRightIntakeVel(sp);
		}
	}

	public void setLeftIntakeVel(double sp) {
		leftController.setReference(sp, SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public void setRightIntakeVel(double sp) {
		rightController.setReference(sp, SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public boolean isLimit() {
		if (leftCoral) {
			// return getLeftIntakeVel() < 100;
			return pdh.getCurrent(Constants.Coral.kLeftPDHChannel) > Constants.Coral.kStopCurrent;
		} else {
			// return getRightIntakeVel() < 100;
			return pdh.getCurrent(Constants.Coral.kRightPDHChannel) > Constants.Coral.kStopCurrent;
		}
	}

	public void toggleSide() {
		leftCoral = !leftCoral;
	}

	public boolean getSide() {
		return leftCoral;
	}

	public boolean onTiltTarget() {
		return Math.abs(getTiltPos() - getTiltSP()) < Constants.Coral.kTiltTollerance;
	}

	public boolean onIntakeTarget() {
		if (leftCoral) {
			return onLeftIntakeTarget();
		} else {
			return onRightIntakeTarget();
		}
	}

	public boolean onLeftIntakeTarget() {
		return Math.abs(getLeftIntakeVel() - getCoralSP().getIntake()) < Constants.Coral.kIntakeTollerance;
	}

	public boolean onRightIntakeTarget() {
		return Math.abs(getRightIntakeVel() - getCoralSP().getIntake()) < Constants.Coral.kIntakeTollerance;
	}

	public Command doAction() {
		Command cmd = null;

		switch (ladder.getLadderSP()) {
			case L4:
			case L3:
			case L2:
			case L1:
				cmd = this.eject;
				break;
			case STATION:
				cmd = this.intake;
				break;
			default:
				cmd = new WaitCommand(0.25);
		}

		return cmd;
	}
}
