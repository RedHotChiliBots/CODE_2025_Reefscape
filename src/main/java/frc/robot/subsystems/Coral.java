package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase {

	// Define Intake Motors
	private final SparkMax leftIntake = new SparkMax(
			Constants.CANId.kCoralLeftIntakeCanId, MotorType.kBrushless);
	private final SparkMax rightIntake = new SparkMax(
			Constants.CANId.kCoralRightIntakeCanId, MotorType.kBrushless);
	private final SparkMax tilt = new SparkMax(
			Constants.CANId.kCoralTiltCanId, MotorType.kBrushless);

	private final SparkMaxConfig leftIntakeConfig = new SparkMaxConfig();
	private final SparkMaxConfig rightIntakeConfig = new SparkMaxConfig();
	private final SparkMaxConfig tiltConfig = new SparkMaxConfig();

	private SparkClosedLoopController leftIntakeController = leftIntake.getClosedLoopController();
	private SparkClosedLoopController rightIntakeController = rightIntake.getClosedLoopController();
	private SparkClosedLoopController tiltController = tilt.getClosedLoopController();

	private RelativeEncoder leftIntakeEncoder = leftIntake.getEncoder();
	private RelativeEncoder rightIntakeEncoder = rightIntake.getEncoder();
	private AbsoluteEncoder tiltEncoder = tilt.getAbsoluteEncoder();

	private SparkLimitSwitch leftLimitSwitch = leftIntake.getForwardLimitSwitch(); // leftIntake.getReverseLimitSwitch();
	private SparkLimitSwitch rightLimitSwitch = rightIntake.getForwardLimitSwitch(); // rightIntake.getReverseLimitSwitch();

	public enum CoralSP {
		STOW(90.0), // degrees
		STATION(55.0), // degrees
		L1(0.0), // degrees
		L2(-35.0), // degrees
		L3(-35.0), // degrees
		L4(-45.0); // degrees

		private final double sp;

		CoralSP(final double sp) {
			this.sp = sp;
		}

		public double getValue() {
			return sp;
		}
	}

	private Ladder ladder = null;

	private double leftIntakeSP = Constants.Coral.STOP;
	private double rightIntakeSP = Constants.Coral.STOP;
	private CoralSP tiltSP = CoralSP.STOW;

	private boolean leftCoral = true;

	private final ShuffleboardTab coralTab = Shuffleboard.getTab("Coral");
	private final GenericEntry sbLeftIntakeVel = coralTab.addPersistent("Left Intake Vel", 0)
			.withWidget("Text View").withPosition(2, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbLeftIntakeSP = coralTab.addPersistent("Left Intake SP", 0)
			.withWidget("Text View").withPosition(3, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbRightIntakeVel = coralTab.addPersistent("Right Intake Vel", 0)
			.withWidget("Text View").withPosition(2, 1).withSize(1, 1).getEntry();
	private final GenericEntry sbRightIntakeSP = coralTab.addPersistent("Right Intake SP", 0)
			.withWidget("Text View").withPosition(3, 1).withSize(1, 1).getEntry();

	private final GenericEntry sbTxtTiltSP = coralTab.addPersistent("Tilt tSP", "")
			.withWidget("Text View").withPosition(1, 2).withSize(1, 1).getEntry();
	private final GenericEntry sbDblTiltSP = coralTab.addPersistent("Tilt dSP", 0)
			.withWidget("Text View").withPosition(2, 2).withSize(1, 1).getEntry();
	private final GenericEntry sbTiltPos = coralTab.addPersistent("Tilt Pos", 0)
			.withWidget("Text View").withPosition(3, 2).withSize(1, 1).getEntry();
	private final GenericEntry sbSide = coralTab.addPersistent("Side", "")
			.withWidget("Text View").withPosition(5, 0).withSize(1, 1).getEntry();

	private final GenericEntry sbLeftLimit = coralTab.addPersistent("Left Limit", false)
			.withWidget("Boolean Box").withPosition(4, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbRightLimit = coralTab.addPersistent("Right Limit", false)
			.withWidget("Boolean Box").withPosition(4, 1).withSize(1, 1).getEntry();

	// Creates a new Coral.
	public Coral(Ladder ladder) {
		System.out.println("+++++ Starting Coral Constructor +++++");
		this.ladder = ladder;

		// Configure Left Intake motor
		leftIntakeConfig
				.inverted(Constants.Coral.kLeftMotorInverted)
				.idleMode(Constants.Coral.kIntakeIdleMode)
				.smartCurrentLimit(Constants.Coral.kLeftCurrentLimit);
		leftIntakeConfig.encoder
				.positionConversionFactor(Constants.Coral.kIntakePositionFactor)
				.velocityConversionFactor(Constants.Coral.kIntakeVelocityFactor);
		leftIntakeConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Coral.kIntakeP)
				.i(Constants.Coral.kIntakeI)
				.d(Constants.Coral.kIntakeD)
				.outputRange(Constants.Coral.kIntakeMinOutput, Constants.Coral.kIntakeMaxOutput)
				.positionWrappingEnabled(Constants.Coral.kLeftEncodeWrapping);

		leftIntake.configure(
				leftIntakeConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Left Intake motor
		rightIntakeConfig
				.inverted(Constants.Coral.kLeftMotorInverted)
				.idleMode(Constants.Coral.kIntakeIdleMode)
				.smartCurrentLimit(Constants.Coral.kLeftCurrentLimit);
		rightIntakeConfig.encoder
				.positionConversionFactor(Constants.Coral.kIntakePositionFactor)
				.velocityConversionFactor(Constants.Coral.kIntakeVelocityFactor);
		rightIntakeConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Coral.kIntakeP)
				.i(Constants.Coral.kIntakeI)
				.d(Constants.Coral.kIntakeD)
				.outputRange(Constants.Coral.kIntakeMinOutput, Constants.Coral.kIntakeMaxOutput)
				.positionWrappingEnabled(Constants.Coral.kLeftEncodeWrapping);

		rightIntake.configure(
				rightIntakeConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Tilt motor
		tiltConfig
				.inverted(Constants.Coral.kTiltMotorInverted)
				.idleMode(Constants.Coral.kTiltIdleMode)
				.smartCurrentLimit(Constants.Coral.kTiltCurrentLimit);
		tiltConfig.encoder
				.positionConversionFactor(Constants.Coral.kTiltPositionFactor)
				.velocityConversionFactor(Constants.Coral.kTiltVelocityFactor);
		tiltConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Coral.kTiltP)
				.i(Constants.Coral.kTiltI)
				.d(Constants.Coral.kTiltD)
				.outputRange(Constants.Coral.kTiltMinOutput, Constants.Coral.kTiltMaxOutput)
				.positionWrappingEnabled(Constants.Coral.kTiltEncodeWrapping);

		tilt.configure(tiltConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		setLeftIntakeVel(leftIntakeSP);
		setRightIntakeVel(rightIntakeSP);
		setTiltPos(tiltSP);

		System.out.println("----- Ending Coral Constructor -----");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		// setTiltSP(sbTiltSP.getDouble(0.0));
		// setLeftIntakeSP(sbLeftIntakeSP.getDouble(0.0));
		// setRightIntakeSP(sbRightIntakeSP.getDouble(0.0));

		sbLeftIntakeVel.setDouble(getLeftIntakeVel());
		sbLeftIntakeSP.setDouble(getLeftIntakeSP());
		sbRightIntakeVel.setDouble(getRightIntakeVel());
		sbRightIntakeSP.setDouble(getRightIntakeSP());

		sbTxtTiltSP.setString(getTiltSP().toString());
		sbDblTiltSP.setDouble(getTiltSP().getValue());
		sbTiltPos.setDouble(getTiltPos());

		if (leftCoral) {
			sbSide.setString("Left");
		} else {
			sbSide.setString("Right");
		}

		sbLeftLimit.setBoolean(isLeftLimit());
		sbRightLimit.setBoolean(isRightLimit());
	}

	public double getLeftIntakeVel() {
		return leftIntakeEncoder.getVelocity();
	}

	public double getRightIntakeVel() {
		return rightIntakeEncoder.getVelocity();
	}

	public double getTiltPos() {
		return tiltEncoder.getPosition();
	}

	public void setTiltPos(CoralSP pos) {
		setTiltSP(pos);
		tiltController.setReference(pos.getValue(), SparkBase.ControlType.kMAXMotionPositionControl);
	}

	public void setTiltPos() {
		tiltController.setReference(getTiltSP().getValue(), SparkBase.ControlType.kMAXMotionPositionControl);
	}

	public void setLeftIntakeVel(double vel) {
		setLeftIntakeSP(vel);
		leftIntakeController.setReference(vel, SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public void setRightIntakeVel(double vel) {
		setRightIntakeSP(vel);
		rightIntakeController.setReference(vel, SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public void setLeftIntakeSP(double sp) {
		leftIntakeSP = sp;
	}

	public void setRightIntakeSP(double sp) {
		rightIntakeSP = sp;
	}

	public double getLeftIntakeSP() {
		return leftIntakeSP;
	}

	public double getRightIntakeSP() {
		return rightIntakeSP;
	}

	public void setTiltSP(CoralSP sp) {
		tiltSP = sp;
	}

	public CoralSP getTiltSP() {
		return tiltSP;
	}

	public boolean isLeftLimit() {
		return leftLimitSwitch.isPressed();
	}

	public boolean isRightLimit() {
		return rightLimitSwitch.isPressed();
	}

	public void toggleSide() {
		leftCoral = !leftCoral;
	}

	public void doAction() {
		double vel = 0.0;

		switch (ladder.getLadderSP()) {
			case STATION:
				vel = Constants.Algae.INTAKE;
				break;
			case L4:
			case L3:
			case L2:
			case L1:
				vel = Constants.Algae.EJECT;
				break;
			default:
		}

		if (leftCoral) {
			setLeftIntakeVel(vel);
		} else {
			setRightIntakeVel(vel);
		}
	}

	public Command doActionCmd() {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			doAction();
		});
	}

	public Command toggleSideCmd() {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			toggleSide();
		});
	}

	/**
	 * setTiltCmd - command factory method to update the Tilt pos
	 * 
	 * @return a command
	 */
	public Command setLeftIntakeCmd(double vel) {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			setLeftIntakeVel(vel);
		});
	}

	public Command setRightIntakeCmd(double vel) {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			setRightIntakeVel(vel);
		});
	}

	public Command setRightIntakeCmd() {
		return setLeftIntakeCmd(getLeftIntakeSP());
	}

	public Command setTiltSPCmd(CoralSP sp) {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			setTiltSP(sp);
		});
	}

	public Command setTiltSPCmd() {
		return setTiltSPCmd(getTiltSP());
	}

	public Command setTiltPosCmd(CoralSP pos) {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			setTiltPos(pos);
		});
	}

	public Command setTiltPosCmd() {
		return setTiltPosCmd(getTiltSP());
	}

	public Command coralL4() {
		return this.runOnce(() -> setTiltSP(Coral.CoralSP.L4));
	}

	public Command coralL3() {
		return this.runOnce(() -> setTiltSP(Coral.CoralSP.L3));
	}

	public Command coralL2() {
		return this.runOnce(() -> setTiltSP(Coral.CoralSP.L2));
	}

	public Command coralL1() {
		return this.runOnce(() -> setTiltSP(Coral.CoralSP.L1));
	}

	public Command coralStation() {
		return this.runOnce(() -> setTiltSP(
				Coral.CoralSP.STATION));
	}

	public Command coralStow() {
		return this.runOnce(() -> setTiltSP(
				Coral.CoralSP.STOW));
	}

	public Command coralLeftIntake() {
		return this.runOnce(() -> setLeftIntakeSP(Constants.Coral.INTAKE));
	}

	public Command coralLeftEject() {
		return this.runOnce(() -> setLeftIntakeSP(Constants.Coral.EJECT));
	}

	public Command coralRightIntake() {
		return this.runOnce(() -> setRightIntakeSP(Constants.Coral.INTAKE));
	}

	public Command coralRightEject() {
		return this.runOnce(() -> setRightIntakeSP(Constants.Coral.EJECT));
	}
}
