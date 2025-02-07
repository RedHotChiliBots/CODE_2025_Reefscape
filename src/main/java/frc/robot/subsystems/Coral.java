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

	private SparkLimitSwitch leftFwdLimitSwitch = leftIntake.getForwardLimitSwitch();
	private SparkLimitSwitch leftRevLimitSwitch = leftIntake.getReverseLimitSwitch();
	private SparkLimitSwitch rightFwdLimitSwitch = rightIntake.getForwardLimitSwitch();
	private SparkLimitSwitch rightRevLimitSwitch = rightIntake.getReverseLimitSwitch();

	private Ladder ladder = null;

	private double leftIntakeSP = Constants.Coral.STOP;
	private double rightIntakeSP = Constants.Coral.STOP;
	private double tiltSP = Constants.Coral.STOW;

	private double prevLeftIntakeSP = leftIntakeSP;
	private double prevRightIntakeSP = rightIntakeSP;
	public VariableChangeTrigger leftIntakeChanged = new VariableChangeTrigger(() -> getLeftIntakeSPChanged());
	public VariableChangeTrigger rightIntakeChanged = new VariableChangeTrigger(() -> getRightIntakeSPChanged());

	private double prevTiltSP = tiltSP;
	public VariableChangeTrigger tiltChanged = new VariableChangeTrigger(() -> getTiltSPChanged());

	private final ShuffleboardTab coralTab = Shuffleboard.getTab("Coral");
	private final GenericEntry sbLeftIntakeVel = coralTab.addPersistent("Left Intake Vel", 0)
			.withWidget("Text View").withPosition(2, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbLeftIntakeSP = coralTab.addPersistent("Left Intake SP", 0)
			.withWidget("Text View").withPosition(3, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbRightIntakeVel = coralTab.addPersistent("Right Intake Vel", 0)
			.withWidget("Text View").withPosition(2, 1).withSize(1, 1).getEntry();
	private final GenericEntry sbRightIntakeSP = coralTab.addPersistent("Right Intake SP", 0)
			.withWidget("Text View").withPosition(3, 1).withSize(1, 1).getEntry();
	private final GenericEntry sbTiltPos = coralTab.addPersistent("Tilt Pos", 0)
			.withWidget("Text View").withPosition(2, 2).withSize(1, 1).getEntry();
	private final GenericEntry sbTiltSP = coralTab.addPersistent("Tilt SP", 0)
			.withWidget("Text View").withPosition(3, 2).withSize(1, 1).getEntry();
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
				.positionConversionFactor(Constants.Coral.kIntakeEncoderPositionFactor)
				.velocityConversionFactor(Constants.Coral.kIntakeEncoderVelocityFactor);
		leftIntakeConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pidf(Constants.Coral.kIntakeP,
						Constants.Coral.kIntakeI,
						Constants.Coral.kIntakeD,
						Constants.Coral.kIntakeFF)
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
				.positionConversionFactor(Constants.Coral.kIntakeEncoderPositionFactor)
				.velocityConversionFactor(Constants.Coral.kIntakeEncoderVelocityFactor);
		rightIntakeConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pidf(Constants.Coral.kIntakeP,
						Constants.Coral.kIntakeI,
						Constants.Coral.kIntakeD,
						Constants.Coral.kIntakeFF)
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
				.positionConversionFactor(Constants.Coral.kTiltEncoderPositionFactor)
				.velocityConversionFactor(Constants.Coral.kTiltEncoderVelocityFactor);
		tiltConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pidf(Constants.Coral.kTiltP,
						Constants.Coral.kTiltI,
						Constants.Coral.kTiltD,
						Constants.Coral.kTiltFF)
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

		setTiltSP(sbTiltSP.getDouble(0.0));
		setLeftIntakeSP(sbLeftIntakeSP.getDouble(0.0));
		setRightIntakeSP(sbRightIntakeSP.getDouble(0.0));

		sbLeftIntakeVel.setDouble(getLeftIntakeVel());
		sbLeftIntakeSP.setDouble(getLeftIntakeSP());
		sbRightIntakeVel.setDouble(getRightIntakeVel());
		sbRightIntakeSP.setDouble(getRightIntakeSP());
		sbTiltPos.setDouble(getTiltPos());
		sbTiltSP.setDouble(getTiltSP());
		sbLeftLimit.setBoolean(isLeftLimit());
		sbRightLimit.setBoolean(isRightLimit());
	}

	private boolean getTiltSPChanged() {
		double currTiltSP = getTiltSP();
		boolean changed = prevTiltSP != currTiltSP;
		prevTiltSP = currTiltSP;
		return changed;
	}

	private boolean getLeftIntakeSPChanged() {
		double currLeftIntakeSP = getLeftIntakeSP();
		boolean changed = prevLeftIntakeSP != currLeftIntakeSP;
		prevLeftIntakeSP = currLeftIntakeSP;
		return changed;
	}

	private boolean getRightIntakeSPChanged() {
		double currRightIntakeSP = getRightIntakeSP();
		boolean changed = prevRightIntakeSP != currRightIntakeSP;
		prevRightIntakeSP = currRightIntakeSP;
		return changed;
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

	public void setTiltPos(double pos) {
		setTiltSP(pos);
		tiltController.setReference(pos, SparkBase.ControlType.kMAXMotionPositionControl);
	}

	public void setTiltPos() {
		tiltController.setReference(getTiltSP(), SparkBase.ControlType.kMAXMotionPositionControl);
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

	public void setTiltSP(double sp) {
		tiltSP = sp;
	}

	public double getTiltSP() {
		return tiltSP;
	}

	public boolean isLeftLimit() {
		return leftFwdLimitSwitch.isPressed();
	}

	public boolean isRightLimit() {
		return rightFwdLimitSwitch.isPressed();
	}

	public Command coralL4() {
		return this.runOnce(() -> setTiltSP(Constants.Coral.L4)).andThen(() -> ladder.setLadderSP(Constants.Coral.L4));
	}

	public Command coralL3() {
		return this.runOnce(() -> setTiltSP(Constants.Coral.L3)).andThen(() -> ladder.setLadderSP(Constants.Coral.L3));
	}

	public Command coralL2() {
		return this.runOnce(() -> setTiltSP(Constants.Coral.L2)).andThen(() -> ladder.setLadderSP(Constants.Coral.L2));
	}

	public Command coralL1() {
		return this.runOnce(() -> setTiltSP(Constants.Coral.L1)).andThen(() -> ladder.setLadderSP(Constants.Coral.L1));
	}

	public Command coralStation() {
		return this.runOnce(() -> setTiltSP(Constants.Coral.STATION)).andThen(() -> ladder.setLadderSP(Constants.Coral.STATION));
	}

	public Command coralStow() {
		return this.runOnce(() -> setTiltSP(Constants.Coral.STOW)).andThen(() -> ladder.setLadderSP(Constants.Coral.STOW));
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
