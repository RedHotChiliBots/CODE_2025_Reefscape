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
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
	private SparkClosedLoopController leftIntakeController = leftIntake.getClosedLoopController();
	// private SparkClosedLoopController rightIntakeController =
	// rightIntake.getClosedLoopController();

	private RelativeEncoder leftEncoder = leftIntake.getEncoder();
	// private RelativeEncoder rightEncoder = rightIntake.getEncoder();
	private AbsoluteEncoder tiltEncoder = tilt.getAbsoluteEncoder();

	private SparkLimitSwitch leftForLimitSwitch = leftIntake.getForwardLimitSwitch();
	private SparkLimitSwitch leftRevLimitSwitch = leftIntake.getReverseLimitSwitch();
	// private SparkLimitSwitch rightForLimitSwitch =
	// rightIntake.getForwardLimitSwitch();
	// private SparkLimitSwitch rightRevLimitSwitch =
	// rightIntake.getReverseLimitSwitch();

	private Ladder ladder = null;

	private double tiltSP = Constants.Algae.STOW;
	private double intakeSP = Constants.Algae.STOP;

	private double prevTiltSP = tiltSP;
	public Trigger tiltChanged = new Trigger(() -> getTiltSPChanged());

	private double prevIntakeSP = intakeSP;
	public Trigger leftIntakeChanged = new Trigger(() -> getIntakeSPChanged());

	private final ShuffleboardTab algaeTab = Shuffleboard.getTab("Algae");
	private final GenericEntry sbLeftVel = algaeTab.addPersistent("Intake Vel", 0)
			.withWidget("Text View").withPosition(2, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbIntakeSP = algaeTab.addPersistent("Intake SP", 0)
			.withWidget("Text View").withPosition(3, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbTiltPos = algaeTab.addPersistent("Tilt Pos", 0)
			.withWidget("Text View").withPosition(2, 1).withSize(1, 1).getEntry();
	private final GenericEntry sbTiltSP = algaeTab.addPersistent("Tilt SP", 0)
			.withWidget("Text View").withPosition(3, 1).withSize(1, 1).getEntry();
	private final GenericEntry sbLimit = algaeTab.addPersistent("Limit", false)
			.withWidget("Boolean Box").withPosition(4, 0).withSize(1, 1).getEntry();

	private int loopCtr = 0;

	// Creates a new Algae.
	public Algae(Ladder ladder) {
		System.out.println("+++++ Starting Algae Constructor +++++");
		this.ladder = ladder;

		// Configure Left Intake motor
		leftConfig
				.inverted(Constants.Algae.kLeftMotorInverted)
				.idleMode(Constants.Algae.kLeftIdleMode)
				.smartCurrentLimit(Constants.Algae.kLeftCurrentLimit);
		leftConfig.encoder
				.positionConversionFactor(Constants.Algae.kLeftEncoderPositionFactor)
				.velocityConversionFactor(Constants.Algae.kLeftEncoderVelocityFactor);
		leftConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pidf(Constants.Algae.kLeftP,
						Constants.Algae.kLeftI,
						Constants.Algae.kLeftD,
						Constants.Algae.kLeftFF)
				.outputRange(Constants.Algae.kLeftMinOutput, Constants.Algae.kLeftMaxOutput)
				.positionWrappingEnabled(Constants.Algae.kLeftEncodeWrapping);

		leftIntake.configure(leftConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Right Intake motor
		rightConfig
				.follow(leftIntake)
				.inverted(Constants.Algae.kRightMotorInverted)
				.idleMode(Constants.Algae.kRightIdleMode)
				.smartCurrentLimit(Constants.Algae.kRightCurrentLimit);
		// rightConfig.encoder
		// .positionConversionFactor(Constants.Algae.kRightEncoderPositionFactor)
		// .velocityConversionFactor(Constants.Algae.kRightEncoderVelocityFactor);
		// rightConfig.closedLoop
		// .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
		// .pidf(Constants.Algae.kRightP,
		// Constants.Algae.kRightI,
		// Constants.Algae.kRightD,
		// Constants.Algae.kRightFF)
		// .outputRange(Constants.Algae.kRightMinOutput,
		// Constants.Algae.kRightMaxOutput)
		// .positionWrappingEnabled(Constants.Algae.kRightEncodeWrapping);

		rightIntake.configure(rightConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Tilt motor
		tiltConfig
				.inverted(Constants.Algae.kTiltMotorInverted)
				.idleMode(Constants.Algae.kTiltIdleMode)
				.smartCurrentLimit(Constants.Algae.kTiltCurrentLimit);
		tiltConfig.encoder
				.positionConversionFactor(Constants.Algae.kTiltEncoderPositionFactor)
				.velocityConversionFactor(Constants.Algae.kTiltEncoderVelocityFactor);
		tiltConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pidf(Constants.Algae.kTiltP,
						Constants.Algae.kTiltI,
						Constants.Algae.kTiltD,
						Constants.Algae.kTiltFF)
				.outputRange(Constants.Algae.kTiltMinOutput, Constants.Algae.kTiltMaxOutput)
				.positionWrappingEnabled(Constants.Algae.kTiltEncodeWrapping);

		tilt.configure(tiltConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		setIntakeVel(intakeSP);
		setTiltPos(tiltSP);

		System.out.println("----- Ending Algae Constructor -----");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run SuffleBoard
		// setTiltSP(sbTiltSP.getDouble(0.0));
		// setIntakeSP(sbIntakeSP.getDouble(0.0));

		sbLeftVel.setDouble(getIntakeVel());
		// sbRightVel.setDouble(getRightVel());
		sbIntakeSP.setDouble(getIntakeSP());
		sbTiltPos.setDouble(getTiltPos());
		sbTiltSP.setDouble(getTiltSP());
		sbLimit.setBoolean(isLimit());

		if (loopCtr++ % 20 == 0.0) {
			System.out.println("Algae Tilt SP: " + tiltSP + " " + sbTiltSP.getDouble(0.0));
			System.out.println("Algae Intake SP: " + intakeSP + " " +
					sbIntakeSP.getDouble(0.0));
		}
	}

	/**
	 * setTiltCmd - command factory method to update the Tilt pos
	 * 
	 * @return a command
	 */
	public Command setTiltCmd(double pos) {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			setTiltPos(pos);
		});
	}

	public Command setTiltCmd() {
		return setTiltCmd(getTiltSP());
	}

	public Command setTiltSPCmd(double sp) {
		return runOnce(() -> {
			setTiltSP(sp);
		});
	}

	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public Command setIntakeCmd(double pos) {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			setIntakeVel(pos);
		});
	}

	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public Command setIntakeCmd() {
		return setIntakeCmd(getIntakeSP());
	}

	public Command setIntakeSPCmd(double sp) {
		return runOnce(() -> {
			setIntakeSP(sp);
		});
	}

	/**
	 * An example method querying a boolean state of the subsystem (for example, a
	 * digital sensor).
	 *
	 * @return value of some boolean subsystem state, such as a digital sensor.
	 */
	public boolean exampleCondition() {
		// Query some boolean state, such as a digital sensor.
		return false;
	}

	public boolean getTiltSPChanged() {
		double currTiltSP = getTiltSP();
		boolean changed = prevTiltSP != currTiltSP;
		if (changed)
			System.out.println("Algae Tilt SP Changed from " + prevTiltSP + " to " + currTiltSP);
		prevTiltSP = currTiltSP;
		return changed;
	}

	public boolean getIntakeSPChanged() {
		double currIntakeSP = getIntakeSP();
		boolean changed = prevIntakeSP != currIntakeSP;
		if (changed)
			System.out.println("Algae Intake SP Changed from " + prevIntakeSP + " to " + currIntakeSP);
		prevIntakeSP = currIntakeSP;
		return changed;
	}

	// Getting the position of the encoders
	public double getIntakeVel() {
		return leftEncoder.getVelocity();
	}

	// public double getRightVel() {
	// return rightEncoder.getVelocity();
	// }

	public double getTiltPos() {
		return tiltEncoder.getPosition();
	}

	// Sets the position of the encoders
	public void setTiltPos(double pos) {
		setTiltSP(pos);
		tiltController.setReference(pos, SparkBase.ControlType.kMAXMotionPositionControl);
	}

	// Sets the position of the encoders
	public void setTiltPos() {
		tiltController.setReference(getTiltSP(), SparkBase.ControlType.kMAXMotionPositionControl);
	}

	public void setIntakeVel(double vel) {
		setIntakeSP(vel);
		leftIntakeController.setReference(vel, SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public void setIntakeVel() {
		leftIntakeController.setReference(getIntakeSP(), SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public void setIntakeSP(double sp) {
		System.out.println("Setting Algae Intake SP to " + sp);
		intakeSP = sp;
	}

	public double getIntakeSP() {
		return intakeSP;
	}

	public void setTiltSP(double sp) {
		System.out.println("Setting Algae Tilt SP to " + sp);
		tiltSP = sp;
	}

	public double getTiltSP() {
		return tiltSP;
	}

	public boolean isLimit() {
		return leftForLimitSwitch.isPressed(); // || rightForLimitSwitch.isPressed();
	}

	public Command algaeBarge() {
		System.out.println("Running algaeBarge");
		return this.runOnce(() -> setTiltSP(Constants.Algae.BARGE))
				.andThen(() -> ladder.setLadderSP(Constants.Algae.BARGE));
	}

	public Command algaeL3() {
		System.out.println("Running algaeL3");
		return this.runOnce(() -> setTiltSP(Constants.Algae.L3)).andThen(() -> ladder.setLadderSP(Constants.Algae.L3));
	}

	public Command algaeL2() {
		System.out.println("Running algaeL2");
		return this.runOnce(() -> setTiltSP(Constants.Algae.L2)).andThen(() -> ladder.setLadderSP(Constants.Algae.L2));
	}

	public Command algaeFloor() {
		System.out.println("Running algaeFloor");
		return this.runOnce(() -> setTiltSP(Constants.Algae.FLOOR))
				.andThen(() -> ladder.setLadderSP(Constants.Algae.FLOOR));
	}

	public Command algaeStow() {
		System.out.println("Running algaeStow");
		return this.runOnce(() -> setTiltSP(Constants.Algae.STOW))
				.andThen(() -> ladder.setLadderSP(Constants.Algae.STOW));
	}

	public Command algaeIntake() {
		System.out.println("Running algaeIntake");
		return this.runOnce(() -> setIntakeSP(Constants.Algae.INTAKE));
	}

	public Command algaeEject() {
		System.out.println("Running algaeEject");
		return this.runOnce(() -> setIntakeSP(Constants.Algae.EJECT));
	}
}