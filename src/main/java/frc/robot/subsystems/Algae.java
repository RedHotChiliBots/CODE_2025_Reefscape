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
	private SparkClosedLoopController intakeController = leftIntake.getClosedLoopController(); // rightIntake.getClosedLoopController();

	private RelativeEncoder intakeEncoder = leftIntake.getEncoder(); // rightIntake.getEncoder();
	private AbsoluteEncoder tiltEncoder = tilt.getAbsoluteEncoder();

	private SparkLimitSwitch isLimitSwitch = leftIntake.getForwardLimitSwitch(); // leftIntake.getReverseLimitSwitch();

	private Ladder ladder = null;

	public enum AlgaeSP {
		STOW(90.0),
		FLOOR(-45.0),
		PROCESSOR(-10.0),
		L2(-35.0),
		L3(-35.0),
		BARGE(-20.0);

		private final double sp;

		AlgaeSP(final double sp) {
			this.sp = sp;
		}

		public double getValue() {
			return sp;
		}
	}

	private AlgaeSP tiltSP = AlgaeSP.STOW;
	private double intakeSP = Constants.Algae.STOP;

	private boolean extractAlgae = false;

	private final ShuffleboardTab algaeTab = Shuffleboard.getTab("Algae");
	private final GenericEntry sbIntakeVel = algaeTab.addPersistent("Intake Vel", 0)
			.withWidget("Text View").withPosition(2, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbIntakeSP = algaeTab.addPersistent("Intake SP", 0)
			.withWidget("Text View").withPosition(3, 0).withSize(1, 1).getEntry();

	private final GenericEntry sbTxtTiltSP = algaeTab.addPersistent("Tilt tSP", "")
			.withWidget("Text View").withPosition(1, 1).withSize(1, 1).getEntry();
	private final GenericEntry sbDblTiltSP = algaeTab.addPersistent("Tilt dSP", 0)
			.withWidget("Text View").withPosition(2, 1).withSize(1, 1).getEntry();
	private final GenericEntry sbTiltPos = algaeTab.addPersistent("Tilt Pos", 0)
			.withWidget("Text View").withPosition(3, 1).withSize(1, 1).getEntry();
	private final GenericEntry sbLimit = algaeTab.addPersistent("Limit", false)
			.withWidget("Boolean Box").withPosition(4, 0).withSize(1, 1).getEntry();

	private final GenericEntry sbExtract = algaeTab.addPersistent("Expel", false)
			.withWidget("Boolean Box").withPosition(4, 1).withSize(1, 1).getEntry();

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
				.positionConversionFactor(Constants.Algae.kIntakePositionFactor)
				.velocityConversionFactor(Constants.Algae.kIntakeVelocityFactor);
		leftConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Algae.kIntakeP)
				.i(Constants.Algae.kIntakeI)
				.d(Constants.Algae.kIntakeD)
				.outputRange(Constants.Algae.kIntakeMinOutput, Constants.Algae.kIntakeMaxOutput)
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
				.positionConversionFactor(Constants.Algae.kTiltPositionFactor)
				.velocityConversionFactor(Constants.Algae.kTiltVelocityFactor);
		tiltConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Algae.kTiltP)
				.i(Constants.Algae.kTiltI)
				.d(Constants.Algae.kTiltD)
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

		sbIntakeVel.setDouble(getIntakeVel());
		sbIntakeSP.setDouble(getIntakeSP());
		sbTiltPos.setDouble(getTiltPos());
		sbTxtTiltSP.setString(getTiltSP().toString());
		sbDblTiltSP.setDouble(getTiltSP().getValue());
		sbLimit.setBoolean(isLimit());
		sbExtract.setBoolean(getExtract());
	}

	/**
	 * setTiltCmd - command factory method to update the Tilt pos
	 * 
	 * @return a command
	 */
	public Command setTiltPosCmd(AlgaeSP pos) {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			setTiltPos(pos);
		});
	}

	public Command setTiltPosCmd() {
		return setTiltPosCmd(getTiltSP());
	}

	public Command setTiltSPCmd(AlgaeSP sp) {
		return runOnce(() -> {
			setTiltSP(sp);
		});
	}

	/**
	 * Example command factory method.
	 * 
	 * @param Intake motor velocity
	 * @return Command to set velocity
	 */
	public Command setIntakeCmd(double vel) {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			setIntakeVel(vel);
		});
	}

	/**
	 * Example command factory method.
	 *
	 * @return Command to set velocity using preset SP
	 */
	public Command setIntakeCmd() {
		return setIntakeCmd(getIntakeSP());
	}

	public Command setIntakeSPCmd(double sp) {
		return runOnce(() -> {
			setIntakeSP(sp);
		});
	}

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
		setIntakeSP(vel);
		intakeController.setReference(vel, SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public void setIntakeVel() {
		intakeController.setReference(getIntakeSP(), SparkBase.ControlType.kMAXMotionVelocityControl);
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

	/**
	 * doActionCmd - Command to perform doAction
	 */
	public Command doActionCmd() {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			doAction();
		});
	}

	public Command toggleExtractCmd() {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			toggleExtract();
		});
	}

	public Command algaeBarge() {
		return this.runOnce(() -> setTiltSP(Algae.AlgaeSP.BARGE));
	}

	public Command algaeL3() {
		return this.runOnce(() -> setTiltSP(Algae.AlgaeSP.L3));
	}

	public Command algaeL2() {
		return this.runOnce(() -> setTiltSP(Algae.AlgaeSP.L2));
	}

	public Command algaeFloor() {
		return this.runOnce(() -> setTiltSP(Algae.AlgaeSP.FLOOR));
	}

	public Command algaeStow() {
		return this.runOnce(() -> setTiltSP(Algae.AlgaeSP.STOW));
	}

	public Command algaeIntake() {
		return this.runOnce(() -> setIntakeSP(Constants.Algae.INTAKE));
	}

	public Command algaeEject() {
		return this.runOnce(() -> setIntakeSP(Constants.Algae.EJECT));
	}
}