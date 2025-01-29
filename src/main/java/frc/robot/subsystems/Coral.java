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
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Coral extends SubsystemBase {

	// Define Intake Motors
	private final SparkMax intake = new SparkMax(
			Constants.CANId.kCoralIntakeCanId, MotorType.kBrushless);
	private final SparkMax tilt = new SparkMax(
			Constants.CANId.kCoralTiltCanId, MotorType.kBrushless);

	private final SparkMaxConfig intakeConfig = new SparkMaxConfig();
	private final SparkMaxConfig tiltConfig = new SparkMaxConfig();

	private SparkClosedLoopController intakeController = intake.getClosedLoopController();
	private SparkClosedLoopController tiltController = tilt.getClosedLoopController();

	private RelativeEncoder intakeEncoder = intake.getEncoder();
	private AbsoluteEncoder tiltEncoder = tilt.getAbsoluteEncoder();

	private double intakeSP = Constants.Coral.STOP;
	private double tiltSP = Constants.Coral.STOW;

	private double prevIntakeSP = intakeSP;
	public VariableChangeTrigger intakeChanged = new VariableChangeTrigger(() -> getIntakeSPChanged());

	private double prevTiltSP = tiltSP;
	public VariableChangeTrigger tiltChanged = new VariableChangeTrigger(() -> getTiltSPChanged());

	private final ShuffleboardTab coralTab = Shuffleboard.getTab("Coral");
	private final GenericEntry sbIntakeVel = coralTab.addPersistent("Intake Vel", 0)
			.withWidget("Text View").withPosition(2, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbIntakeSP = coralTab.addPersistent("Intake SP", 0)
			.withWidget("Text View").withPosition(4, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbTiltPos = coralTab.addPersistent("Tilt Pos", 0)
			.withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbTiltSP = coralTab.addPersistent("Tilt SP", 0)
			.withWidget("Text View").withPosition(4, 1).withSize(2, 1).getEntry();

	// Creates a new Coral.
	public Coral() {
		System.out.println("+++++ Starting Coral Constructor +++++");

		// Configure Left Intake motor
		intakeConfig
				.inverted(Constants.Coral.kLeftMotorInverted)
				.idleMode(Constants.Coral.kIntakeIdleMode)
				.smartCurrentLimit(Constants.Coral.kLeftCurrentLimit);
		intakeConfig.encoder
				.positionConversionFactor(Constants.Coral.kIntakeEncoderPositionFactor)
				.velocityConversionFactor(Constants.Coral.kIntakeEncoderVelocityFactor);
		intakeConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pidf(Constants.Coral.kIntakeP,
						Constants.Coral.kIntakeI,
						Constants.Coral.kIntakeD,
						Constants.Coral.kIntakeFF)
				.outputRange(Constants.Coral.kIntakeMinOutput, Constants.Coral.kIntakeMaxOutput)
				.positionWrappingEnabled(Constants.Coral.kLeftEncodeWrapping);

		intake.configure(intakeConfig,
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

		setIntakeVel(intakeSP);
		setTiltPos(tiltSP);

		System.out.println("----- Ending Coral Constructor -----");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		setTiltSP(sbTiltSP.getDouble(0.0));
		setIntakeSP(sbIntakeSP.getDouble(0.0));
		
		sbIntakeVel.setDouble(getIntakeVel());
		sbIntakeSP.setDouble(getIntakeSP());
		sbTiltPos.setDouble(getTiltPos());
		sbTiltSP.setDouble(getTiltSP());
	}

	private boolean getTiltSPChanged() {
		double currTiltSP = getTiltSP();
		boolean changed = prevTiltSP != currTiltSP;
		prevTiltSP = currTiltSP;
		return changed;
	}

	private boolean getIntakeSPChanged() {
		double currIntakeSP = getIntakeSP();
		boolean changed = prevIntakeSP != currIntakeSP;
		prevIntakeSP = currIntakeSP;
		return changed;
	}

	public double getIntakeVel() {
		return intakeEncoder.getVelocity();
	}

	public double getTiltPos() {
		return tiltEncoder.getPosition();
	}

	public void setTiltPos(double pos) {
		setTiltSP(pos);
		tiltController.setReference(pos, SparkBase.ControlType.kMAXMotionPositionControl);
	}

	public void setIntakeVel(double vel) {
		setIntakeSP(vel);
		intakeController.setReference(vel, SparkBase.ControlType.kMAXMotionVelocityControl);
	}

	public void setIntakeSP(double sp) {
		intakeSP = sp;
	}

	public double getIntakeSP() {
		return intakeSP;
	}

	public void setTiltSP(double sp) {
		tiltSP = sp;
	}

	public double getTiltSP() {
		return tiltSP;
	}
}
