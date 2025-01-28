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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

	private final SparkMaxConfig leftConfig = new SparkMaxConfig();
	private final SparkMaxConfig rightConfig = new SparkMaxConfig();
	private final SparkMaxConfig tiltConfig = new SparkMaxConfig();

	private SparkClosedLoopController tiltController = tilt.getClosedLoopController();
	private SparkClosedLoopController leftIntakeController = tilt.getClosedLoopController();
	private SparkClosedLoopController rightIntakeController = tilt.getClosedLoopController();

	private RelativeEncoder leftEncoder = null;
	private RelativeEncoder rightEncoder = null;
	private AbsoluteEncoder tiltEncoder = null;

	private double tiltSP = Constants.Coral.STOW;
	private double intakeSP = Constants.Coral.STOP;

	private double prevTiltSP = 0.0;
	public VariableChangeTrigger tiltChanged = new VariableChangeTrigger(() -> getTiltSPChanged());

	private double prevIntakeSP = 0.0;
	public VariableChangeTrigger intakeChanged = new VariableChangeTrigger(() -> getIntakeSPChanged());

	private final ShuffleboardTab coralTab = Shuffleboard.getTab("Coral");
	private final GenericEntry sbLeftVel = coralTab.addPersistent("Left Vel", 0)
			.withWidget("Text View").withPosition(2, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbRightVel = coralTab.addPersistent("Right Vel", 0)
			.withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbIntakeSP = coralTab.addPersistent("Intake SP", 0)
			.withWidget("Text View").withPosition(4, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbTiltPos = coralTab.addPersistent("Tilt Pos", 0)
			.withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbTiltSP = coralTab.addPersistent("Tilt Pos SP", 0)
			.withWidget("Text View").withPosition(4, 1).withSize(2, 1).getEntry();

	// Creates a new Coral.
	public Coral() {
		System.out.println("+++++ Starting Coral Constructor +++++");

		// Configure Left Intake motor
		leftConfig
				.inverted(Constants.Coral.kLeftMotorInverted)
				.idleMode(Constants.Coral.kLeftIdleMode)
				.smartCurrentLimit(Constants.Coral.kLeftCurrentLimit);
		leftConfig.encoder
				.positionConversionFactor(Constants.Coral.kLeftEncoderPositionFactor)
				.velocityConversionFactor(Constants.Coral.kLeftEncoderVelocityFactor);
		leftConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pidf(Constants.Coral.kLeftP,
						Constants.Coral.kLeftI,
						Constants.Coral.kLeftD,
						Constants.Coral.kLeftFF)
				.outputRange(Constants.Coral.kLeftMinOutput, Constants.Coral.kLeftMaxOutput)
				.positionWrappingEnabled(Constants.Coral.kLeftEncodeWrapping);

		leftIntake.configure(leftConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Right Intake motor
		rightConfig
				.inverted(Constants.Coral.kRightMotorInverted)
				.idleMode(Constants.Coral.kRightIdleMode)
				.smartCurrentLimit(Constants.Coral.kRightCurrentLimit);
		rightConfig.encoder
				.positionConversionFactor(Constants.Coral.kRightEncoderPositionFactor)
				.velocityConversionFactor(Constants.Coral.kRightEncoderVelocityFactor);
		rightConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pidf(Constants.Coral.kRightP,
						Constants.Coral.kRightI,
						Constants.Coral.kRightD,
						Constants.Coral.kRightFF)
				.outputRange(Constants.Coral.kRightMinOutput, Constants.Coral.kRightMaxOutput)
				.positionWrappingEnabled(Constants.Coral.kRightEncodeWrapping);

		rightIntake.configure(rightConfig,
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
		
		SmartDashboard.putData(this);

		System.out.println("----- Ending Coral Constructor -----");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		sbLeftVel.setDouble(getLeftVel());
		sbRightVel.setDouble(getRightVel());
		sbIntakeSP.setDouble(getIntakeSP());
		sbTiltPos.setDouble(getTiltPos());
		sbTiltSP.setDouble(getTiltSP());

		setTiltSP(sbTiltSP.getDouble(0.0));
		setIntakeSP(sbIntakeSP.getDouble(0.0));
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

	public double getLeftVel() {
		return leftEncoder.getVelocity();
	}

	public double getRightVel() {
		return rightEncoder.getVelocity();
	}

	public double getTiltPos() {
		return tiltEncoder.getPosition();
	}

	public void setTiltPos(double pos) {
		tiltController.setReference(pos, SparkBase.ControlType.kMAXMotionPositionControl);
	}

	public void setIntakeVel(double vel) {
		leftIntakeController.setReference(vel, SparkBase.ControlType.kMAXMotionVelocityControl);
		rightIntakeController.setReference(vel, SparkBase.ControlType.kMAXMotionVelocityControl);
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
