package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
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

public class Climber extends SubsystemBase {

	// Define Intake Motors
	private final SparkMax leftClimber = new SparkMax(
			Constants.CANId.kClimberLeftCanId, MotorType.kBrushless);
	private final SparkMax rightClimber = new SparkMax(
			Constants.CANId.kClimberRightCanId, MotorType.kBrushless);

	private final SparkMaxConfig leftConfig = new SparkMaxConfig();
	private final SparkMaxConfig rightConfig = new SparkMaxConfig();

	private SparkClosedLoopController leftController = leftClimber.getClosedLoopController();
	// private SparkClosedLoopController rightController = rightClimber.getClosedLoopController();

	private AbsoluteEncoder leftEncoder = leftClimber.getAbsoluteEncoder();
	// private AbsoluteEncoder rightEncoder = rightClimber.getAbsoluteEncoder();

	private SparkLimitSwitch isLimitSwitch = leftClimber.getForwardLimitSwitch();	// leftClimber.getReverseLimitSwitch();

	public enum ClimberSP {
		STOW(90.0), // degrees
		READY(0.0), // degrees
		CLIMB(-25.0); // degrees

		private final double sp;

		ClimberSP(final double sp) {
			this.sp = sp;
		}

		public double getValue() {
			return sp;
		}
	}

	private ClimberSP climberSP = Climber.ClimberSP.STOW;

	private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
	private final GenericEntry sbTxtSP = climberTab.addPersistent("Climber tSP", "")
			.withWidget("Text View").withPosition(1, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbDblSP = climberTab.addPersistent("Climber dSP", 0)
			.withWidget("Text View").withPosition(2, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbLeftPos = climberTab.addPersistent("Climber Pos", 0)
			.withWidget("Text View").withPosition(3, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbLimit = climberTab.addPersistent("Climber Limit", false)
			.withWidget("Boolean Box").withPosition(4, 0)
			.withSize(1, 1).getEntry();

	// Creates a new Climber.
	public Climber() {
		System.out.println("+++++ Starting Climber Constructor +++++");

		// Configure Left Intake motor
		leftConfig
				.inverted(Constants.Climber.kLeftMotorInverted)
				.idleMode(Constants.Climber.kLeftIdleMode)
				.smartCurrentLimit(Constants.Climber.kLeftCurrentLimit);
		leftConfig.encoder
				.positionConversionFactor(Constants.Climber.kTiltPositionFactor)
				.velocityConversionFactor(Constants.Climber.kTiltVelocityFactor);
		leftConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Climber.kLeftPosP)
				.i(Constants.Climber.kLeftPosI)
				.d(Constants.Climber.kLeftPosD)
				.outputRange(Constants.Climber.kLeftPosMinOutput, Constants.Climber.kLeftPosMaxOutput)
				.positionWrappingEnabled(Constants.Climber.kLeftEncodeWrapping);

		leftClimber.configure(leftConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Right Intake motor
		rightConfig
				.follow(leftClimber, true)
				.inverted(Constants.Climber.kRightMotorInverted)
				.idleMode(Constants.Climber.kRightIdleMode)
				.smartCurrentLimit(Constants.Climber.kRightCurrentLimit);
		// rightConfig.encoder
		// .positionConversionFactor(Constants.Climber.kRightEncoderPositionFactor)
		// .velocityConversionFactor(Constants.Climber.kRightEncoderVelocityFactor);
		// rightConfig.closedLoop
		// .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
		// .p(Constants.Climber.kRightPosP)
		// .i(Constants.Climber.kRightPosI)
		// .d(Constants.Climber.kRightPosD)
		// .outputRange(Constants.Climber.kRightPosMinOutput,
		// Constants.Climber.kRightPosMaxOutput)

		// .p(Constants.Climber.kRightVelP, ClosedLoopSlot.kSlot1)
		// .i(Constants.Climber.kRightVelI, ClosedLoopSlot.kSlot1)
		// .d(Constants.Climber.kRightVelD, ClosedLoopSlot.kSlot1)
		// .velocityFF(Constants.Climber.kRightVelFF, ClosedLoopSlot.kSlot1)
		// .outputRange(Constants.Climber.kRightVelMinOutput,
		// Constants.Climber.kRightVelMaxOutput,
		// ClosedLoopSlot.kSlot1)
		// .positionWrappingEnabled(Constants.Climber.kRightEncodeWrapping);

		rightClimber.configure(rightConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Initialize intake start positions
		setClimberPos(Climber.ClimberSP.STOW);

		System.out.println("----- Ending Climber Constructor -----");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		sbLeftPos.setDouble(getLeftPos());
		sbTxtSP.setString(getClimberSP().toString());
		sbDblSP.setDouble(getClimberSP().getValue());
		sbLimit.setBoolean(getLimitSwitch());
	}

	public Command setClimberCmd(ClimberSP pos) {
		// Inline construction of command goes here.
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(
				() -> {
					setClimberPos(pos);
				});
	}

	public Command setClimberCmd() {
		// Inline construction of command goes here.
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(
				() -> {
					setClimberPos(getClimberSP());
				});
	}

	public double getLeftPos() {
		return leftEncoder.getPosition();
	}

	// public double getRightPos() {
	// 	return rightEncoder.getPosition();
	// }

	public void setClimberPos(ClimberSP pos) {
		setClimberSP(pos);
		leftController.setReference(pos.getValue(),
				SparkBase.ControlType.kMAXMotionPositionControl);
		// rightController.setReference(pos,
		// SparkBase.ControlType.kMAXMotionPositionControl);
	}

	public void setClimberSP(ClimberSP sp) {
		climberSP = sp;
	}

	public ClimberSP getClimberSP() {
		return climberSP;
	}

	public boolean getLimitSwitch() {
		return isLimitSwitch.isPressed(); // || rightForLimitSwitch.isPressed();
	}

	public Command climberClimb() {
		return this.runOnce(() -> setClimberPos(ClimberSP.CLIMB));
	}

	public Command climberReady() {
		return this.runOnce(() -> setClimberPos(ClimberSP.READY));
	}

	public Command climberStow() {
		return this.runOnce(() -> setClimberPos(ClimberSP.STOW));
	}
}
