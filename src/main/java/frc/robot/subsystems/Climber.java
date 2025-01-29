package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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

public class Climber extends SubsystemBase {

	// Define Intake Motors
	private final SparkMax leftClimber = new SparkMax(
			Constants.CANId.kClimberLeftCanId, MotorType.kBrushless);
	private final SparkMax rightClimber = new SparkMax(
			Constants.CANId.kClimberRightCanId, MotorType.kBrushless);

	private final SparkMaxConfig leftConfig = new SparkMaxConfig();
	private final SparkMaxConfig rightConfig = new SparkMaxConfig();

	private SparkClosedLoopController leftController = leftClimber.getClosedLoopController();
	private SparkClosedLoopController rightController = rightClimber.getClosedLoopController();

	private AbsoluteEncoder leftEncoder = leftClimber.getAbsoluteEncoder();
	private AbsoluteEncoder rightEncoder = rightClimber.getAbsoluteEncoder();

	private double climberSP = Constants.Ladder.STOW;

	private double prevClimberSP = climberSP;
	public VariableChangeTrigger climberChanged = new VariableChangeTrigger(() -> getClimberSPChanged());

	private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
	private final GenericEntry sbLeftPos = climberTab.addPersistent("Left Pos", 0)
			.withWidget("Text View").withPosition(2, 0)
			.withSize(2, 1).getEntry();
	private final GenericEntry sbRightPos = climberTab.addPersistent("Right Pos", 0)
			.withWidget("Text View").withPosition(2, 1)
			.withSize(2, 1).getEntry();
	private final GenericEntry sbClimberSP = climberTab.addPersistent("Climber SP", 0)
			.withWidget("Text View").withPosition(4, 0)
			.withSize(2, 1).getEntry();

	// Creates a new Climber.
	public Climber() {
		System.out.println("+++++ Starting Climber Constructor +++++");

		// Configure Left Intake motor
		leftConfig
				.inverted(Constants.Climber.kLeftMotorInverted)
				.idleMode(Constants.Climber.kLeftIdleMode)
				.smartCurrentLimit(Constants.Climber.kLeftCurrentLimit);
		leftConfig.encoder
				.positionConversionFactor(Constants.Climber.kLeftEncoderPositionFactor)
				.velocityConversionFactor(Constants.Climber.kLeftEncoderVelocityFactor);
		leftConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Climber.kLeftPosP)
				.i(Constants.Climber.kLeftPosI)
				.d(Constants.Climber.kLeftPosD)
				.outputRange(Constants.Climber.kLeftPosMinOutput, Constants.Climber.kLeftPosMaxOutput)

				.p(Constants.Climber.kLeftVelP, ClosedLoopSlot.kSlot1)
				.i(Constants.Climber.kLeftVelI, ClosedLoopSlot.kSlot1)
				.d(Constants.Climber.kLeftVelD, ClosedLoopSlot.kSlot1)
				.velocityFF(Constants.Climber.kLeftVelFF, ClosedLoopSlot.kSlot1)
				.outputRange(Constants.Climber.kLeftVelMinOutput, Constants.Climber.kLeftVelMaxOutput,
						ClosedLoopSlot.kSlot1)
				.positionWrappingEnabled(Constants.Climber.kLeftEncodeWrapping);

		leftClimber.configure(leftConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Right Intake motor
		rightConfig
				.inverted(Constants.Climber.kRightMotorInverted)
				.idleMode(Constants.Climber.kRightIdleMode)
				.smartCurrentLimit(Constants.Climber.kRightCurrentLimit);
		rightConfig.encoder
				.positionConversionFactor(Constants.Climber.kRightEncoderPositionFactor)
				.velocityConversionFactor(Constants.Climber.kRightEncoderVelocityFactor);
		rightConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Climber.kRightPosP)
				.i(Constants.Climber.kRightPosI)
				.d(Constants.Climber.kRightPosD)
				.outputRange(Constants.Climber.kRightPosMinOutput, Constants.Climber.kRightPosMaxOutput)

				.p(Constants.Climber.kRightVelP, ClosedLoopSlot.kSlot1)
				.i(Constants.Climber.kRightVelI, ClosedLoopSlot.kSlot1)
				.d(Constants.Climber.kRightVelD, ClosedLoopSlot.kSlot1)
				.velocityFF(Constants.Climber.kRightVelFF, ClosedLoopSlot.kSlot1)
				.outputRange(Constants.Climber.kRightVelMinOutput, Constants.Climber.kRightVelMaxOutput,
						ClosedLoopSlot.kSlot1)
				.positionWrappingEnabled(Constants.Climber.kRightEncodeWrapping);

		rightClimber.configure(rightConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Initialize intake start positions
		setClimberPos(Constants.Climber.STOW);

		System.out.println("----- Ending Climber Constructor -----");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		setClimberSP(sbClimberSP.getDouble(0.0));
		
		sbLeftPos.setDouble(getLeftPos());
		sbRightPos.setDouble(getRightPos());
		sbClimberSP.setDouble(getClimberSP());
	}

	private boolean getClimberSPChanged() {
		double currClimberSP = getClimberSP();
		boolean changed = prevClimberSP != currClimberSP;
		prevClimberSP = currClimberSP;
		return changed;
	}

	public double getLeftPos() {
		return leftEncoder.getPosition();
	}

	public double getRightPos() {
		return rightEncoder.getPosition();
	}

	public void setClimberPos(double pos) {
		setClimberSP(pos);
		leftController.setReference(pos,
				SparkBase.ControlType.kMAXMotionPositionControl);
		rightController.setReference(pos,
				SparkBase.ControlType.kMAXMotionPositionControl);
	}
	
	public void setClimberSP(double sp) {
		climberSP = sp;
	}

	public double getClimberSP() {
		return climberSP;
	}
}
