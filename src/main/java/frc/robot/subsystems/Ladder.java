package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
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

public class Ladder extends SubsystemBase {

	// Define Intake Motors
	private final SparkMax leftLadder = new SparkMax(
			Constants.CANId.kLadderLeftCanId, MotorType.kBrushless);
	private final SparkMax rightLadder = new SparkMax(
			Constants.CANId.kLadderRightCanId, MotorType.kBrushless);

	private final SparkMaxConfig leftConfig = new SparkMaxConfig();
	private final SparkMaxConfig rightConfig = new SparkMaxConfig();

	private SparkClosedLoopController leftController = leftLadder.getClosedLoopController();
	private SparkClosedLoopController rightController = rightLadder.getClosedLoopController();

	private RelativeEncoder leftEncoder = leftLadder.getEncoder();
	private RelativeEncoder rightEncoder = rightLadder.getEncoder();

	private double ladderSP = Constants.Ladder.STOW;

	private double prevLadderSP = ladderSP;
	public VariableChangeTrigger ladderChanged = new VariableChangeTrigger(() -> getLadderSPChanged());

	private final ShuffleboardTab ladderTab = Shuffleboard.getTab("Ladder");
	private final GenericEntry sbLeftPos = ladderTab.addPersistent("Left Pos", 0)
			.withWidget("Text View").withPosition(2, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbRightPos = ladderTab.addPersistent("Right Pos", 0)
			.withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbLadderSP = ladderTab.addPersistent("Ladder SP", 0)
			.withWidget("Text View").withPosition(4, 0).withSize(2, 1).getEntry();

	// Creates a new Ladder.
	public Ladder() {
		System.out.println("+++++ Starting Ladder Constructor +++++");

		// Configure Left Intake motor
		leftConfig
				.inverted(Constants.Ladder.kLeftMotorInverted)
				.idleMode(Constants.Ladder.kLeftIdleMode)
				.smartCurrentLimit(Constants.Ladder.kLeftCurrentLimit);
		leftConfig.encoder
				.positionConversionFactor(Constants.Ladder.kLeftEncoderPositionFactor)
				.velocityConversionFactor(Constants.Ladder.kLeftEncoderVelocityFactor);
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
				.positionWrappingEnabled(Constants.Ladder.kLeftEncodeWrapping);

		leftLadder.configure(leftConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Right Intake motor
		rightConfig
				.inverted(Constants.Ladder.kRightMotorInverted)
				.idleMode(Constants.Ladder.kRightIdleMode)
				.smartCurrentLimit(Constants.Ladder.kRightCurrentLimit);
		rightConfig.encoder
				.positionConversionFactor(Constants.Ladder.kRightEncoderPositionFactor)
				.velocityConversionFactor(Constants.Ladder.kRightEncoderVelocityFactor);
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
				.positionWrappingEnabled(Constants.Ladder.kRightEncodeWrapping);

		rightLadder.configure(rightConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Initialize intake start positions
		setLadderPos(ladderSP);

		System.out.println("----- Ending Ladder Constructor -----");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		sbLeftPos.setDouble(getLeftPos());
		sbRightPos.setDouble(getRightPos());
		sbLadderSP.setDouble(getLadderSP());

		setLadderSP(sbLadderSP.getDouble(0.0));
	}

	private boolean getLadderSPChanged() {
		double currLadderSP = getLadderSP();
		boolean changed = prevLadderSP != currLadderSP;
		prevLadderSP = currLadderSP;
		return changed;
	}

	// public boolean isLeftOnTarget() {
	// return leftController.atSetpoint();
	// }

	public double getLeftPos() {
		return leftEncoder.getPosition();
	}

	public double getRightPos() {
		return rightEncoder.getPosition();
	}

	public void setLadderPos(double pos) {
		leftController.setReference(pos,
				SparkBase.ControlType.kMAXMotionPositionControl);
		rightController.setReference(pos,
				SparkBase.ControlType.kMAXMotionPositionControl);
	}

	public void setLadderSP(double sp) {
		ladderSP = sp;
	}

	public double getLadderSP() {
		return ladderSP;
	}
}
