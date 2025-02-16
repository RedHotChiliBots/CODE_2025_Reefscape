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
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
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

	private SparkLimitSwitch isLimitSwitch = leftLadder.getForwardLimitSwitch(); // leftLadder.getReverseLimitSwitch();

	// define ladder positions
	public enum LadderSP {
		BARGE(8.0 + (5.0 / 12.0)),
		L4(6.0 + (0.0 / 12.0)),
		L3(3.0 + (11.625 / 12.0)),
		L2(2.0 + (7.825 / 12.0)),
		L1(1.0 + (6.0 / 12.0)),
		STATION(3.0 + (1.5 / 12.0)),
		PROCESSOR(0.0 + (7.0 / 12.0)),
		FLOOR(0.5),
		STOW(0.5),
		START(0.0);

		private final double sp;

		LadderSP(final double sp) {
			this.sp = sp;
		}

		public double getValue() {
			return sp;
		}
	}

	private LadderSP ladderSP = LadderSP.STOW;

	private boolean firstPeriod = true;
	private boolean zeroingLadder = false;

	private final ShuffleboardTab ladderTab = Shuffleboard.getTab("Ladder");

	private final GenericEntry sbTxtSP = ladderTab.addPersistent("Ladder tSP", "")
			.withWidget("Text View").withPosition(1, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbDblSP = ladderTab.addPersistent("Ladder dSP", 0)
			.withWidget("Text View").withPosition(2, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbLadderPos = ladderTab.addPersistent("Ladder Pos", 0)
			.withWidget("Text View").withPosition(3, 0)
			.withSize(1, 1).getEntry();
	private final GenericEntry sbLimit = ladderTab.addPersistent("Ladder Limit", false)
			.withWidget("Boolean Box").withPosition(4, 0)
			.withSize(1, 1).getEntry();

	// Creates a new Ladder.
	public Ladder() {
		System.out.println("+++++ Starting Ladder Constructor +++++");

		// Configure Left Intake motor
		leftConfig
				.inverted(Constants.Ladder.kLeftMotorInverted)
				.idleMode(Constants.Ladder.kLeftIdleMode)
				.smartCurrentLimit(Constants.Ladder.kLeftCurrentLimit);
		leftConfig.encoder
				.positionConversionFactor(Constants.Ladder.kLiftPostionFactor)
				.velocityConversionFactor(Constants.Ladder.kLiftVelocityFactor);
		leftConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.Ladder.kLeftPosP)
				.i(Constants.Ladder.kLeftPosI)
				.d(Constants.Ladder.kLeftPosD)
				.outputRange(Constants.Ladder.kLeftPosMinOutput, Constants.Ladder.kLeftPosMaxOutput)

				.p(Constants.Ladder.kLeftVelP, ClosedLoopSlot.kSlot1)
				.i(Constants.Ladder.kLeftVelI, ClosedLoopSlot.kSlot1)
				.d(Constants.Ladder.kLeftVelD, ClosedLoopSlot.kSlot1)
				// .velocityFF(Constants.Ladder.kLeftVelFF, ClosedLoopSlot.kSlot1)
				.outputRange(Constants.Ladder.kLeftVelMinOutput, Constants.Ladder.kLeftVelMaxOutput,
						ClosedLoopSlot.kSlot1)
				.positionWrappingEnabled(Constants.Ladder.kLeftEncodeWrapping);

		leftLadder.configure(leftConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configure Right Intake motor
		rightConfig
				.follow(leftLadder, true)
				.inverted(Constants.Ladder.kRightMotorInverted)
				.idleMode(Constants.Ladder.kRightIdleMode)
				.smartCurrentLimit(Constants.Ladder.kRightCurrentLimit);
		// rightConfig.encoder
		// .positionConversionFactor(Constants.Ladder.kRightEncoderPositionFactor)
		// .velocityConversionFactor(Constants.Ladder.kRightEncoderVelocityFactor);
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
		// .positionWrappingEnabled(Constants.Ladder.kRightEncodeWrapping);

		rightLadder.configure(rightConfig,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Initialize intake start positions
		setLadderPos(ladderSP);

		System.out.println("----- Ending Ladder Constructor -----");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		sbTxtSP.setString(getLadderSP().toString());
		sbDblSP.setDouble(getLadderSP().getValue());
		sbLadderPos.setDouble(getLeftPos());
		sbLimit.setBoolean(isLimit());

		if (firstPeriod || zeroingLadder) {
			zeroLadder();
		}
	}

	private void zeroLadder() {
		if (firstPeriod) {
			leftLadder.set(Constants.Ladder.DOWN);
			firstPeriod = false;
			zeroingLadder = true;
		}

		if (isLimit()) {
			leftLadder.set(Constants.Ladder.STOP);
			leftEncoder.setPosition(LadderSP.START.getValue());
			setLadderPos(LadderSP.STOW);
			zeroingLadder = false;
		}
	}

	/**
	 * setTiltCmd - command factory method to update the Tilt pos
	 * 
	 * @return a command
	 */
	public Command setLadderSPCmd(LadderSP sp) {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			setLadderSP(sp);
		});
	}

	/**
	 * setTiltCmd - command factory method to update the Tilt pos
	 * 
	 * @return a command
	 */
	public Command setLadderPosCmd(LadderSP pos) {
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			setLadderPos(pos);
		});
	}

	public Command setLadderPosCmd() {
		return setLadderPosCmd(getLadderSP());
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

	public void setLadderPos(LadderSP pos) {
		setLadderSP(pos);
		leftController.setReference(pos.getValue(),
				SparkBase.ControlType.kMAXMotionPositionControl);
		rightController.setReference(pos.getValue(),
				SparkBase.ControlType.kMAXMotionPositionControl);
	}

	public void setLadderPos() {
		leftController.setReference(
				getLadderSP().getValue(),
				SparkBase.ControlType.kMAXMotionPositionControl);
		rightController.setReference(
				getLadderSP().getValue(),
				SparkBase.ControlType.kMAXMotionPositionControl);
	}

	public void setLadderSP(LadderSP sp) {
		ladderSP = sp;
	}

	public LadderSP getLadderSP() {
		return ladderSP;
	}

	public boolean isLimit() {
		return isLimitSwitch.isPressed(); // || rightForLimitSwitch.isPressed();
	}
}
