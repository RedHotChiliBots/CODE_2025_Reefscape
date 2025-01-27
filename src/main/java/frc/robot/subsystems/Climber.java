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
        private final SparkMax leftIntake = new SparkMax(
                        Constants.CANId.kClimberLeftCanId, MotorType.kBrushless);
        private final SparkMax rightIntake = new SparkMax(
                        Constants.CANId.kClimberRightCanId, MotorType.kBrushless);

        private final SparkMaxConfig leftConfig = new SparkMaxConfig();
        private final SparkMaxConfig rightConfig = new SparkMaxConfig();

        private SparkClosedLoopController leftController = leftIntake.getClosedLoopController();
        private SparkClosedLoopController rightController = rightIntake.getClosedLoopController();

        private AbsoluteEncoder leftEncoder = null;
        private AbsoluteEncoder rightEncoder = null;

        private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
        private final GenericEntry sbLeftPos = climberTab.addPersistent("Left Pos", 0)
                        .withWidget("Text View").withPosition(2, 0)
                        .withSize(2, 1).getEntry();
        private final GenericEntry sbLeftPosSP = climberTab.addPersistent("Left Pos SP", 0)
                        .withWidget("Text View").withPosition(4, 0)
                        .withSize(2, 1).getEntry();
        private final GenericEntry sbRightPos = climberTab.addPersistent("Right Pos", 0)
                        .withWidget("Text View").withPosition(2, 1)
                        .withSize(2, 1).getEntry();
        private final GenericEntry sbRightPosSP = climberTab.addPersistent("Right Pos SP", 0)
                        .withWidget("Text View").withPosition(4, 1)
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

                leftIntake.configure(leftConfig,
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

                rightIntake.configure(rightConfig,
                                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                // Initialize intake start positions
                SetPos(Constants.Climber.STOW);

                System.out.println("----- Ending Climber Constructor -----");
        }

        @Override
        public void periodic() {
                // This method will be called once per scheduler run
        }

        public double GetLeftPos() {
                return leftEncoder.getPosition();
        }

        public double GetRightPos() {
                return rightEncoder.getPosition();
        }

        public void SetPos(double pos) {
                leftController.setReference(pos,
                                SparkBase.ControlType.kMAXMotionPositionControl);
                rightController.setReference(pos,
                                SparkBase.ControlType.kMAXMotionPositionControl);
        }
}
