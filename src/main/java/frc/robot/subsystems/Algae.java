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

        private RelativeEncoder leftEncoder = null;
        private RelativeEncoder rightEncoder = null;
        private AbsoluteEncoder tiltEncoder = null;

        private final ShuffleboardTab algaeTab = Shuffleboard.getTab("Algae");
        private final GenericEntry sbLeftPos = algaeTab.addPersistent("Left Pos", 0)
                        .withWidget("Text View").withPosition(2, 0).withSize(2, 1).getEntry();
        private final GenericEntry sbLeftPosSP = algaeTab.addPersistent("Left Pos SP", 0)
                        .withWidget("Text View").withPosition(4, 0).withSize(2, 1).getEntry();
        private final GenericEntry sbRightPos = algaeTab.addPersistent("Right Pos", 0)
                        .withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
        private final GenericEntry sbRightPosSP = algaeTab.addPersistent("Right Pos SP", 0)
                        .withWidget("Text View").withPosition(4, 1).withSize(2, 1).getEntry();
        private final GenericEntry sbTiltPos = algaeTab.addPersistent("Tilt Pos", 0)
                        .withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
        private final GenericEntry sbTiltPosSP = algaeTab.addPersistent("Tilt Pos SP", 0)
                        .withWidget("Text View").withPosition(4, 1).withSize(2, 1).getEntry();

        // Creates a new Algae.
        public Algae() {
                System.out.println("+++++ Starting Algae Constructor +++++");

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
                                .inverted(Constants.Algae.kRightMotorInverted)
                                .idleMode(Constants.Algae.kRightIdleMode)
                                .smartCurrentLimit(Constants.Algae.kRightCurrentLimit);
                rightConfig.encoder
                                .positionConversionFactor(Constants.Algae.kRightEncoderPositionFactor)
                                .velocityConversionFactor(Constants.Algae.kRightEncoderVelocityFactor);
                rightConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(Constants.Algae.kRightP,
                                                Constants.Algae.kRightI,
                                                Constants.Algae.kRightD,
                                                Constants.Algae.kRightFF)
                                .outputRange(Constants.Algae.kRightMinOutput, Constants.Algae.kRightMaxOutput)
                                .positionWrappingEnabled(Constants.Algae.kRightEncodeWrapping);

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

                Stop();
                SetTiltPos(Constants.Algae.STOW);

                System.out.println("----- Ending Algae Constructor -----");
        }

        @Override
        public void periodic() {
                // This method will be called once per scheduler run SuffleBoard
        }

        // Getting the position of the encoders
        public double GetLeftVel() {
                return leftEncoder.getVelocity();
        }

        public double GetRightVel() {
                return rightEncoder.getVelocity();
        }

        public double GetTiltPos() {
                return tiltEncoder.getPosition();
        }

        // Sets the position of the encoders
        public void SetTiltPos(double pos) {
                tiltController.setReference(pos, SparkBase.ControlType.kMAXMotionPositionControl);
        }

        public void Stop() {
                leftIntake.set(Constants.Algae.STOP);
                rightIntake.set(Constants.Algae.STOP);
        }

        public void Intake() {
                leftIntake.set(Constants.Algae.INTAKE);
                rightIntake.set(Constants.Algae.INTAKE);
        }

        public void Eject() {
                leftIntake.set(Constants.Algae.EJECT);
                rightIntake.set(Constants.Algae.EJECT);
        }
}