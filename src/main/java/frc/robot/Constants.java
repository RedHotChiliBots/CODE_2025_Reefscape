// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class CANId {
    // Drive/Turn CAN IDs
    public static final int kRearRightDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 11;
    public static final int kFrontLeftDrivingCanId = 12;
    public static final int kFrontRightDrivingCanId = 13;

    public static final int kRearRightTurningCanId = 15;
    public static final int kRearLeftTurningCanId = 16;
    public static final int kFrontLeftTurningCanId = 17;
    public static final int kFrontRightTurningCanId = 18;

    public static final int kCoralLeftIntakeCanId = 20;
    public static final int kCoralRightIntakeCanId = 21;
    public static final int kCoralTiltCanId = 22;

    public static final int kAlgaeLeftIntakeCanId = 30;
    public static final int kAlgaeRightIntakeCanId = 31;
    public static final int kAlgaeTiltCanId = 32;

    public static final int kClimberLeftIntakeCanId = 40;
    public static final int kClimberRightIntakeCanId = 41;

    public static final int kLadderLeftIntakeCanId = 50;
    public static final int kLadderRightIntakeCanId = 51;
  }

  public static final class MotorConstants {
    public static final double kVortexFreeSpeedRpm = 6784;
    public static final double kNeoFreeSpeedRpm = 5676;
    public static final double k550FreeSpeedRpm = 11000;
  }

  public static final class Coral {

    // define coral positions
    public static final double STOW = 1.0;
    public static final double STATION = 1.0;
    public static final double L1 = 1.0;
    public static final double L2 = 1.0;
    public static final double L3 = 1.0;
    public static final double L4 = 1.0;

    // define coral velocities
    public static final double STOP = 0.0;
    public static final double INTAKE = 1.0;
    public static final double EJECT = -1.0;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kLeftMotorPinionTeeth = 14;

    // Invert the right encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kRightEncoderInverted = false;

    public static final boolean kRightMotorInverted = false;
    public static final boolean kLeftMotorInverted = true;
    public static final boolean kTiltMotorInverted = false;

    public static final boolean kLeftEncodeWrapping = false;
    public static final boolean kRightEncodeWrapping = false;
    public static final boolean kTiltEncodeWrapping = false;

    // Calculations required for left motor conversion factors and feed forward
    public static final double kLeftMotorFreeSpeedRps = MotorConstants.k550FreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.feetToMeters(0.33333);// 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kLeftMotorReduction = (45.0 * 22) / (kLeftMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kLeftMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kLeftMotorReduction;

    public static final double kLeftEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kLeftMotorReduction; // meters
    public static final double kLeftEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kLeftMotorReduction) / 60.0; // meters per second

    public static final double kRightEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kRightEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kRightEncoderPositionPIDMinInput = 0; // radians
    public static final double kRightEncoderPositionPIDMaxInput = kRightEncoderPositionFactor; // radians

    public static final double kTiltEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTiltEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kTiltEncoderPositionPIDMinInput = 0; // radians
    public static final double kTiltEncoderPositionPIDMaxInput = kRightEncoderPositionFactor; // radians

    public static final double kLeftP = 0.04;
    public static final double kLeftI = 0;
    public static final double kLeftD = 0;
    public static final double kLeftFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kLeftMinOutput = -1;
    public static final double kLeftMaxOutput = 1;

    public static final double kRightP = 1;
    public static final double kRightI = 0;
    public static final double kRightD = 0;
    public static final double kRightFF = 0;
    public static final double kRightMinOutput = -1;
    public static final double kRightMaxOutput = 1;

    public static final double kTiltP = 1;
    public static final double kTiltI = 0;
    public static final double kTiltD = 0;
    public static final double kTiltFF = 0;
    public static final double kTiltMinOutput = -1;
    public static final double kTiltMaxOutput = 1;

    public static final IdleMode kLeftIdleMode = IdleMode.kBrake;
    public static final IdleMode kRightIdleMode = IdleMode.kBrake;
    public static final IdleMode kTiltIdleMode = IdleMode.kBrake;

    public static final int kLeftCurrentLimit = 20; // amps
    public static final int kRightCurrentLimit = 20; // amps
    public static final int kTiltCurrentLimit = 20; // amps
  }

  public static final class Algae {

    // define algae positions
    public static final double STOW = 1.0;
    public static final double FLOOR = 1.0;
    public static final double L1 = 1.0;
    public static final double L2 = 1.0;
    public static final double L3 = 1.0;
    public static final double L4 = 1.0;
    public static final double BARGE = 1.0;

    // define algae velocities
    public static final double STOP = 0.0;
    public static final double INTAKE = 1.0;
    public static final double EJECT = -1.0;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kLeftMotorPinionTeeth = 14;

    // Invert the right encoder, since the output shaft rotates in the opvelite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kRightEncoderInverted = false;

    public static final boolean kRightMotorInverted = false;
    public static final boolean kLeftMotorInverted = true;
    public static final boolean kTiltMotorInverted = false;

    public static final boolean kLeftEncodeWrapping = false;
    public static final boolean kRightEncodeWrapping = false;
    public static final boolean kTiltEncodeWrapping = false;

    // Calculations required for left motor conversion factors and feed forward
    public static final double kLeftMotorFreeSpeedRps = MotorConstants.k550FreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.feetToMeters(0.33333);// 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kLeftMotorReduction = (45.0 * 22) / (kLeftMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kLeftMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kLeftMotorReduction;

    public static final double kLeftEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kLeftEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kleftEncoderPositionPIDMinInput = 0; // radians
    public static final double kLeftEncoderPositionPIDMaxInput = kLeftEncoderPositionFactor; // radians

    public static final double kRightEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kRightEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kRightEncoderPositionPIDMinInput = 0; // radians
    public static final double kRightEncoderPositionPIDMaxInput = kRightEncoderPositionFactor; // radians

    public static final double kTiltEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTiltEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kTiltEncoderPositionPIDMinInput = 0; // radians
    public static final double kTiltEncoderPositionPIDMaxInput = kRightEncoderPositionFactor; // radians

    public static final double kLeftP = 0.04;
    public static final double kLeftI = 0;
    public static final double kLeftD = 0;
    public static final double kLeftFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kLeftMinOutput = -1;
    public static final double kLeftMaxOutput = 1;

    public static final double kRightP = 1;
    public static final double kRightI = 0;
    public static final double kRightD = 0;
    public static final double kRightFF = 0;
    public static final double kRightMinOutput = -1;
    public static final double kRightMaxOutput = 1;

    public static final double kTiltP = 1;
    public static final double kTiltI = 0;
    public static final double kTiltD = 0;
    public static final double kTiltFF = 0;
    public static final double kTiltMinOutput = -1;
    public static final double kTiltMaxOutput = 1;

    public static final IdleMode kLeftIdleMode = IdleMode.kBrake;
    public static final IdleMode kRightIdleMode = IdleMode.kBrake;
    public static final IdleMode kTiltIdleMode = IdleMode.kBrake;

    public static final int kLeftCurrentLimit = 20; // amps
    public static final int kRightCurrentLimit = 20; // amps
    public static final int kTiltCurrentLimit = 20; // amps
  }

  public static final class Ladder {

    // define ladder positions
    public static final double STOW = 1.0;
    public static final double FLOOR = 1.0;
    public static final double STATION = 1.0;
    public static final double L1 = 1.0;
    public static final double L2 = 1.0;
    public static final double L3 = 1.0;
    public static final double L4 = 1.0;
    public static final double BARGE = 1.0;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kLeftMotorPinionTeeth = 14;

    // Invert the right encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kRightEncoderInverted = false;

    public static final boolean kRightMotorInverted = false;
    public static final boolean kLeftMotorInverted = true;

    public static final boolean kLeftEncodeWrapping = false;
    public static final boolean kRightEncodeWrapping = false;

    // Calculations required for left motor conversion factors and feed forward
    public static final double kLeftMotorFreeSpeedRps = MotorConstants.k550FreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.feetToMeters(0.33333);// 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kLeftMotorReduction = (45.0 * 22) / (kLeftMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kLeftMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kLeftMotorReduction;

    public static final double kLeftEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kLeftEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kleftEncoderPositionPIDMinInput = 0; // radians
    public static final double kLeftEncoderPositionPIDMaxInput = kLeftEncoderPositionFactor; // radians

    public static final double kRightEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kRightEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kRightEncoderPositionPIDMinInput = 0; // radians
    public static final double kRightEncoderPositionPIDMaxInput = kRightEncoderPositionFactor; // radians

    public static final double kLeftPosP = 0.04;
    public static final double kLeftPosI = 0;
    public static final double kLeftPosD = 0;
    public static final double kLeftPosFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kLeftPosMinOutput = -1;
    public static final double kLeftPosMaxOutput = 1;

    public static final double kLeftVelP = 0.04;
    public static final double kLeftVelI = 0;
    public static final double kLeftVelD = 0;
    public static final double kLeftVelFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kLeftVelMinOutput = -1;
    public static final double kLeftVelMaxOutput = 1;

    public static final double kRightPosP = 1;
    public static final double kRightPosI = 0;
    public static final double kRightPosD = 0;
    public static final double kRightPosFF = 0;
    public static final double kRightPosMinOutput = -1;
    public static final double kRightPosMaxOutput = 1;

    public static final double kRightVelP = 1;
    public static final double kRightVelI = 0;
    public static final double kRightVelD = 0;
    public static final double kRightVelFF = 0;
    public static final double kRightVelMinOutput = -1;
    public static final double kRightVelMaxOutput = 1;

    public static final IdleMode kLeftIdleMode = IdleMode.kBrake;
    public static final IdleMode kRightIdleMode = IdleMode.kBrake;

    public static final int kLeftCurrentLimit = 20; // amps
    public static final int kRightCurrentLimit = 20; // amps
  }

  public static final class Climber {

    // define climber positions
    public static final double STOW = 1.0;
    public static final double READY = 1.0;
    public static final double CLIMB = 1.0;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kLeftMotorPinionTeeth = 14;

    // Invert the right encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kRightEncoderInverted = false;

    public static final boolean kRightMotorInverted = false;
    public static final boolean kLeftMotorInverted = true;

    public static final boolean kLeftEncodeWrapping = false;
    public static final boolean kRightEncodeWrapping = false;

    // Calculations required for left motor conversion factors and feed forward
    public static final double kLeftMotorFreeSpeedRps = MotorConstants.k550FreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.feetToMeters(0.33333);// 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kLeftMotorReduction = (45.0 * 22) / (kLeftMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kLeftMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kLeftMotorReduction;

    public static final double kLeftEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kLeftEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kleftEncoderPositionPIDMinInput = 0; // radians
    public static final double kLeftEncoderPositionPIDMaxInput = kLeftEncoderPositionFactor; // radians

    public static final double kRightEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kRightEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kRightEncoderPositionPIDMinInput = 0; // radians
    public static final double kRightEncoderPositionPIDMaxInput = kRightEncoderPositionFactor; // radians

    public static final double kLeftPosP = 0.04;
    public static final double kLeftPosI = 0;
    public static final double kLeftPosD = 0;
    public static final double kLeftPosFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kLeftPosMinOutput = -1;
    public static final double kLeftPosMaxOutput = 1;

    public static final double kLeftVelP = 0.04;
    public static final double kLeftVelI = 0;
    public static final double kLeftVelD = 0;
    public static final double kLeftVelFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kLeftVelMinOutput = -1;
    public static final double kLeftVelMaxOutput = 1;

    public static final double kRightPosP = 1;
    public static final double kRightPosI = 0;
    public static final double kRightPosD = 0;
    public static final double kRightPosFF = 0;
    public static final double kRightPosMinOutput = -1;
    public static final double kRightPosMaxOutput = 1;

    public static final double kRightVelP = 1;
    public static final double kRightVelI = 0;
    public static final double kRightVelD = 0;
    public static final double kRightVelFF = 0;
    public static final double kRightVelMinOutput = -1;
    public static final double kRightVelMaxOutput = 1;

    public static final IdleMode kLeftIdleMode = IdleMode.kBrake;
    public static final IdleMode kRightIdleMode = IdleMode.kBrake;

    public static final int kLeftCurrentLimit = 20; // amps
    public static final int kRightCurrentLimit = 20; // amps
  }

}
