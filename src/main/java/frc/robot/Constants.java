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

	public static class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDriveDeadband = 0.05;
	}

	public static final class CANId {

		public static final int kPDHCanID = 1;

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

		public static final int kClimberLeftCanId = 40;
		public static final int kClimberRightCanId = 41;

		public static final int kLadderLeftCanId = 50;
		public static final int kLadderRightCanId = 51;
	}

	public static final class MotorConstants {
		public static final double kVortexFreeSpeedRpm = 6784;
		public static final double kNeoFreeSpeedRpm = 5676;
		public static final double k550FreeSpeedRpm = 11000;
	}

	public static final class ChassisConstants {
		// not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double kMaxSpeedMetersPerSecond = 4.8;
		public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

		public static final double kDirectionSlewRate = 1.2; // radians per second
		public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
		public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

		// Chassis configuration
		public static final double kTrackWidth = Units.inchesToMeters(24.75);
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = Units.inchesToMeters(24.75);
		// Distance between front and back wheels on robot
		public static final double kWheelRadius = Math.sqrt(2.0 * Math.pow(kWheelBase, 2)) / 2.0;

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		// Angular offsets of the modules relative to the chassis in radians
		public static final double kFrontLeftChassisAngularOffset = 0.0; // -Math.PI / 2;
		public static final double kFrontRightChassisAngularOffset = 0.0; // 0;
		public static final double kBackLeftChassisAngularOffset = 0.0; // Math.PI;
		public static final double kBackRightChassisAngularOffset = 0.0; // Math.PI / 2;

		public static final boolean kGyroReversed = false;
	}

	public static final class DriveConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double kMaxSpeedMetersPerSecond = 4.8;
		public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

		// Chassis configuration
		public static final double kTrackWidth = Units.inchesToMeters(26.5);
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = Units.inchesToMeters(26.5);
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		// Angular offsets of the modules relative to the chassis in radians
		public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
		public static final double kFrontRightChassisAngularOffset = 0;
		public static final double kBackLeftChassisAngularOffset = Math.PI;
		public static final double kBackRightChassisAngularOffset = Math.PI / 2;

		public static final boolean kGyroReversed = false;

		public static final int kDrivingMotorCurrentLimit = 50; // amps
		public static final int kTurningMotorCurrentLimit = 20; // amps
	}

	public static final class ModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T,
		// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
		// more teeth will result in a robot that drives faster).
		public static final int kDrivingMotorPinionTeeth = 14;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kNeoFreeSpeedRpm / 60;
		public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); // 0.0762;
		public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
		// teeth on the bevel pinion
		public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
		public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
				/ kDrivingMotorReduction;
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

		public static final double kPXController = 1;
		public static final double kPYController = 1;
		public static final double kPThetaController = 1;

		// Constraint for the motion profiled robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
	}

	public static final class Coral {

		// define coral positions
		// public static final double STOW = 90.0; 	// degrees
		// public static final double STATION = 55.0; 	// degrees
		// public static final double L1 = 0.0;		// degrees
		// public static final double L2 = -35.0;		// degrees
		// public static final double L3 = -35.0;		// degrees
		// public static final double L4 = -45.0;		// degrees

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

		public static final double kIntakeEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
				/ kLeftMotorReduction; // meters
		public static final double kIntakeEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
				/ kLeftMotorReduction) / 60.0; // meters per second

		public static final double kTiltEncoderPositionFactor = (2 * Math.PI); // radians
		public static final double kTiltEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
		public static final double kTiltEncoderPositionPIDMinInput = 0; // radians
		public static final double kTiltEncoderPositionPIDMaxInput = kTiltEncoderPositionFactor; // radians

		public static final double kIntakeP = 0.04;
		public static final double kIntakeI = 0;
		public static final double kIntakeD = 0;
		public static final double kIntakeFF = 1 / kDriveWheelFreeSpeedRps;
		public static final double kIntakeMinOutput = -1;
		public static final double kIntakeMaxOutput = 1;

		public static final double kTiltP = 1;
		public static final double kTiltI = 0;
		public static final double kTiltD = 0;
		public static final double kTiltFF = 0;
		public static final double kTiltMinOutput = -1;
		public static final double kTiltMaxOutput = 1;

		public static final IdleMode kIntakeIdleMode = IdleMode.kBrake;
		public static final IdleMode kTiltIdleMode = IdleMode.kBrake;

		public static final int kLeftCurrentLimit = 20; // amps
		public static final int kTiltCurrentLimit = 20; // amps
	}

	public static final class Algae {

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

		// define ladder zeroing speeds
		public static final double DOWN = -0.10;
		public static final double STOP = 0.0;

		// The MAXSwerve module can be configured with one of three pinion gears: 12T,
		// 13T, or 14T.
		// This changes the drive speed of the module (a pinion gear with more teeth
		// will result in a
		// robot that drives faster).
		public static final int kLeftMotorPinionTeeth = 14;

		// Invert the right encoder, since the output shaft rotates in the opposite
		// direction of
		// the steering motor in the MAXSwerve Module.
		public static final boolean kRightEncoderInverted = true;

		public static final boolean kRightMotorInverted = false;
		public static final boolean kLeftMotorInverted = false;

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
		public static final double STOW = 90.0;		// degrees, straight up
		public static final double READY = 0.0; 	// degrees, horizontal but not lifting
		public static final double CLIMB = -25.0; 	// degrees, fully lifting

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
