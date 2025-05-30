// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;


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

	public static final class PWMId {
		public static final int kGrenadePin = 0;
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

	public static final class ColorConstants {
		public static final String Stopped = Color.kRed.toHexString();
		public static final String Moving = Color.kYellow.toHexString();
		public static final String OnTarget = Color.kGreen.toHexString();
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
		public static final double kTrackWidth = Units.inchesToMeters(24.0);
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = Units.inchesToMeters(24.5);
		// Distance between front and back wheels on robot
		public static final double kWheelRadius = Math.sqrt(2.0 * Math.pow(kWheelBase, 2)) / 2.0;

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		// Angular offsets of the modules relative to the chassis in radians
		public static final double kFrontLeftChassisAngularOffset = Math.PI / 2;
		public static final double kFrontRightChassisAngularOffset = 0.0;
		public static final double kBackLeftChassisAngularOffset = Math.PI;
		public static final double kBackRightChassisAngularOffset = -Math.PI / 2;

		// public static final double kFrontLeftChassisAngularOffset = 0.0;
		// public static final double kFrontRightChassisAngularOffset = 0.0;
		// public static final double kBackLeftChassisAngularOffset = 0.0;
		// public static final double kBackRightChassisAngularOffset = 0.0;

		public static final boolean kGyroReversed = false;
	}

	public static final class DriveConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double kMaxSpeedMetersPerSecond = 4.8;
		public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

		// Chassis configuration
		public static final double kTrackWidth = Units.inchesToMeters(24.0);
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = Units.inchesToMeters(24.5);
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
		public static final int kDrivingMotorPinionTeeth = 16;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kVortexFreeSpeedRpm / 60;
		public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); // 0.0762;
		public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
		// teeth on the bevel pinion
		public static final double kDrivingMotorReduction = 5.9;	//(45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
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

	// Gear ratios for Max and Ultra gearboxes
	public static final class GearBox {
		public static final double Max3 = 3.0;
		public static final double Max4 = 4.0;
		public static final double Max5 = 5.0;
		public static final double Max9 = 9.0;

		public static final double Ultra3 = 2.89;
		public static final double Ultra4 = 3.61;
		public static final double Ultra5 = 5.23;
	}

	public static final class Coral {

		public static final double kTiltTollerance = 0.5; // degrees
		public static final double kIntakeTollerance = 10.0; // RPMs

		// define coral velocities
		public static final double STOP = 0.0;
		public static final double INTAKE = 5000.0;
		public static final double EJECT = -5000.0;

		public static final double kTiltZeroOffset = 0.6310901;
		public static final boolean kTiltZeroCentered = true;
		public static final boolean kTiltMotorInverted = true;
		public static final boolean kTiltEncoderInverted = false;

		public static final boolean kRightMotorInverted = true;
		public static final boolean kLeftMotorInverted = false;

		public static final boolean kIntakeEncodeWrapping = false;
		public static final boolean kTiltEncodeWrapping = false;

		public static final double kIntakePositionFactor = 1.0 / (GearBox.Ultra4 * GearBox.Ultra4 * GearBox.Ultra4); // radians
		public static final double kIntakeVelocityFactor = kIntakePositionFactor / 60.0; // radians per second

		public static final double kTiltPositionFactor = 360; // 1.0 / (GearBox.Ultra4 * GearBox.Ultra4 * GearBox.Ultra4)
																				// * 360.0; // degrees
		public static final double kTiltVelocityFactor = kTiltPositionFactor / 60.0; // degrees per second

		public static final double kIntakeVelP = 0.0001;
		public static final double kIntakeVelI = 0;
		public static final double kIntakeVelD = 0;
		public static final double kIntakeVelFF = 1.0 / MotorConstants.k550FreeSpeedRpm;
		public static final double kIntakeVelMinOutput = -1;
		public static final double kIntakeVelMaxOutput = 1;

		public static final double kIntakeVelMaxVel = 10000.0;
		public static final double kIntakeVelMaxAccel = 20000.0;
		public static final double kIntakeVelAllowedErr = 1.0;

		// public static final double kTiltPosP = 0.005; // 0.0022;
		// public static final double kTiltPosI = 0.0; // 0.00000000015;
		// public static final double kTiltPosD = 0.02; // 0.1;
		// public static final double kTiltPosMinOutput = -1.0;
		// public static final double kTiltPosMaxOutput = 1.0;

		// public static final double kTiltPosMaxVel = 10000.0;
		// public static final double kTiltPosMaxAccel = 30000.0;
		// public static final double kTiltPosAllowedErr = 0.1;

		public static final double kTiltPosP = 0.01;	// MaxMotion 0.0015;
		public static final double kTiltPosI = 0.000001;	// MaxMotion 0.0;
		public static final double kTiltPosD = 0.0;	// MaxMotion 0.0;
		public static final double kTiltPosMinOutput = -0.88; // MaxMotion 1.0;
		public static final double kTiltPosMaxOutput = 0.88; // MaxMotion 1.0;

		public static final double kTiltPosMaxVel = 8000.0;	//30000.0;
		public static final double kTiltPosMaxAccel = 24000.0;	//40000.0;
		public static final double kTiltPosAllowedErr = 0.1;

		public static final IdleMode kIntakeIdleMode = IdleMode.kBrake;
		public static final IdleMode kTiltIdleMode = IdleMode.kBrake;

		public static final double kStopCurrent = 4.0; // amps
		public static final int kLeftPDHChannel = 11; // amps
		public static final int kRightPDHChannel = 15; // amps

		public static final int kLeftCurrentLimit = 20; // amps
		public static final int kRightCurrentLimit = 20; // amps
		public static final int kTiltCurrentLimit = 20; // amps
	}

	public static final class Algae {

		public static final double kTiltTollerance = 0.5; // degrees
		public static final double kIntakeTollerance = 10.0; // RPMs

		// define algae velocities
		public static final double STOP = 0.0;
		public static final double INTAKE = 5000.0;
		public static final double EJECT = -5000.0;

		public static final double kTiltZeroOffset = 0.4808731;
		public static final boolean kTiltZeroCentered = true;
		public static final boolean kTiltMotorInverted = true;
		public static final boolean kTiltEncoderInverted = true;

		public static final boolean kRightMotorInverted = true;
		public static final boolean kLeftMotorInverted = false;

		public static final boolean kIntakeEncodeWrapping = false;
		public static final boolean kTiltEncodeWrapping = false;

		public static final double kIntakePositionFactor = 1.0 / (GearBox.Ultra4 * GearBox.Ultra4 * GearBox.Ultra4); // radians
		public static final double kIntakeVelocityFactor = kIntakePositionFactor / 60.0; // radians per second

		public static final double kTiltPositionFactor = 360; // 1.0 / (GearBox.Ultra4 * GearBox.Ultra4 * GearBox.Ultra4)
																				// * 360.0; // degrees
		public static final double kTiltVelocityFactor = kTiltPositionFactor / 60.0; // degrees per second

		public static final double kIntakeVelP = 0.0001;
		public static final double kIntakeVelI = 0.0;
		public static final double kIntakeVelD = 0.0;
		public static final double kIntakeVelFF = 1.0 / MotorConstants.k550FreeSpeedRpm;
		public static final double kIntakeVelMinOutput = -1;
		public static final double kIntakeVelMaxOutput = 1;

		public static final double kIntakeVelMaxVel = 10000.0;
		public static final double kIntakeVelMaxAccel = 20000.0;
		public static final double kIntakeVelAllowedErr = 1.0;


		public static final double kTiltPosP = 0.01;	//01; // MaxMotion 0.005;
		public static final double kTiltPosI = 0.0;	//00001; // MaxMotion 0.0;
		public static final double kTiltPosD = 0.0; // MaxMotion 0.02;
		public static final double kTiltPosMinOutput = -0.6; // MaxMotion 1.0;
		public static final double kTiltPosMaxOutput = 0.6; // MaxMotion 1.0;

		public static final double kTiltPosMaxVel = 10000.0;
		public static final double kTiltPosMaxAccel = 30000.0;
		public static final double kTiltPosAllowedErr = 0.1;

		public static final IdleMode kLeftIdleMode = IdleMode.kBrake;
		public static final IdleMode kRightIdleMode = IdleMode.kBrake;
		public static final IdleMode kTiltIdleMode = IdleMode.kBrake;

		public static final int kLeftCurrentLimit = 20; // amps
		public static final int kRightCurrentLimit = 20; // amps
		public static final int kTiltCurrentLimit = 20; // amps
	}

	public static final class Ladder {

		public static final double kMaxLadderHeight = 93.5;	// inches

		// define ladder zeroing speeds
		public static final double DOWN = 0.10;
		public static final double STOP = 0.0;

		public static final double kTollerance = 0.25; // Inches

		public static final boolean kRightMotorInverted = false;
		public static final boolean kLeftMotorInverted = true;

		public static final boolean kLeftEncoderInverted = true;
		public static final boolean kRightEncodeIntered = false;

		public static final boolean kLeftEncodeWrapping = false;
		public static final boolean kRightEncodeWrapping = false;

		public static final double kLiftGearBox = GearBox.Max4 * GearBox.Max5; // to 1 ratio
		public static final double kSprocketDia = 1.375; // inches
		public static final double kStage3perStage1 = 3.0; // ratio

		public static final double kLiftPostionFactor = ((1.0 / kLiftGearBox) * (Math.PI * kSprocketDia))
				* kStage3perStage1;
		public static final double kLiftVelocityFactor = kLiftPostionFactor / 60.0;

		public static final double kPosP = 0.1;	//0.055;	// maxmotion 0.1;
		public static final double kPosI = 0.0;	//0.000001;	// maxmotion 0.0;
		public static final double kPosD = 0.0022;	//0.0;	// maxmotion 0.0022;
		public static final double kPosMinOutput = -1.0;	//-0.7;	// maxmotion -1.0;
		public static final double kPosMaxOutput = 1.0;	//0.7; // maxmotion 1.0;

		public static final double kMaxVel = 5500;
			//3000.0; //100000
		public static final double kMaxAccel = 10000; //5000 // 12000
		public static final double kAllowedErr = 0.2;

		public static final IdleMode kLeftIdleMode = IdleMode.kBrake;
		public static final IdleMode kRightIdleMode = IdleMode.kBrake;

		public static final int kLeftCurrentLimit = 50; // amps
		public static final int kRightCurrentLimit = 50; // amps
	}

	public static final class Climber {

		public static final double kTollerance = 0.5; // degrees

		public static final double kLeftZeroOffset = 0.6643792;
		public static final boolean kLeftZeroCentered = true;
		public static final boolean kLeftMotorInverted = true;
		public static final boolean kLeftEncoderInverted = true;

		public static final double kRightZeroOffset = 0.4019657;
		public static final boolean kRightZeroCentered = true;
		public static final boolean kRightMotorInverted = false;
		public static final boolean kRightEncoderInverted = false;

		public static final boolean kLeftEncodeWrapping = false;
		public static final boolean kRightEncodeWrapping = false;

		public static final double kTiltPositionFactor = 360; // 1.0 / (GearBox.Max9 * GearBox.Max5 * GearBox.Max5) *
																				// 360.0; // degrees
		public static final double kTiltVelocityFactor = kTiltPositionFactor / 60.0; // degrees per second

		// public static final double kTiltPosP = 0.002;
		// public static final double kTiltPosI = 0.000001;
		// public static final double kTiltPosD = 0.0;

		public static final double kPosP = 0.01; // maxmotion 0.025;
		public static final double kPosI = 0.0; // maxmotion 0.0
		public static final double kPosD = 0.0; // maxmotion 0.0
		public static final double kPosMinOutput = -0.5; // maxmotion -1.0
		public static final double kPosMaxOutput = 0.5; // maxmotion 1.0

		public static final double kPosMaxVel = 100000.0; //5000.0
		public static final double kPosMaxAccel = 40000.0; //5000.0
		public static final double kPosAllowedErr = 0.1;

		public static final IdleMode kLeftIdleMode = IdleMode.kBrake;
		public static final IdleMode kRightIdleMode = IdleMode.kBrake;

		public static final int kLeftCurrentLimit = 50; // amps
		public static final int kRightCurrentLimit = 50; // amps
	}
	
	public static final class Vision {
		public static final double kXP = 0.6;
		public static final double kXI = 0.0;
		public static final double kXD = 0.0;
		public static final double kXTollerance = 0.1;

		public static final double kYP = 0.6;
		public static final double kYI = 0.0;
		public static final double kYD = 0.0;
		public static final double kYTollerance = 0.1;

		public static final double kRP = 0.03;
		public static final double kRI = 0.0;
		public static final double kRD = 0.0;
		public static final double kRTollerance = 0.1;
		public static final double kRMin = 0.0;
		public static final double kRMax = 360.0;
	}
}
