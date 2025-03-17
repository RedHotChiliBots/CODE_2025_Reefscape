// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Algae.AlgaeSP;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Coral.CoralSP;
import frc.robot.subsystems.Ladder;
import frc.robot.subsystems.Ladder.LadderSP;
import frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...

	private final PhotonCamera camera1 = new PhotonCamera("PhotonVision 1");
	private final PhotonCamera camera2 = new PhotonCamera("PhotonVision 2");
	private final PhotonCamera camera3 = new PhotonCamera("PhotonVision 3");
	private final PhotonCamera camera4 = new PhotonCamera("PhotonVision 4");
	private final List<PhotonCamera> cameras = List.of(camera1, camera2, camera3, camera4);

	private final Vision vision = new Vision(cameras.get(0), cameras.get(1),
			cameras.get(2), cameras.get(3));

	private final Chassis chassis = new Chassis();
	private final Ladder ladder = new Ladder();
	private final Algae algae = new Algae(ladder);
	private final Coral coral = new Coral(ladder, algae);
	private final Climber climber = new Climber();

	// Define HIDs
	private final CommandXboxController m_driverController = new CommandXboxController(
			OIConstants.kDriverControllerPort);
	private final CommandXboxController m_operatorController = new CommandXboxController(
			OIConstants.kOperatorControllerPort);
	private final GenericHID m_operatorHID = new GenericHID(
			OIConstants.kOperatorControllerPort);

	// =====TESTING=====//
	public final Command goBarge = new ParallelCommandGroup(
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.BARGE)),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.STOW)),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.BARGE)));
	public final Command goL4 = new ParallelCommandGroup(
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.L4)),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.L4)),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.STOW)));
	public final Command goL3 = new ParallelCommandGroup(
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.L3)),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.L3)),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.L3)));
	public final Command goL2 = new ParallelCommandGroup(
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.L2)),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.L2)),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.L2)));
	public final Command goL1 = new ParallelCommandGroup(
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.L1)),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.L1)),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.STOW)));
	public final Command goStation = new ParallelCommandGroup(
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.STATION)),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.STATION)),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.STOW)));
	public final Command goProcessor = new ParallelCommandGroup(
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.PROCESSOR)),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.STOW)),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.PROCESSOR)));
	public final Command goFloor = new ParallelCommandGroup(
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.FLOOR)),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.STOW)),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.FLOOR)));
	public final Command goStow = new ParallelCommandGroup(
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.STOW)),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.STOW)),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.STOW)));

	public final Command doAction = new ParallelCommandGroup(
			coral.doAction(),
			algae.doAction());

	private final Autos auton = new Autos(this, chassis, ladder, algae, coral, climber);

	private final ShuffleboardTab cmdTab = Shuffleboard.getTab("Commands");

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		vision.setChassis(chassis);

		// algaeTab.add("Processor3", algae.processor);
		// algaeTab.add("Floor3", algae.floor);
		// algaeTab.add("Intake3", algae.intake);
		// algaeTab.add("Eject3", algae.eject);

		// cmdTab.add("climberStow", climberStow);
		// cmdTab.add("climberReady", climberReady);
		// cmdTab.add("climberZero", climberZero);
		// cmdTab.add("climberClimb", climberClimb);

		// cmdTab.add("algaeStow", algaeStow);
		// cmdTab.add("algaeZero", algaeZero);
		// cmdTab.add("algaeBarge", algaeBarge);
		// cmdTab.add("algaeProcessor", algaeProcessor);
		// cmdTab.add("algaeFloor", algaeFloor);
		// cmdTab.add("algaeL3", algaeL3);
		// cmdTab.add("algaeL2", algaeL2);

		// cmdTab.add("coralStow", coralStow);
		// cmdTab.add("coralZero", coralZero);
		// cmdTab.add("coralStation", coralStation);
		// cmdTab.add("coralL4", coralL4);
		// cmdTab.add("coralL3", coralL3);
		// cmdTab.add("coralL2", coralL2);
		// cmdTab.add("coralL1", coralL1);

		// compTab.add("Chassis", chassis);
		// compTab.add("Coral", coral);
		// compTab.add("Algae", algae);
		// compTab.add("Ladder", ladder);
		// compTab.add("Climber", climber);

		for (PhotonCamera camera : cameras) {
			camera.setPipelineIndex(0); // default pipeline set up in PhotonVision web interface
		}

		// Configure the trigger bindings
		configureBindings();

		// Configure default commands
		chassis.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				new RunCommand(
						() -> chassis.drive(
								-MathUtil.applyDeadband(m_driverController.getLeftY()
										* chassis.spdMultiplier, OIConstants.kDriveDeadband),
								-MathUtil.applyDeadband(m_driverController.getLeftX()
										* chassis.spdMultiplier, OIConstants.kDriveDeadband),
								-MathUtil.applyDeadband(m_driverController.getRightX()
										* chassis.spdMultiplier, OIConstants.kDriveDeadband),
								true),
						chassis));

		// climber.setDefaultCommand(
		// // The right Y stick controls movement
		// new RunCommand(
		// () -> climber.moveClimber(
		// -MathUtil.applyDeadband(m_operatorController.getRightY(), 0.10)),
		// // OIConstants.kDriveDeadband)),
		// climber));

		// coral.setDefaultCommand(
		// // The left stick controls translation of the robot.
		// // Turning is controlled by the X axis of the right stick.
		// new RunCommand(
		// () -> coral.moveTilt(
		// -MathUtil.applyDeadband(m_operatorController.getLeftX(),
		// OIConstants.kDriveDeadband)),
		// coral));

		// algae.setDefaultCommand(
		// // The left stick controls translation of the robot.
		// // Turning is controlled by the X axis of the right stick.
		// new RunCommand(
		// () -> algae.moveTilt(
		// -MathUtil.applyDeadband(m_operatorController.getRightX(),
		// OIConstants.kDriveDeadband)),
		// algae));

		ShuffleboardLayout toggleCommands = cmdTab
				.getLayout("Toggle", BuiltInLayouts.kList)
				.withSize(3, 4)
				.withPosition(13, 7)
				.withProperties(Map.of("Label position", "Hidden"));
		toggleCommands.add("Algae Extract", algae.toggleExtract);
		toggleCommands.add("Coral Left-Right", coral.toggleSide);

		ShuffleboardLayout goCommands = cmdTab
				.getLayout("Go To", BuiltInLayouts.kList)
				.withSize(3, 12)
				.withPosition(17, 1)
				.withProperties(Map.of("Label position",
						"Hidden"));
		goCommands.add("Barge", goBarge)
				.withProperties(Map.of("show type", false));
		goCommands.add("L4", goL4)
				.withProperties(Map.of("show type", false));
		goCommands.add("L3", goL3)
				.withProperties(Map.of("show type", false));
		goCommands.add("L2", goL2)
				.withProperties(Map.of("show type", false));
		goCommands.add("L1", goL1)
				.withProperties(Map.of("show type", false));
		goCommands.add("Station", goStation)
				.withProperties(Map.of("show type", false));
		goCommands.add("Processor", goProcessor)
				.withProperties(Map.of("show type", false));
		goCommands.add("Floor", goFloor)
				.withProperties(Map.of("show type", false));
		goCommands.add("Stow", goStow)
				.withProperties(Map.of("show type", false));

		ShuffleboardLayout doCommands = cmdTab
				.getLayout("Do", BuiltInLayouts.kList)
				.withSize(3, 4)
				.withPosition(21, 1)
				.withProperties(Map.of("Label position",
						"Hidden"));
		doCommands.add("Do Action", doAction)
				.withProperties(Map.of("show type", false));
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link
	 * CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {

		m_driverController.leftBumper()
				.onFalse(new InstantCommand(() -> chassis.setSpdHigh()))
				.onTrue(new InstantCommand(() -> chassis.setSpdLow()));

		m_driverController.rightBumper()
				.onFalse(new InstantCommand(() -> chassis.setPoseErr()))
				.onTrue(new InstantCommand(() -> chassis.setPoseZero()));

		// m_driverController.x().onTrue(chassis.setX);

		// m_operatorController.y().onTrue(climber.stow);
		// m_operatorController.b().onTrue(climber.ready);
		// m_operatorController.a().onTrue(climber.climb);
		// m_operatorController.x().onTrue(climber.zero);

		// m_operatorController.start().onTrue(algae.intake);
		// m_operatorController.back().onTrue(algae.eject);

		// new POVButton(m_operatorHID, 0).onTrue(algae.stow);
		// new POVButton(m_operatorHID, 90).onTrue(algae.processor);
		// new POVButton(m_operatorHID, 270).onTrue(algae.processor);
		// new POVButton(m_operatorHID, 180).onTrue(algae.floor);

		m_operatorController.y().onTrue(this.goL4);
		m_operatorController.b().onTrue(this.goL3);
		m_operatorController.a().onTrue(this.goL2);
		m_operatorController.x().onTrue(this.goL1);

		new POVButton(m_operatorHID, 0).onTrue(this.goBarge);
		new POVButton(m_operatorHID, 90).onTrue(this.goStation);
		new POVButton(m_operatorHID, 270).onTrue(this.goProcessor);
		new POVButton(m_operatorHID, 180).onTrue(this.goFloor);

		m_operatorController.start().onTrue(climber.ready);
		m_operatorController.back().onTrue(this.goStow);

		m_operatorController.leftBumper().onTrue(climber.climb);
		m_operatorController.rightBumper().onTrue(this.doAction);

	}

	public RobotContainer getRobotContainer() {
		return this;
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// return new ChassisTimedDrive(chassis, 0.25, 1.0);
		return auton.getAutoChooser().getSelected();
	}
}
