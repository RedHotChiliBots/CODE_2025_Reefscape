// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Ladder;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
	private final Chassis chassis = new Chassis();
	private final Algae algae = new Algae();
	private final Coral coral = new Coral();
	private final Climber climber = new Climber();
	private final Ladder ladder = new Ladder();
	private final PhotonCamera camera1;
	private final PhotonCamera camera2;
	private final PhotonCamera camera3;
	private final PhotonCamera camera4;

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController m_driverController = new CommandXboxController(
			OIConstants.kDriverControllerPort);
	private final CommandXboxController m_operatorController = new CommandXboxController(
			OIConstants.kOperatorControllerPort);

	private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
	private final ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");
	private final ShuffleboardTab coralTab = Shuffleboard.getTab("Coral");
	private final ShuffleboardTab algaeTab = Shuffleboard.getTab("Algae");
	private final ShuffleboardTab ladderTab = Shuffleboard.getTab("Ladder");
	private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		compTab.add("Chassis", chassis);
		compTab.add("Coral", coral);
		compTab.add("Algae", algae);
		compTab.add("Ladder", ladder);
		compTab.add("Climber", climber);
		camera1 = new PhotonCamera("OrangePiCamera1");
		camera2 = new PhotonCamera("OrangePiCamera2");
		camera3 = new PhotonCamera("OrangePiCamera3");
		camera4 = new PhotonCamera("OrangePiCamera4");

		// Configure the trigger bindings
		configureBindings();

		// Configure default commands
		chassis.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				new RunCommand(
						() -> chassis.drive(
								-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
								-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
								-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
								true),
						chassis));

		algae.tiltChanged.onTrue(new RunCommand(() -> algae.setTiltPos(algae.getTiltSP())));
		algae.intakeChanged.onTrue(new RunCommand(() -> algae.setIntakeVel(algae.getIntakeSP())));

		coral.tiltChanged.onTrue(new RunCommand(() -> coral.setTiltPos(coral.getTiltSP())));
		coral.intakeChanged.onTrue(new RunCommand(() -> coral.setIntakeVel(coral.getIntakeSP())));

		climber.climberChanged.onTrue(new RunCommand(() -> climber.setClimberPos(climber.getClimberSP())));

		ladder.ladderChanged.onTrue(new RunCommand(() -> ladder.setLadderPos(ladder.getLadderSP())));

		algae.setDefaultCommand(
		// The left stick controls translation of the robot.
		// Turning is controlled by the X axis of the right stick.
		new ParallelCommandGroup(
		new RunCommand( () -> algae.setTiltPos(coral.getTiltSP()), algae),
		new RunCommand( () -> algae.setIntakeVel(coral.getIntakeSP()))));

		coral.setDefaultCommand(
		// The left stick controls translation of the robot.
		// Turning is controlled by the X axis of the right stick.
		new ParallelCommandGroup(
		new RunCommand( () -> coral.setTiltPos(coral.getTiltSP()), coral),
		new RunCommand( () -> coral.setIntakeVel(coral.getIntakeSP()))));

		climber.setDefaultCommand(
		// The left stick controls translation of the robot.
		// Turning is controlled by the X axis of the right stick.
		new RunCommand(
		() -> climber.setClimberPos(climber.getClimberSP()), climber));

		ladder.setDefaultCommand(
		// The left stick controls translation of the robot.
		// Turning is controlled by the X axis of the right stick.
		new RunCommand(
		() -> ladder.setLadderPos(ladder.getLadderSP()), ladder));
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
		// Schedule `exampleMethodCommand` when the Xbox controller's B button is
		// pressed,
		// cancelling on release.
		// m_driverController.b().whileTrue(chassis.exampleMethodCommand());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(chassis);
	}
}
