// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Ladder;

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
	private final Ladder ladder = new Ladder();
	private final Algae algae = new Algae(ladder);
	private final Coral coral = new Coral(ladder);
	private final Climber climber = new Climber();

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

		// algae.tiltChanged.onTrue(new RunCommand(() ->
		// algae.setTiltPos(algae.getTiltSP())));
		// algae.intakeChanged.onTrue(new RunCommand(() ->
		// algae.setIntakeVel(algae.getIntakeSP())));

		// coral.tiltChanged.onTrue(new RunCommand(() ->
		// coral.setTiltPos(coral.getTiltSP())));
		// coral.intakeChanged.onTrue(new RunCommand(() ->
		// coral.setIntakeVel(coral.getIntakeSP())));

		// climber.climberChanged.onTrue(new RunCommand(() ->
		// climber.setClimberPos(climber.getClimberSP())));

		// ladder.ladderChanged.onTrue(new RunCommand(() ->
		// ladder.setLadderPos(ladder.getLadderSP())));

		algae.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				new ParallelCommandGroup(
						new RunCommand(() -> algae.setTiltPos(coral.getTiltSP()), algae),
						new RunCommand(() -> algae.setIntakeVel(coral.getLeftIntakeSP()))));

		coral.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				new ParallelCommandGroup(
						new RunCommand(() -> coral.setTiltPos(coral.getTiltSP()), coral),
						new RunCommand(() -> coral.setLeftIntakeVel(coral.getLeftIntakeSP())),
						new RunCommand(() -> coral.setRightIntakeVel(coral.getRightIntakeSP()))));

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

		// ParallelCommandGroup algaeBarge = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderSP(Constants.Ladder.BARGE), ladder),
		// new RunCommand(() -> algae.setTiltSP(Constants.Algae.BARGE), algae));
		// ParallelCommandGroup algaeL3 = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderSP(Constants.Ladder.L3), ladder),
		// new RunCommand(() -> algae.setTiltSP(Constants.Algae.L3), algae));
		// ParallelCommandGroup algaeL2 = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderSP(Constants.Ladder.L2), ladder),
		// new RunCommand(() -> algae.setTiltSP(Constants.Algae.L2), algae));
		// ParallelCommandGroup algaeProcessor = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderSP(Constants.Ladder.PROCESSOR), ladder),
		// new RunCommand(() -> algae.setTiltSP(Constants.Algae.PROCESSOR), algae));
		// ParallelCommandGroup algaeFloor = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderSP(Constants.Ladder.FLOOR), ladder),
		// new RunCommand(() -> algae.setTiltSP(Constants.Algae.FLOOR), algae));
		// ParallelCommandGroup algaeStow = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderSP(Constants.Ladder.STOW), ladder),
		// new RunCommand(() -> algae.setTiltSP(Constants.Algae.STOW), algae));
		// ParallelCommandGroup algaePos = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderPos(), ladder),
		// new RunCommand(() -> algae.setTiltPos(), algae));

		// ParallelCommandGroup coralL4 = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderSP(Constants.Ladder.L4), ladder),
		// new RunCommand(() -> coral.setTiltSP(Constants.Coral.L4), coral));
		// ParallelCommandGroup coralL3 = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderSP(Constants.Ladder.L3), ladder),
		// new RunCommand(() -> coral.setTiltSP(Constants.Coral.L3), coral));
		// ParallelCommandGroup coralL2 = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderSP(Constants.Ladder.L2), ladder),
		// new RunCommand(() -> coral.setTiltSP(Constants.Coral.L2), coral));
		// ParallelCommandGroup coralL1 = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderSP(Constants.Ladder.L1), ladder),
		// new RunCommand(() -> coral.setTiltSP(Constants.Coral.L1), coral));
		// ParallelCommandGroup coralStation = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderSP(Constants.Ladder.STATION), ladder),
		// new RunCommand(() -> coral.setTiltSP(Constants.Coral.STATION), coral));
		// ParallelCommandGroup coralStow = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderSP(Constants.Ladder.STOW), ladder),
		// new RunCommand(() -> coral.setTiltSP(Constants.Coral.STOW), coral));
		// ParallelCommandGroup coralPos = new ParallelCommandGroup(
		// new RunCommand(() -> ladder.setLadderPos(), ladder),
		// new RunCommand(() -> coral.setTiltPos(), coral));

		// RunCommand algaeIntake = new RunCommand(() ->
		// algae.setIntakeSP(Constants.Algae.INTAKE), algae);
		// RunCommand algaeScore = new RunCommand(() ->
		// algae.setIntakeSP(Constants.Algae.EJECT), algae);

		// RunCommand coralIntake = new RunCommand(() ->
		// coral.setIntakeSP(Constants.Coral.INTAKE), coral);
		// RunCommand coralScore = new RunCommand(() ->
		// coral.setIntakeSP(Constants.Coral.EJECT), coral);

		// RunCommand climberClimb = new RunCommand(() ->
		// climber.setClimberPos(Constants.Climber.CLIMB), climber);
		// RunCommand climberReady = new RunCommand(() ->
		// climber.setClimberPos(Constants.Climber.READY), climber);
		// RunCommand climberStow = new RunCommand(() ->
		// climber.setClimberPos(Constants.Climber.STOW), climber);

		// ladderTab.addPersistent("Ladder Limit", false)
		// .withWidget("Boolean Box").withPosition(4, 0).withSize(1, 1).getEntry();

		ShuffleboardLayout algaeCommands = Shuffleboard.getTab("Competition")
				.getLayout("Algae Commands", BuiltInLayouts.kList)
				.withSize(2, 5)
				.withPosition(0, 1);
		// .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for
		// commands
		algaeCommands.add("Barge", algae.algaeBarge());
		algaeCommands.add("L3", algae.algaeL3());
		algaeCommands.add("L2", algae.algaeL2());
		algaeCommands.add("Floor", algae.algaeFloor());
		algaeCommands.add("Stow", algae.algaeStow());
		algaeCommands.add("Intake", algae.algaeIntake());
		algaeCommands.add("Eject", algae.algaeEject());

		ShuffleboardLayout coralCommands = Shuffleboard.getTab("Competition")
				.getLayout("Coral Commands", BuiltInLayouts.kList)
				.withSize(2, 5)
				.withPosition(2, 1);
		// .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for
		// commands
		coralCommands.add("L4", coral.coralL4());
		coralCommands.add("L3", coral.coralL3());
		coralCommands.add("L2", coral.coralL2());
		coralCommands.add("L1", coral.coralL1());
		coralCommands.add("Station", coral.coralStation());
		coralCommands.add("Stow", coral.coralStow());
		coralCommands.add("Left Intake", coral.coralLeftIntake());
		coralCommands.add("Left Eject", coral.coralLeftEject());
		coralCommands.add("Right Intake", coral.coralRightIntake());
		coralCommands.add("Right Eject", coral.coralRightEject());

		ShuffleboardLayout climberCommands = Shuffleboard.getTab("Competition")
				.getLayout("Climber Commands", BuiltInLayouts.kList)
				.withSize(2, 2)
				.withPosition(4, 1);
		// .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for
		// commands
		climberCommands.add("Climb", climber.climberClimb());
		climberCommands.add("Ready", climber.climberReady());
		climberCommands.add("Stow", climber.climberStow());

		// algaeTab.add("Algae Barge", algaeBarge);
		// algaeTab.add("Algae L3", algaeL3);
		// algaeTab.add("Algae L2", algaeL2);
		// algaeTab.add("Algae Processor", algaeProcessor);
		// algaeTab.add("Algae Floor", algaeFloor);
		// algaeTab.add("Algae Stow", algaeStow);

		// coralTab.add("Coral L4", coralL4);
		// coralTab.add("Coral L3", coralL3);
		// coralTab.add("Coral L2", coralL2);
		// coralTab.add("Coral L1", coralL1);
		// coralTab.add("Coral Station", coralStation);
		// coralTab.add("Coral Stow", coralStow);

		// algaeTab.add("Algae Score", algaeScore);
		// algaeTab.add("Algae Intake", algaeIntake);

		// coralTab.add("Coral Score", coralScore);
		// coralTab.add("Coral Intake", coralIntake);

		// climberTab.add("Climber Climb", climberClimb);
		// climberTab.add("Climber Ready", climberReady);
		// climberTab.add("Climber Stow", climberStow);
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
		// m_driverController.b().whileTrue(chassis.exampleMethodCommand());
		// new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new
		// ExampleCommand(m_exampleSubsystem));

		new Trigger(algae.tiltChanged).onTrue(algae.setTiltCmd());
		new Trigger(algae.leftIntakeChanged).onTrue(algae.setIntakeCmd());

		new Trigger(coral.tiltChanged).onTrue(coral.setTiltCmd());
		new Trigger(coral.leftIntakeChanged).onTrue(coral.setLeftIntakeCmd());
		new Trigger(coral.rightIntakeChanged).onTrue(coral.setRightIntakeCmd());

		new Trigger(climber.climberChanged).onTrue(climber.setClimberCmd());

		new Trigger(ladder.ladderChanged).onTrue(ladder.setLadderCmd());
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
