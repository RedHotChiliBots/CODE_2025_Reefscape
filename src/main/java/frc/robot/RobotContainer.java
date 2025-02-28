// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Ladder;
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
	// private final PhotonCamera camera1 = null;
	// private final PhotonCamera camera2 = null;
	// private final PhotonCamera camera3 = null;
	// private final PhotonCamera camera4 = null;
	// private final Vision vision = new Vision(camera1, camera2, camera3, camera4);

	private final Chassis chassis = new Chassis();
	private final Ladder ladder = new Ladder();
	private final Algae algae = new Algae(ladder);
	private final Coral coral = new Coral(ladder);
	private final Climber climber = new Climber();
	private final PhotonCamera camera1 = new PhotonCamera("Camera1");
    private final PhotonCamera camera2 = new PhotonCamera("Camera2");
    private final PhotonCamera camera3 = new PhotonCamera("Camera3");
    private final PhotonCamera camera4 = new PhotonCamera("Camera4");
	private final List<PhotonCamera> cameras = List.of(camera1, camera2, camera3, camera4);

	private final Vision vision = new Vision(cameras.get(0), cameras.get(1), cameras.get(2), cameras.get(3));
	
	private final Autos auton = new Autos(chassis, ladder, algae, coral, climber);

	RunCommand climberStow = new RunCommand(() -> climber.setClimberPos(Climber.ClimberSP.STOW), climber);
	RunCommand climberReady = new RunCommand(() -> climber.setClimberPos(Climber.ClimberSP.READY), climber);
	RunCommand climberZero = new RunCommand(() -> climber.setClimberPos(Climber.ClimberSP.ZERO), climber);
	RunCommand climberClimb = new RunCommand(() -> climber.setClimberPos(Climber.ClimberSP.CLIMB), climber);

	RunCommand algaeStow = new RunCommand(() -> algae.setTiltPos(Algae.AlgaeSP.STOW), algae);
	RunCommand algaeZero = new RunCommand(() -> algae.setTiltPos(Algae.AlgaeSP.ZERO), algae);
	RunCommand algaeBarge = new RunCommand(() -> algae.setTiltPos(Algae.AlgaeSP.BARGE), algae);
	RunCommand algaeProcessor = new RunCommand(() -> algae.setTiltPos(Algae.AlgaeSP.PROCESSOR), algae);
	RunCommand algaeFloor = new RunCommand(() -> algae.setTiltPos(Algae.AlgaeSP.FLOOR), algae);
	RunCommand algaeL3 = new RunCommand(() -> algae.setTiltPos(Algae.AlgaeSP.L3), algae);
	RunCommand algaeL2 = new RunCommand(() -> algae.setTiltPos(Algae.AlgaeSP.L2), algae);

	RunCommand coralStow = new RunCommand(() -> coral.setTiltPos(Coral.CoralSP.STOW), coral);
	RunCommand coralZero = new RunCommand(() -> coral.setTiltPos(Coral.CoralSP.ZERO), coral);
	RunCommand coralStation = new RunCommand(() -> coral.setTiltPos(Coral.CoralSP.STATION), coral);
	RunCommand coralL4 = new RunCommand(() -> coral.setTiltPos(Coral.CoralSP.L4), coral);
	RunCommand coralL3 = new RunCommand(() -> coral.setTiltPos(Coral.CoralSP.L3), coral);
	RunCommand coralL2 = new RunCommand(() -> coral.setTiltPos(Coral.CoralSP.L2), coral);
	RunCommand coralL1 = new RunCommand(() -> coral.setTiltPos(Coral.CoralSP.L1), coral);

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController m_driverController = new CommandXboxController(
			OIConstants.kDriverControllerPort);
	private final CommandXboxController m_operatorController = new CommandXboxController(
			OIConstants.kOperatorControllerPort);
	private final GenericHID m_operatorHID = new GenericHID(
			OIConstants.kOperatorControllerPort);

	private final ShuffleboardTab cmdTab = Shuffleboard.getTab("Commands");
	private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
	// private final ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");
	// private final ShuffleboardTab coralTab = Shuffleboard.getTab("Coral");
	// private final ShuffleboardTab algaeTab = Shuffleboard.getTab("Algae");
	// private final ShuffleboardTab ladderTab = Shuffleboard.getTab("Ladder");
	// private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		cmdTab.add("climberStow", climberStow);
		cmdTab.add("climberReady", climberReady);
		cmdTab.add("climberZero", climberZero);
		cmdTab.add("climberClimb", climberClimb);

		cmdTab.add("algaeStow", algaeStow);
		cmdTab.add("algaeZero", algaeZero);
		cmdTab.add("algaeBarge", algaeBarge);
		cmdTab.add("algaeProcessor", algaeProcessor);
		cmdTab.add("algaeFloor", algaeFloor);
		cmdTab.add("algaeL3", algaeL3);
		cmdTab.add("algaeL2", algaeL2);

		cmdTab.add("coralStow", coralStow);
		cmdTab.add("coralZero", coralZero);
		cmdTab.add("coralStation", coralStation);
		cmdTab.add("coralL4", coralL4);
		cmdTab.add("coralL3", coralL3);
		cmdTab.add("coralL2", coralL2);
		cmdTab.add("coralL1", coralL1);

		compTab.add("Chassis", chassis);
		compTab.add("Coral", coral);
		compTab.add("Algae", algae);
		compTab.add("Ladder", ladder);
		compTab.add("Climber", climber);
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
								-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
								-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
								-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
								false),
						chassis));

		// climber.setDefaultCommand(
		// 		// The right Y stick controls movement
		// 		new RunCommand(
		// 				() -> climber.moveClimber(
		// 						-MathUtil.applyDeadband(m_operatorController.getRightY(), 0.10)),
		// 				// OIConstants.kDriveDeadband)),
		// 				climber));

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

		ShuffleboardLayout algaeCommands = Shuffleboard.getTab("Competition")
				.getLayout("Algae Commands", BuiltInLayouts.kList)
				.withSize(2, 5)
				.withPosition(0, 1)
				.withProperties(Map.of("Label position", "TOP"));
		algaeCommands.add("Barge", algae.algaeBarge());
		algaeCommands.add("L3", algae.algaeL3());
		algaeCommands.add("L2", algae.algaeL2());
		algaeCommands.add("Processor", algae.algaeProcessor());
		algaeCommands.add("Floor", algae.algaeFloor());
		algaeCommands.add("Stow", algae.algaeStow());
		algaeCommands.add("Intake", algae.algaeIntake());
		algaeCommands.add("Eject", algae.algaeEject());

		ShuffleboardLayout coralCommands = Shuffleboard.getTab("Competition")
				.getLayout("Coral", BuiltInLayouts.kList)
				.withSize(2, 5)
				.withPosition(2, 1)
				.withProperties(Map.of("Label position", "TOP"));
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

		ShuffleboardLayout toggleCommands = Shuffleboard.getTab("Competition")
				.getLayout("Toggle", BuiltInLayouts.kList)
				.withSize(2, 2)
				.withPosition(4, 4)
				.withProperties(Map.of("Label position", "TOP"));
		toggleCommands.add("Extract", algae.toggleExtractCmd());
		toggleCommands.add("Left-Right", coral.toggleSideCmd());

		ShuffleboardLayout climberCommands = Shuffleboard.getTab("Competition")
				.getLayout("Climber", BuiltInLayouts.kList)
				.withSize(2, 2)
				.withPosition(4, 1)
				.withProperties(Map.of("Label position", "TOP"));
		climberCommands.add("Climb", climber.climberClimb());
		climberCommands.add("Ready", climber.climberReady());
		climberCommands.add("Stow", climber.climberStow());
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

		m_driverController.y().onTrue(new RunCommand(() -> climber.setClimberPos(Climber.ClimberSP.STOW), climber));

		m_driverController.b().onTrue(new RunCommand(() -> climber.setClimberPos(Climber.ClimberSP.READY), climber));

		m_driverController.a().onTrue(new RunCommand(() -> climber.setClimberPos(Climber.ClimberSP.CLIMB), climber));

		m_driverController.x().onTrue(new RunCommand(() -> climber.setClimberPos(Climber.ClimberSP.ZERO), climber));

		new POVButton(m_operatorHID, 0).onTrue(new ParallelCommandGroup(
				ladder.setLadderSPCmd(Ladder.LadderSP.BARGE),
				coral.setTiltSPCmd(Coral.CoralSP.STOW),
				algae.setTiltSPCmd(Algae.AlgaeSP.BARGE)));

		new POVButton(m_operatorHID, 90).onTrue(new ParallelCommandGroup(
				ladder.setLadderSPCmd(Ladder.LadderSP.PROCESSOR),
				coral.setTiltSPCmd(Coral.CoralSP.STOW),
				algae.setTiltSPCmd(Algae.AlgaeSP.PROCESSOR)));

		new POVButton(m_operatorHID, 180).onTrue(new ParallelCommandGroup(
				ladder.setLadderSPCmd(Ladder.LadderSP.FLOOR),
				coral.setTiltSPCmd(Coral.CoralSP.STOW),
				algae.setTiltSPCmd(Algae.AlgaeSP.FLOOR)));

		new POVButton(m_operatorHID, 270).onTrue(new ParallelCommandGroup(
				ladder.setLadderSPCmd(Ladder.LadderSP.STATION),
				coral.setTiltSPCmd(Coral.CoralSP.STATION),
				algae.setTiltSPCmd(Algae.AlgaeSP.STOW)));

		m_operatorController.y().onTrue(new ParallelCommandGroup(
				ladder.setLadderSPCmd(Ladder.LadderSP.L4),
				coral.setTiltSPCmd(Coral.CoralSP.L4),
				algae.setTiltSPCmd(Algae.AlgaeSP.STOW)));

		m_operatorController.b().onTrue(new ParallelCommandGroup(
				ladder.setLadderSPCmd(Ladder.LadderSP.L3),
				coral.setTiltSPCmd(Coral.CoralSP.L3),
				algae.setTiltSPCmd(Algae.AlgaeSP.STOW)));

		m_operatorController.a().onTrue(new ParallelCommandGroup(
				ladder.setLadderSPCmd(Ladder.LadderSP.L2),
				coral.setTiltSPCmd(Coral.CoralSP.L2),
				algae.setTiltSPCmd(Algae.AlgaeSP.STOW)));

		m_operatorController.x().onTrue(new ParallelCommandGroup(
				ladder.setLadderSPCmd(Ladder.LadderSP.L1),
				coral.setTiltSPCmd(Coral.CoralSP.L1),
				algae.setTiltSPCmd(Algae.AlgaeSP.STOW)));

		m_operatorController.leftStick().onTrue(new ParallelCommandGroup(
				ladder.setLadderSPCmd(Ladder.LadderSP.STOW),
				coral.setTiltSPCmd(Coral.CoralSP.STOW),
				algae.setTiltSPCmd(Algae.AlgaeSP.STOW)));

		m_operatorController.leftBumper().onTrue(new ParallelCommandGroup(
				ladder.setLadderPosCmd(),
				coral.setTiltPosCmd(),
				algae.setTiltPosCmd()));

		m_operatorController.rightBumper().onTrue(new ParallelCommandGroup(
				coral.doActionCmd(),
				algae.doActionCmd()));

		m_operatorController.start().debounce(0.1, DebounceType.kRising)
				.onTrue(coral.toggleSideCmd());

		m_operatorController.back().debounce(0.1, DebounceType.kRising)
				.onTrue(algae.toggleExtractCmd());

		m_driverController.leftBumper()
				.onTrue(coral.setLeftIntakeCmd(Constants.Coral.INTAKE));
		m_driverController.rightBumper()
				.onTrue(coral.setLeftIntakeCmd(Constants.Coral.EJECT));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return auton.getChooser().getSelected();
	}
}
