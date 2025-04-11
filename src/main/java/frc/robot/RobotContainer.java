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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlgaeEject;
import frc.robot.commands.AlgaeIntake;
import frc.robot.commands.Autos;
import frc.robot.commands.CoralEject;
import frc.robot.commands.CoralIntake;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Algae.AlgaeSP;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Coral.CoralSP;
import frc.robot.subsystems.Ladder;
import frc.robot.subsystems.Ladder.LadderSP;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

import java.util.Map;

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
	private final Coral coral = new Coral(chassis, ladder, algae);
	private final Climber climber = new Climber();
	private final Vision vision = new Vision(
			chassis::addVisionMeasurement,
			new VisionIOPhotonVision(VisionConstants.camera0Name,
					VisionConstants.robotToCamera0),
			new VisionIOPhotonVision(VisionConstants.camera1Name,
					VisionConstants.robotToCamera1));

	// Define HIDs
	private final CommandXboxController m_driverController = new CommandXboxController(
			OIConstants.kDriverControllerPort);
	private final CommandXboxController m_operatorController = new CommandXboxController(
			OIConstants.kOperatorControllerPort);
	private final GenericHID m_operatorHID = new GenericHID(
			OIConstants.kOperatorControllerPort);

	public final Command goBarge = new SequentialCommandGroup(
			new WaitUntilCommand(() -> ladder.isLadderZeroed()),
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.BARGE), ladder),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.STOW), coral),
			new WaitCommand(0.5),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.BARGE), algae));

	public final Command goL4 = new SequentialCommandGroup(
			new WaitUntilCommand(() -> ladder.isLadderZeroed()),
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.L4), ladder),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.STOWDN), algae),
			new WaitCommand(0.75),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.L4), coral));

	public final Command goL35 = new SequentialCommandGroup(
			new WaitUntilCommand(() -> ladder.isLadderZeroed()),
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.L35), ladder),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.L35), algae),
			new WaitCommand(0.75),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.L35), coral));

	public final Command goL3 = new SequentialCommandGroup(
			new WaitUntilCommand(() -> ladder.isLadderZeroed()),
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.L3), ladder),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.L3), algae),
			new WaitCommand(0.75),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.L3), coral));

	public final Command goL2 = new SequentialCommandGroup(
			new WaitUntilCommand(() -> ladder.isLadderZeroed()),
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.L2), ladder),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.L2), algae),
			new WaitCommand(0.75),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.L2), coral));

	public final Command goL1 = new SequentialCommandGroup(
			new WaitUntilCommand(() -> ladder.isLadderZeroed()),
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.L1), ladder),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.STOWDN), algae),
			new WaitCommand(0.75),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.L1), coral));

	public final Command goStation = new SequentialCommandGroup(
			new WaitUntilCommand(() -> ladder.isLadderZeroed()),
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.STATION), ladder),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.STOWDN), algae),
			new WaitCommand(0.75),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.STATION), coral));

	public final Command goProcessor = new SequentialCommandGroup(
			new WaitUntilCommand(() -> ladder.isLadderZeroed()),
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.PROCESSOR), ladder),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.STOW), coral),
			new WaitCommand(0.75),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.PROCESSOR), algae));

	public final Command goFloor = new SequentialCommandGroup(
			new WaitUntilCommand(() -> ladder.isLadderZeroed()),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.FLOOR), algae),
			new WaitCommand(0.75),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.STOW), coral),
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.FLOOR), ladder));

	public final Command goStow = new SequentialCommandGroup(
			new WaitUntilCommand(() -> ladder.isLadderZeroed()),
			new InstantCommand(() -> coral.setTiltPos(CoralSP.STOW)),
			new WaitCommand(0.1),
			new InstantCommand(() -> algae.setTiltPos(AlgaeSP.STOWUP)),
			new WaitCommand(0.75),
			new InstantCommand(() -> ladder.setLadderPos(LadderSP.STOW)));

	public final Command doAction = new SequentialCommandGroup(
			new WaitUntilCommand(() -> ladder.onTarget()),
			new ParallelCommandGroup(
					new InstantCommand(() -> coral.doAction(), coral),
					new InstantCommand(() -> algae.doAction(), algae)),
			new ParallelCommandGroup(
					new CoralEject(coral).onlyIf(() -> coral.getIntakeEject() == Coral.IntakeEject.EJECT),
					new CoralIntake(coral).onlyIf(() -> coral.getIntakeEject() == Coral.IntakeEject.INTAKE),
					new AlgaeEject(algae).onlyIf(() -> algae.getIntakeEject() == Algae.IntakeEject.EJECT),
					new AlgaeIntake(algae).onlyIf(() -> algae.getIntakeEject() == Algae.IntakeEject.INTAKE)),
			new InstantCommand(() -> algae.setIntakeEject(Algae.IntakeEject.STOP)),
			new InstantCommand(() -> coral.setIntakeEject(Coral.IntakeEject.STOP)));

	public final Command doAutonAction = new SequentialCommandGroup(
			new WaitCommand(0.1),
			new WaitUntilCommand(() -> ladder.onTarget()),
			new ParallelCommandGroup(
					new InstantCommand(() -> coral.doAction(), coral),
					new InstantCommand(() -> algae.doAction(), algae)),
			new ParallelCommandGroup(
					new CoralEject(coral).onlyIf(() -> coral.getIntakeEject() == Coral.IntakeEject.EJECT),
					new CoralIntake(coral).onlyIf(() -> coral.getIntakeEject() == Coral.IntakeEject.INTAKE),
					new AlgaeEject(algae).onlyIf(() -> algae.getIntakeEject() == Algae.IntakeEject.EJECT),
					new AlgaeIntake(algae).onlyIf(() -> algae.getIntakeEject() == Algae.IntakeEject.INTAKE)),
			new InstantCommand(() -> algae.setIntakeEject(Algae.IntakeEject.STOP)),
			new InstantCommand(() -> coral.setIntakeEject(Coral.IntakeEject.STOP)),
			new WaitCommand(0.15));

	private final Autos auton = new Autos(this, chassis, ladder, algae, coral, climber);

	private final ShuffleboardTab cmdTab = Shuffleboard.getTab("Commands");

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

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
		goCommands.add("L35", goL35)
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

		m_operatorController.y().onTrue(this.goL4);
		m_operatorController.x().onTrue(this.goL3);
		m_operatorController.b().onTrue(this.goL2);
		m_operatorController.a().onTrue(algae.intake);

		new POVButton(m_operatorHID, 0).onTrue(this.goBarge);
		new POVButton(m_operatorHID, 90).onTrue(this.goStation);
		new POVButton(m_operatorHID, 270).onTrue(this.goProcessor);
		new POVButton(m_operatorHID, 180).onTrue(this.goFloor);

		// m_operatorController.start().onTrue(climber.ready);
		m_operatorController.back().onTrue(this.goStow);
		m_operatorController.start().onTrue(climber.climb);

		m_operatorController.leftBumper().onTrue(this.goL35);
		m_operatorController.rightBumper().onTrue(this.coral.eject); // was doAction 10:35 AM

		m_operatorController.rightStick().onTrue(algae.eject);
		m_operatorController.leftStick().onTrue(coral.intake);

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
