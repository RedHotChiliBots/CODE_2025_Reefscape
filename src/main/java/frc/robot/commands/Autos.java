// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Ladder;

public class Autos {
  private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

  // Define a chooser for autonomous commands
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();;

  RobotContainer robotContainer;
  Chassis chassis;
  Ladder ladder;
  Algae algae;
  Coral coral;
  Climber climber;

  // AutonLeave autoLeave;
  // Command autoLeave2;
  
  /** Example static factory for an autonomous command. */
  // public static Command AutonLeave(Chassis chassis, Ladder ladder, Algae algae,
  // Coral coral, Climber climber) {
  // return Commands.sequence(new AutonLeave(chassis, ladder, algae, coral,
  // climber));
  // }

  public Autos(RobotContainer robotContainer, Chassis chassis, Ladder ladder, Algae algae, Coral coral, Climber climber) {

    System.out.println("+++++ Starting Autos Constructor +++++");

    this.robotContainer = robotContainer;
    this.chassis = chassis;
    this.ladder = ladder;
    this.algae = algae;
    this.coral = coral;
    this.climber = climber;

    // ********************************************
    // Add Auton Command chooser to Shuffleboard
    compTab.add("Auton Command", chooser)
        .withWidget("ComboBox Chooser")
        .withPosition(0, 0)
        .withSize(3, 1);

    // ********************************************
    // Add Auton Command chooser to Shuffleboard
    compTab.add("Auto Command", autoChooser)
        .withWidget("ComboBox Chooser")
        .withPosition(0, 3)
        .withSize(3, 1);

    // this.autoLeave0 = new AutonLeave(chassis, ladder, algae, coral, climber);

    // this.autoLeave = new InstantCommand(() -> chassis.drive(0.25, 0.0, 0.0, true))
    //     .andThen(new WaitCommand(1.0))
    //     .andThen(() -> chassis.drive(0.0, 0.0, 0.0, true));

    // this.autoLeaveNScoreL1 = new InstantCommand(() -> chassis.drive(0.25, 0.0,
    // 0.0, true))
    // .andThen(new WaitCommand(1.0))
    // .alongWith(robotContainer.goL1)
    // .andThen(() -> chassis.drive(0.0, 0.0, 0.0, true))
    // .andThen(new WaitCommand(0.1))
    // .andThen(robotContainer.doAction);

    // this.autoLeaveNScoreL4 = new InstantCommand(() -> chassis.drive(0.25, 0.0, 0.0, true))
    //     .andThen(new WaitCommand(1.0))
    //     .alongWith(robotContainer.goL4)
    //     .andThen(new InstantCommand(() -> chassis.drive(0.0, 0.0, 0.0, true)))
    //     .andThen(new WaitCommand(0.1))
    //     .andThen(robotContainer.doAction);

    // this.autoLeave = new InstantCommand(() -> chassis.drive(0.25, 0.0, 0.0,
    // true))
    // .andThen(new WaitCommand(1.0))
    // .andThen(() -> chassis.drive(0.0, 0.0, 0.0, true));

    // this.autoLeaveNScoreL1 = new InstantCommand(() -> chassis.drive(0.25, 0.0,
    //     0.0, true))
    //     .andThen(new WaitCommand(1.0))
    //     .alongWith(robotContainer.goL1)
    //     .andThen(() -> chassis.drive(0.0, 0.0, 0.0, true))
    //     .andThen(new WaitCommand(0.1))
    //     .andThen(robotContainer.doAction);

    // this.autoLeaveNScoreL4 = new InstantCommand(() -> chassis.drive(0.25, 0.0,
    //     0.0, true))
    //     .andThen(new WaitCommand(1.0))
    //     .alongWith(robotContainer.goL4)
    //     .andThen(new InstantCommand(() -> chassis.drive(0.0, 0.0, 0.0, true)))
    //     .andThen(new WaitCommand(0.1))
    //     .andThen(robotContainer.doAction);

    String temp = AutoBuilder.isConfigured() ? "IS" : "IS NOT";
    DriverStation.reportWarning("AutoBuilder " + temp + " configured", false);
    temp = AutoBuilder.isPathfindingConfigured() ? "IS" : "IS NOT";
    DriverStation.reportWarning("AutoBuilder Pathfinding " + temp + " configured", false);

    // ********************************************
    // Generate Auto commands
    // Note: Named commands used in Auto command must be defined
    // before defining the Auto command
    NamedCommands.registerCommand("Leave", autoLeave());
    NamedCommands.registerCommand("LeaveNScoreL1", autoLeaveNScoreL1(robotContainer, chassis));
    NamedCommands.registerCommand("LeaveNScoreL4", autoLeaveNScoreL4(robotContainer, chassis));

    // ********************************************
    // Initialize auto command chooser with auton commands
    // chooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Leave", autoLeave());
    autoChooser.addOption("LeaveNScoreL1", autoLeaveNScoreL1(robotContainer, chassis));
    autoChooser.addOption("LeaveNScoreL4", autoLeaveNScoreL4(robotContainer, chassis));

    chooser.setDefaultOption("Leave", autoLeave());
    chooser.addOption("LeaveNScoreL1", autoLeaveNScoreL1(robotContainer, chassis));
    chooser.addOption("LeaveNScoreL4", autoLeaveNScoreL4(robotContainer, chassis));

    System.out.println("----- Ending Autos Constructor -----");
  }

  public Command autoLeave() {
    return Commands.sequence(
        new InstantCommand(() -> chassis.drive(0.25, 0.0, 0.0, true)),
        new WaitCommand(1.0),
        new InstantCommand(() -> chassis.drive(0.0, 0.0, 0.0, true)));
  }

  public Command autoLeaveNScoreL1(RobotContainer robotContainer, Chassis chassis) {
    return Commands.sequence(
        Commands.parallel(
            new InstantCommand(() -> chassis.driveCmd(0.25, 0.0, 0.0, true)),
            robotContainer.goL1)
            .withTimeout(1.0),
        new InstantCommand(() -> chassis.driveCmd(0.0, 0.0, 0.0, true)),
        robotContainer.doAction);
  }

  public Command autoLeaveNScoreL4(RobotContainer robotContainer, Chassis chassis) {
    return Commands.sequence(
        Commands.parallel(
            new InstantCommand(() -> chassis.driveCmd(0.25, 0.0, 0.0, true)),
            robotContainer.goL4)
            .withTimeout(1.0),
        new InstantCommand(() -> chassis.driveCmd(0.0, 0.0, 0.0, true)),
        robotContainer.doAction);
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  public SendableChooser<Command> getChooser() {
    return chooser;
  }
}
