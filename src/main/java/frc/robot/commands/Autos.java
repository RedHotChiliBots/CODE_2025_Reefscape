// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Ladder;

public class Autos {
  private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

  // Define a chooser for autonomous commands
  // private final SendableChooser<Command> chooser = new SendableChooser<>();
  private SendableChooser<Command> autoChooser = null;

  RobotContainer robotContainer;
  Chassis chassis;
  Ladder ladder;
  Algae algae;
  Coral coral;
  Climber climber;

  AutonLeave autonLeave;
  AutonLeaveNScoreL1 autonLeaveNScoreL1;
  AutonLeaveNScoreL4 autonLeaveNScoreL4;
  Pose2d startPose;
  Command resetPose;
  Command resetOdo;

  /** Example static factory for an autonomous command. */
  // public static Command AutonLeave(Chassis chassis, Ladder ladder, Algae algae,
  // Coral coral, Climber climber) {
  // return Commands.sequence(new AutonLeave(chassis, ladder, algae, coral,
  // climber));
  // }

  public Autos(RobotContainer robotContainer, Chassis chassis, Ladder ladder, Algae algae, Coral coral,
      Climber climber) {

    System.out.println("+++++ Starting Autos Constructor +++++");

    this.robotContainer = robotContainer;
    this.chassis = chassis;
    this.ladder = ladder;
    this.algae = algae;
    this.coral = coral;
    this.climber = climber;

    String temp = AutoBuilder.isConfigured() ? "IS" : "IS NOT";
    DriverStation.reportWarning("AutoBuilder " + temp + " configured", false);
    temp = AutoBuilder.isPathfindingConfigured() ? "IS" : "IS NOT";
    DriverStation.reportWarning("AutoBuilder Pathfinding " + temp + " configured", false);

    this.autonLeave = new AutonLeave(chassis, ladder, algae, coral, climber);
    this.autonLeaveNScoreL1 = new AutonLeaveNScoreL1(robotContainer, chassis, ladder, algae, coral, climber);
    this.autonLeaveNScoreL4 = new AutonLeaveNScoreL4(robotContainer, chassis, ladder, algae, coral, climber);

    this.startPose = new Pose2d(new Translation2d(7.3, 4.0), Rotation2d.fromDegrees(180));
    this.resetPose = new InstantCommand(() -> chassis.resetPose(startPose));
    this.resetOdo = new InstantCommand(() -> chassis.resetOdometry(startPose));

    // ********************************************
    // Generate Auto commands
    // Note: Named commands used in Auto command must be defined
    // before defining the Auto command
    NamedCommands.registerCommand("resetPose", resetPose);
    NamedCommands.registerCommand("resetOdo", resetOdo);
    NamedCommands.registerCommand("ExtractTrue", algae.setExtractTrue);
    NamedCommands.registerCommand("goL3", robotContainer.goL3);
    NamedCommands.registerCommand("goL35", robotContainer.goL35);
    NamedCommands.registerCommand("goBarge", robotContainer.goBarge);
    NamedCommands.registerCommand("goProcessor", robotContainer.goProcessor);
    NamedCommands.registerCommand("doAutonAction", robotContainer.doAutonAction);

    // ********************************************
    // Initialize auto command chooser with auton commands
    autoChooser = AutoBuilder.buildAutoChooser("BigKahuna");

    // ********************************************
    // Add Auton Command chooser to Shuffleboard
    compTab.add("Auto Command", autoChooser)
        .withWidget("ComboBox Chooser")
        .withPosition(0, 0)
        .withSize(3, 1);
        
    System.out.println("----- Ending Autos Constructor -----");
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }
}
