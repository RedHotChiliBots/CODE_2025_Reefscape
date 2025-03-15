// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Ladder;

public class Autos {
  private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

  // Define a chooser for autonomous commands
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  Chassis chassis;
  Ladder ladder;
  Algae algae;
  Coral coral;
  Climber climber;

  AutonLeave autoLeave;
  Command autoLeave2;
  
  /** Example static factory for an autonomous command. */
  // public static Command AutonLeave(Chassis chassis, Ladder ladder, Algae algae,
  // Coral coral, Climber climber) {
  // return Commands.sequence(new AutonLeave(chassis, ladder, algae, coral,
  // climber));
  // }

  public Autos(Chassis chassis, Ladder ladder, Algae algae, Coral coral, Climber climber) {

    System.out.println("+++++ Starting Autos Constructor +++++");

    this.chassis = chassis;
    this.ladder = ladder;
    this.algae = algae;
    this.coral = coral;
    this.climber = climber;

    // ********************************************
    // Add Auton Command chooser to Shuffleboard
    compTab.add("Auton Command", chooser)
        .withWidget("ComboBox Chooser")
        .withPosition(8, 1)
        .withSize(3, 1);

    this.autoLeave = new AutonLeave(chassis, ladder, algae, coral, climber);

    this.autoLeave2 = new InstantCommand(() -> chassis.drive(0.25, 0.0, 0.0, true))
        .andThen(new WaitCommand(1.0))
        .andThen(() -> chassis.drive(0.0, 0.0, 0.0, true));

    // ********************************************
    // Generate Auto commands
    // Note: Named commands used in Auto command must be defined
    // before defining the Auto command

    String temp = AutoBuilder.isConfigured() ? "IS" : "IS NOT";
    DriverStation.reportWarning("AutoBuilder " + temp + " configured", false);
    temp = AutoBuilder.isPathfindingConfigured() ? "IS" : "IS NOT";
    DriverStation.reportWarning("AutoBuilder Pathfinding " + temp + " configured", false);

    // AutonShootStay autoShootStay = new AutonShootStay(chassis, intake, feeder,
    // shooter);
    // AutonSpeakerAmp autoSpeakerAmp = new AutonSpeakerAmp(chassis, this, vision,
    // intake, feeder, shooter);

    // ********************************************
    // Initialize auto command chooser with auton commands
    // chooser = AutoBuilder.buildAutoChooser();

    chooser.setDefaultOption("Leave", autoLeave);
    chooser.addOption("Leave2", autoLeave2);
    // chooser.addOption("Shoot N Stay", autoShootStay);
    // chooser.addOption("Speaker Amp", autoSpeakerAmp);
    // chooser.addOption("Auto ZigZag3Cmd", cmdAutoZigZag3m);

    System.out.println("----- Ending Autos Constructor -----");
  }

  public SendableChooser<Command> getChooser() {
    return chooser;
  }
}
