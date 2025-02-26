// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Ladder;
import frc.robot.subsystems.Vision;

public final class Autos {
  private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

  // Define a chooser for autonomous commands
  private final SendableChooser<Command> chooser;

  /** Example static factory for an autonomous command. */
  // public static Command AutonLeave(Chassis chassis, Ladder ladder, Algae algae, Coral coral, Climber climber) {
  //   return Commands.sequence(new AutonLeave(chassis, ladder, algae, coral, climber));
  // }

  public Autos(Chassis chassis, Vision vision, Ladder ladder, Algae algae, Coral coral, Climber climber) {

    System.out.println("+++++ Starting Autos Constructor +++++");

    // ********************************************
    // Generate Auto commands
    // Note: Named commands used in Auto command must be defined
    // before defining the Auto command
    AutonLeave autoLeave = new AutonLeave(chassis, vision, ladder, algae, coral, climber);
    // AutonShootStay autoShootStay = new AutonShootStay(chassis, intake, feeder,
    // shooter);
    // AutonSpeakerAmp autoSpeakerAmp = new AutonSpeakerAmp(chassis, this, vision,
    // intake, feeder, shooter);

    // ********************************************
    // Initialize auto command chooser with auton commands
    chooser = AutoBuilder.buildAutoChooser();

    chooser.setDefaultOption("Leave", autoLeave);
    // chooser.addOption("Shoot N Stay", autoShootStay);
    // chooser.addOption("Speaker Amp", autoSpeakerAmp);
    // chooser.addOption("Auto ZigZag3Cmd", cmdAutoZigZag3m);

    // ********************************************
    // Add Auton Command chooser to Shuffleboard
    compTab.add("Auton Command", chooser)
        .withWidget("ComboBox Chooser")
        .withPosition(6, 1)
        .withSize(4, 1);

    System.out.println("----- Ending Autos Constructor -----");
  }

  public SendableChooser<Command> getChooser() {
    return chooser;
  }
}
