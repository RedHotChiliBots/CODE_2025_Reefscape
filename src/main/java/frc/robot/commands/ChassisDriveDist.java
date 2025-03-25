// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;

public class ChassisDriveDist extends Command {
  /** Creates a new ChassisDrive. */

  private Chassis chassis = null;
  private Translation2d initTrans = null;
  private double dist = 0.0;
  private double vel = 0.0;

  public ChassisDriveDist(Chassis chassis, double vel, double dist) {
    this.chassis = chassis;
    this.dist = dist;
    this.vel = vel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTrans = chassis.getPose().getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.drive(vel, 0.0, 0.0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.drive(0.0, 0.0, 0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currDist = chassis.getPose().getTranslation().getDistance(initTrans);
    return currDist >= dist;
  }
}
