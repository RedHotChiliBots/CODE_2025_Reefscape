// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;

public class ChassisTimedDrive extends Command {
  /** Creates a new ChassisDrive. */

  private Chassis chassis = null;
  private double sec = 0.0;
  private double vel = 0.0;
  private final Timer timer = new Timer();

  public ChassisTimedDrive(Chassis chassis, double vel, double sec) {
    this.chassis = chassis;
    this.sec = sec;
    this.vel = vel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.drive(vel, 0.0, 0.0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.drive(0.0, 0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(sec);
  }
}
