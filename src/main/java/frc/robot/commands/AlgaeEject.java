// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;

public class AlgaeEject extends Command {
  /** Creates a new ChassisDrive. */

  private Algae algae = null;
  private final Timer timer = new Timer();

  public AlgaeEject(Algae algae) {
    this.algae = algae;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
    algae.setIntakeVel(Constants.Coral.EJECT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algae.setIntakeVel(Constants.Coral.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.0) ;
  }
}
