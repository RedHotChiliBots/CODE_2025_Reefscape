// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.Chassis;

public class ChassisDrivePose extends Command {
  /** Creates a new ChassisDrive. */

  private Chassis chassis = null;
  private Pose2d currPose = null;
  private Pose2d tgtPose = null;
  private double tgtRot = 0.0;
  private double currRot = 0.0;
  private double xComp = 0.0;
  private double yComp = 0.0;

  private PIDController xCont = new PIDController(Vision.kXP, Vision.kXI, Vision.kXD);
  private PIDController yCont = new PIDController(Vision.kYP, Vision.kYI, Vision.kYD);
  private PIDController rotCont = new PIDController(Vision.kRP, Vision.kRI, Vision.kRD);

  public ChassisDrivePose(Chassis chassis, Pose2d tgtPose) {
    this.chassis = chassis;
    this.tgtPose = tgtPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xCont.setTolerance(Vision.kXTollerance);
    yCont.setTolerance(Vision.kYTollerance);
    rotCont.setTolerance(Vision.kRTollerance);
    rotCont.enableContinuousInput(Vision.kRMin, Vision.kRMax);
     
    tgtRot = tgtPose.getRotation().getRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currPose = chassis.getPose();
    currRot = currPose.getRotation().getRadians();

    xComp = currPose.minus(tgtPose).getX();
    yComp = currPose.minus(tgtPose).getY();

    double x = xCont.calculate(xComp, 0.0);
    double y = xCont.calculate(yComp, 0.0);
    double r = rotCont.calculate(currRot, tgtRot);

    chassis.drive(x, y, r, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.drive(0.0, 0.0, 0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xCont.atSetpoint() && yCont.atSetpoint() && rotCont.atSetpoint();
  }
}
