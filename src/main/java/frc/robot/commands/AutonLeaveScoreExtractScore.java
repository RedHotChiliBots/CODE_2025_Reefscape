// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Ladder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonLeaveScoreExtractScore extends SequentialCommandGroup {
  /** Creates a new AutonShootLeave. */

  public AutonLeaveScoreExtractScore(RobotContainer robotContainer, Chassis chassis, Ladder ladder, Algae algae,
      Coral coral,
      Climber climber) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutonLeaveNScoreL4(robotContainer, chassis, ladder, algae, coral, climber),
        robotContainer.goL35,
        new WaitUntilCommand(() -> ladder.onTarget()),
        new AlgaeIntake(algae),
        new ParallelRaceGroup(
            new WaitUntilCommand(() -> algae.isLimit()),
            new WaitCommand(4.0)),
        new ChassisDriveTime(chassis, 0.2, 0.15),
        new ChassisRotateDegree(chassis, 0.2, 90.0),
        new ParallelCommandGroup(
            new ChassisDriveTime(chassis, -0.25, 4.0).until(() -> chassis.isJerk()),
            robotContainer.goProcessor),
        new AlgaeEject(algae));
  }
}
