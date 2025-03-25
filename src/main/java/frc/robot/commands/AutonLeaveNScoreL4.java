// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Algae.AlgaeSP;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Coral.CoralSP;
import frc.robot.subsystems.Ladder;
import frc.robot.subsystems.Ladder.LadderSP;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonLeaveNScoreL4 extends SequentialCommandGroup {
  /** Creates a new AutonShootLeave. */

  public AutonLeaveNScoreL4(RobotContainer robotContainer, Chassis chassis, Ladder ladder, Algae algae, Coral coral,
      Climber climber) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new ChassisDriveTime(chassis, -0.25, 1.25).until(() -> chassis.isJerk()),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> ladder.isLadderZeroed()),
                new InstantCommand(() -> ladder.setLadderPos(LadderSP.L4), ladder),
                new InstantCommand(() -> algae.setTiltPos(AlgaeSP.STOWDN), algae),
                new WaitCommand(0.75),
                new InstantCommand(() -> coral.setTiltPos(CoralSP.L4), coral))),
        new WaitUntilCommand(() -> ladder.onTarget()),
        new InstantCommand(() -> coral.doAction()),
        new ParallelCommandGroup(
            new CoralEject(coral).onlyIf(() -> coral.getIntakeEject() == Coral.IntakeEject.EJECT),
            new CoralIntake(coral).onlyIf(() -> coral.getIntakeEject() == Coral.IntakeEject.INTAKE),
            new AlgaeEject(algae).onlyIf(() -> algae.getIntakeEject() == Algae.IntakeEject.EJECT),
            new AlgaeIntake(algae).onlyIf(() -> algae.getIntakeEject() == Algae.IntakeEject.INTAKE)),
        new InstantCommand(() -> algae.setIntakeEject(Algae.IntakeEject.STOP)),
        new InstantCommand(() -> coral.setIntakeEject(Coral.IntakeEject.STOP)));
  }
}

// new WaitUntilCommand(() -> ladder.isLadderZeroed()),
// new InstantCommand(() -> ladder.setLadderPos(LadderSP.L35), ladder),
// new InstantCommand(() -> algae.setTiltPos(AlgaeSP.L35), algae),
// new WaitCommand(0.75),
// new InstantCommand(() -> coral.setTiltPos(CoralSP.L35), coral));
