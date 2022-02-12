/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoMidShoot extends SequentialCommandGroup {
  /**
   * Creates a new AutoMidShoot.
   */
  public AutoMidShoot(DrivetrainSubsystem drive, ShooterSubsystem shoot, IntakeSubsystem intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
      addCommands(
        new AutoTurn(135, drive),
        new ShootBall(shoot, intake).withTimeout(5),
        new AutoDrive(-50, drive)
      
      );
  }
}
