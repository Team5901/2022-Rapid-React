/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//STARTING POSITION: INCHES FROM WALL

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoFarLeftShoot5 extends SequentialCommandGroup {
  /**
   * Creates a new AutoFarLeftShoot5.
   */
  public AutoFarLeftShoot5(DrivetrainSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shoot, VisionSubsystem see) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
  
       addCommands(
      new AutoDrive(86.63, drive),
      new ParallelCommandGroup(
        new IntakeIn(intake).withTimeout(2),
        new AutoDrive(76, drive)),
      new AutoTurn(-135, drive),
      
      new ParallelCommandGroup(
        new AutoDrive(175.86, drive),
        new AutoAim(drive, see).withTimeout(4)),
      new ShootBall(shoot, intake).withTimeout(4),
      new AutoTurn(180, drive).withTimeout(4),
      new AutoDrive(30, drive)
      
      );
    
  }
}
