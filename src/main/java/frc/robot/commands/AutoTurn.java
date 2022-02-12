/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoTurn extends CommandBase {
  /**
   * Creates a new AutoTurn.
   */
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
  private final double m_angle; 

  public AutoTurn(double angle , DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_angle = angle;
    m_DrivetrainSubsystem = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(" AutoTurn " + m_angle + "degrees");
    m_DrivetrainSubsystem.resetAngle();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DrivetrainSubsystem.TurnControl(m_angle);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.TurnControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_DrivetrainSubsystem.getAngle()) >= m_angle;
  }
}
