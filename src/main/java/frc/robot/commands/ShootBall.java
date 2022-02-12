/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import some sort of timer
public class ShootBall extends CommandBase {
  /**
   * Creates a new BallShooter.
   */
  final ShooterSubsystem m_ShooterSubsystem;
  final IntakeSubsystem m_IntakeSubsystem;

  public ShootBall(ShooterSubsystem subsystem1, IntakeSubsystem subsystem2) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = subsystem1;
    m_IntakeSubsystem = subsystem2;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.shooterSpeedUp(Constants.ShooterConstants.kShooter_LineRPM);

    if(m_ShooterSubsystem.getShooterRPM() > Constants.ShooterConstants.kShooter_LineRPM-250){
      m_IntakeSubsystem.conveyorMotorOn(Constants.IntakeConstants.kConveyorSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.stopShooter();
    m_IntakeSubsystem.conveyorMotorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
