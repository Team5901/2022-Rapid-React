// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LEDSubsystem;

public class ShootHigh extends CommandBase {
  /** Creates a new ShootHigh. */
  final LEDSubsystem m_LEDSubsystem;
  final ShooterSubsystem m_ShooterSubsystem;

  public ShootHigh(ShooterSubsystem subsystem1, LEDSubsystem subsystem2) {
    m_ShooterSubsystem = subsystem1;
    m_LEDSubsystem = subsystem2;
    addRequirements(subsystem1, subsystem2);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shooter is working");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.shooterSpeedUp(ShooterConstants.kShoot_highRPM);

    if(m_ShooterSubsystem.getShooterRPM() > Constants.ShooterConstants.kShoot_highRPM-100){
      m_ShooterSubsystem.LoaderIn();
    m_LEDSubsystem.BlueViolent();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping SHOOTHIGH command - shooter and loader");
    m_ShooterSubsystem.stopShooter();
    m_ShooterSubsystem.LoaderStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
