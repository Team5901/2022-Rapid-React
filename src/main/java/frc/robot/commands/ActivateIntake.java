// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ActivateIntake extends CommandBase {
  final IntakeSubsystem m_IntakeSubsystem;
  final LEDSubsystem m_LEDSubsystem;
  public ActivateIntake(IntakeSubsystem subsystem,LEDSubsystem subsystem2) {
    m_IntakeSubsystem = subsystem;
    m_LEDSubsystem = subsystem2;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.PistonOut();
    System.out.println("ACTIVATEINTAKE Command is running");
  }

  //This is for the motors on the intake
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.IntakeIn();
    m_IntakeSubsystem.MidtakeOn();

    if(m_IntakeSubsystem.ballExist()){
      m_LEDSubsystem.Lime();
    }
    else{
      m_LEDSubsystem.White();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.PistonIn();
    m_IntakeSubsystem.IntakeStop();
    m_IntakeSubsystem.MidtakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
