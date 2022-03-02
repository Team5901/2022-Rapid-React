// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOut extends CommandBase {
  final IntakeSubsystem m_IntakeSubsystem;
  public IntakeOut(IntakeSubsystem subsystem) {
    m_IntakeSubsystem = subsystem;
    addRequirements(subsystem);
  }

// //public class IntakeOut extends CommandBase {
//   /** Creates a new IntakeOut. */
//   public IntakeOut() {
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.PistonOut();
    System.out.println("Piston is out");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.IntakeOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.IntakeStop(); 
    m_IntakeSubsystem.PistonIn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
