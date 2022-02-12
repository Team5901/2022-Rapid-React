/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class GearShift extends CommandBase {
  final DrivetrainSubsystem m_DrivetrainSubsystem;
/*

*/
  /**
   * Creates a new GearShift.
   */
  public GearShift(DrivetrainSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DrivetrainSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  //tried using execute instead of initialized
  @Override
  public void initialize() {
    //when piston is in, shiftStatus = false
    System.out.println("Gearshift");
    if (m_DrivetrainSubsystem.shiftStatus()){
      m_DrivetrainSubsystem.shiftIn();
    }
    else{
      m_DrivetrainSubsystem.shiftOut();
    }
   
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
