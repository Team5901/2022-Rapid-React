/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAim extends CommandBase {
  final DrivetrainSubsystem m_DrivetrainSubsystem;
  final VisionSubsystem m_VisionSubsystem;

  public AutoAim(DrivetrainSubsystem subsystem1, VisionSubsystem subsystem2) {
    m_DrivetrainSubsystem = subsystem1;
    m_VisionSubsystem = subsystem2;
    addRequirements(subsystem1,subsystem2);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_VisionSubsystem.turnOnLED();
    double TargetAngle = m_DrivetrainSubsystem.getAngle();

    if (m_VisionSubsystem.targetAvailable() == true && Math.abs(TargetAngle) >= 1.0 ){
        System.out.println("Target Acquired - Rotating");
        m_DrivetrainSubsystem.cougarDrive(Constants.DriveConstants.kVisionSpeedRatio, Constants.DriveConstants.kVisionTurnRatio*TargetAngle,false,false);

    }
    else if (m_VisionSubsystem.targetAvailable() == true && Math.abs(TargetAngle) < 1.0 ) {
        System.out.println("Target Angle Reached");
             
    }
    else {
        System.out.println("No Target Available");
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_VisionSubsystem.turnOffLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
