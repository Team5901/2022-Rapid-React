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

public class AutoDrive extends CommandBase {
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
  private final double m_distance;

  /**
   * 
   * Creates a new autodrive.
   */
  public AutoDrive(double distance, DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    //m_DrivetrainSubsystem = subsystem;
    //addRequirements();
    m_distance = distance;
    m_DrivetrainSubsystem = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(" Autodrive " + m_distance + " inches");
    m_DrivetrainSubsystem.resetEncoders();
  }
  
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  //18.84in(wheel circumference) = 0.48m. 
  public void execute() {
    System.out.println("AutoDrive Execute Distance: " + m_distance);
    m_DrivetrainSubsystem.AutoDroive(m_distance);
  }
  
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AutoDrive end function");
    m_DrivetrainSubsystem.AutoDroive(0);

    //m_DrivetrainSubsystem.cougarDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("AutoDrive isFinished function");
    return Math.abs(m_DrivetrainSubsystem.getAverageEncoderDistance()-m_distance) <=  Constants.DriveConstants.kAutoDistanceError;
  }
}
