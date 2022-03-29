// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class TeleopDrive extends CommandBase {
  /** Creates a new TeleopDrive. */
  final DrivetrainSubsystem m_DrivetrainSubsystem;
  final LEDSubsystem m_LEDSubsystem;
  private double m_fwd;
  private double m_rot;
  private boolean m_climbMode;
  private boolean m_turboMode;


  public TeleopDrive(double fwd,double rot,boolean climbMode, boolean turboMode,DrivetrainSubsystem subsystem1, LEDSubsystem subsystem2) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem1, subsystem2);
    m_fwd = fwd;
    m_rot = rot;
    m_climbMode = climbMode;
    m_turboMode = turboMode;
    m_DrivetrainSubsystem = subsystem1;
    m_LEDSubsystem = subsystem2;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
      if(m_climbMode == true){
        m_LEDSubsystem.HotPink();
      }
      else if(m_turboMode == true){
        m_LEDSubsystem.Heartbeat_Red();
      }


      m_DrivetrainSubsystem.cougarDrive(m_fwd,m_rot,m_climbMode,m_turboMode);
  
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
