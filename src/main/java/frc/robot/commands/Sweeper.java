/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class Sweeper extends CommandBase {
  final IntakeSubsystem m_IntakeSubsystem;
  private final Timer m_timer = new Timer();
  /**
   * Creates a new IntakeIn.
   */
  public Sweeper(IntakeSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = subsystem;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_IntakeSubsystem.solenoidOn();
    System.out.println("YOU JUST GOT SWEPT!!!!!!");
    
  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //start spinning wheels
    m_IntakeSubsystem.intakeMotorOn(-IntakeConstants.kIntakeSpeed);

    //f ball detected
     if (m_IntakeSubsystem.ballExist() && !m_IntakeSubsystem.towerFull()){
      System.out.println("Ball Exists");
      m_IntakeSubsystem.conveyorMotorOn(IntakeConstants.kConveyorSpeed);
    }
    else  {
       m_IntakeSubsystem.conveyorMotorOff();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.solenoidOff();
    m_IntakeSubsystem.intakeMotorOff();
    m_IntakeSubsystem.conveyorMotorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
