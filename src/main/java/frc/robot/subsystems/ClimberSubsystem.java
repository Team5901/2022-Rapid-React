/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimberSubsystem.
   */

  private final WPI_TalonSRX LeftClimber = new WPI_TalonSRX(Constants.RobotPorts.kLeftClimberMotor);
  private final WPI_TalonSRX RightClimber= new WPI_TalonSRX(Constants.RobotPorts.kRightClimberMotor);
  
  public ClimberSubsystem() {
    LeftClimber.configOpenloopRamp(0.5);
    RightClimber.configOpenloopRamp(0.5);
  }

  public void setTheclimb(double value) {
    LeftClimber.set(-value);
    RightClimber.set(-value);
    
 
  }
  public void stopTheclimb() {
    LeftClimber.stopMotor();
    RightClimber.stopMotor();
  }
  public double getClimberheight() {
    return LeftClimber.getSelectedSensorPosition();

  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climberheight", getClimberheight());

    // This method will be called once per scheduler run
  }
}
