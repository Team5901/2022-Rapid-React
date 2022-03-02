/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotPorts;
import frc.robot.Constants.ShooterConstants;



public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private final WPI_TalonFX ShooterMotor = new WPI_TalonFX(RobotPorts.kShooterMotor);

   public ShooterSubsystem() {

    /* Config the peak and nominal outputs */
    ShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    /* Config the peak and nominal outputs */
    ShooterMotor.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
    ShooterMotor.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
    ShooterMotor.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
    ShooterMotor.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);

    // Reverse motor direction
    ShooterMotor.setInverted(false);

    //Set PID values
    ShooterMotor.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.kF, 0);
    ShooterMotor.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kP, 0);
    ShooterMotor.config_kI(ShooterConstants.kPIDLoopIdx, ShooterConstants.kI, 0);
    ShooterMotor.config_kD(ShooterConstants.kPIDLoopIdx, ShooterConstants.kD, 0);
    ShooterMotor.configClosedloopRamp(1);
    ShooterMotor.setSensorPhase(true);
  }

  public void shooterSpeedUp(double RPM){
      double targetVelocity_UnitsPer100ms = RPM * 2048 / 600;
      ShooterMotor.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }

  public double getShooterRPM(){
      return (ShooterMotor.getSelectedSensorVelocity()/2048*600);
  }

  public void stopShooter(){
    ShooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM",getShooterRPM());
    //This method will be called once per scheduler run
  }
}
