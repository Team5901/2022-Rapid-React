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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;



public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private final WPI_TalonFX mav = new WPI_TalonFX(10);

   public ShooterSubsystem() {

    /* Config the peak and nominal outputs */
    mav.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    /* Config the peak and nominal outputs */
    mav.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
    mav.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
    mav.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
    mav.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);

    // Reverse motor direction
    mav.setInverted(false);

    //Set PID values
    mav.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.kF, 0);
    mav.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kP, 0);
    mav.config_kI(ShooterConstants.kPIDLoopIdx, ShooterConstants.kI, 0);
    mav.config_kD(ShooterConstants.kPIDLoopIdx, ShooterConstants.kD, 0);
    mav.configClosedloopRamp(1);
    mav.setSensorPhase(true);
  }

  public void shooterSpeedUp(double RPM){
      double targetVelocity_UnitsPer100ms = RPM * 2048 / 600;
      mav.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
 
  }

  public double getShooterRPM(){
      return (mav.getSelectedSensorVelocity()/2048*600);
  }

  public void stopShooter(){
    mav.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM",getShooterRPM());
    //This method will be called once per scheduler run
  }

}
