/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotPorts;


public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */
  private final WPI_TalonSRX boss = new WPI_TalonSRX(RobotPorts.kIntakeMotor);
  private final WPI_TalonSRX fronzack = new WPI_TalonSRX(RobotPorts.kConveyorMotor);
  private final Compressor joe = new Compressor(0);
  private final Solenoid phil = new Solenoid(RobotPorts.kIntakeSolenoid);
  private final DigitalInput bill = new DigitalInput(0);
  private final DigitalInput bob = new DigitalInput(1);
  public IntakeSubsystem() {
    boss.setInverted(true);
    boss.configVoltageCompSaturation(11);
    boss.enableVoltageCompensation(true);

  }
 
  public void conveyorMotorOn(double power){
    fronzack.set(power);

  }

  public void conveyorMotorOff(){
   fronzack.set(0);

  }

  public void intakeMotorOn(double power){
    boss.set(power);

  }

  public void intakeMotorOff(){
    boss.stopMotor();
  }
  
  public void compressorOn(){
    joe.setClosedLoopControl(true);
  }

  public void compressorOff(){
    joe.setClosedLoopControl(false);

  }

  public void solenoidOn(){
    phil.set(true);

  }

  public void solenoidOff(){
    phil.set(false);

  }

  public boolean ballExist(){
    //reverse input since 1 = no ball 0 = ball
    return !bill.get();
 }

 public boolean towerFull(){
  //reverse input since 1 = no ball 0 = ball
  return !bob.get();
}


    

  @Override
  public void periodic()
   {
     SmartDashboard.putBoolean("Tower Full", towerFull());
    // This method will be called once per scheduler run
  }
}
