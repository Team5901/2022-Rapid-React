/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotPorts;



public class IntakeSubsystem extends SubsystemBase {
  
  //Declare motors/solenoids/sensors related to intake subsystem here
  private final WPI_TalonSRX IntakeMotor = new WPI_TalonSRX(RobotPorts.kIntakeMotor);
  private final WPI_TalonFX LoadingMotor = new WPI_TalonFX(RobotPorts.kLoadingMotor);
  private final Solenoid IntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,RobotPorts.kIntakeSolenoid);
  private final DigitalInput IntakeSensor = new DigitalInput(9);


  public IntakeSubsystem() {
  }

  public void IntakeIn(){
    //set motor to output in positive direction
    IntakeMotor.set(IntakeConstants.kIntakeSpeed);
  }

  public void IntakeOut(){
    //set motor to output in positive direction
    IntakeMotor.set(-IntakeConstants.kIntakeSpeed);
  }
  public void IntakeStop(){
    //stops the motor
    IntakeMotor.stopMotor();
  }
  public void PistonOut() {
    //extends piston out
    IntakeSolenoid.set(true);
  }

  public void PistonIn() {
    //retracts piston in
    IntakeSolenoid.set(false);
  }

  public void LoaderIn(){
    //Pull ball in loader
    LoadingMotor.set(IntakeConstants.kLoaderSpeed);
  }

  public void LoaderOut(){ 
    //Push ball out of loader
    LoadingMotor.set(-IntakeConstants.kLoaderSpeed); 
  }

 public void  LoaderStop(){
    LoadingMotor.stopMotor();
  }

  public boolean ballExist(){
    //reverse input since 1 = no ball 0 = ball
    return !IntakeSensor.get();
 }

  @Override
  public void periodic()
   {
     SmartDashboard.getBoolean("Ball Sensor Tripped", ballExist());
    // This method will be called once per scheduler run
  }
}
