// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final Spark Lightbulb = new Spark(3);

  public LEDSubsystem() {

  }

  public void Shot_blue(){
    Lightbulb.set(-0.83);
  }
  public void Heartbeat_Red(){
    Lightbulb.set(-0.25);
  }
  public void Gold(){
    Lightbulb.set(0.67);
  }
  public void Lime(){
    Lightbulb.set(0.73);
  }
  public void White(){
    Lightbulb.set(0.93);
  }
  
  public void HotPink(){
    Lightbulb.set(0.57);
  }
  
  public void BlueViolent(){
    Lightbulb.set(0.87);
  }

  public void Aqua(){
    Lightbulb.set(0.81);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
