/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.management.remote.TargetedNotification;

import org.ejml.interfaces.decomposition.DecompositionSparseInterface;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;

public class VisionSubsystem extends SubsystemBase {

  public VisionSubsystem() {   

  }

  //Get Horizontal Offset Angle
  public double getTx(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  //Get Vertical Offset Angle
  public double getTy(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  //Get Target Area
  public double getTa(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }

  //Return true if target is available, otherwise false
  public boolean targetAvailable(){
    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) < 1.0)
    {
      return false;
    }
    else
    {
      return true;
    }  
  }

  public void turnOffLED(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  public void turnOnLED(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);    
  }

  public void setPipeline(int n){
    //select pipeline n, where n is 0-9
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(n); 
  }

  public void takeSnapshot(){
    //reset snapshot mode
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(1); 
  }

  public void resetSnapshot(){
    //take one snapshot
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(0); 
  }
  public double calculateDistance(double angle){
    double B;
    B = 72;
    double A;
    A = B/ Math.tan((angle + 54) * Math.PI / 180);
        
    return A;
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Limelight Target Angle", getTx());
    SmartDashboard.putBoolean("Limelight Target Available",targetAvailable());
    SmartDashboard.putNumber("Distance Estimation", calculateDistance(getTy()));
  }
  

// This method will be called once per scheduler run
}
