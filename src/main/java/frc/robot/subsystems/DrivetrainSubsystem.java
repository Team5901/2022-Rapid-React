/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotPorts;

public class DrivetrainSubsystem extends SubsystemBase {
  
  PowerDistribution PowerDistributionPanel = new PowerDistribution();
  
  private final WPI_TalonFX leftFrontDriveMotor = new WPI_TalonFX(RobotPorts.kLeftFrontMotor);
  private final WPI_TalonFX leftRearDriveMotor = new WPI_TalonFX(RobotPorts.kLeftRearMotor);
  private final WPI_TalonFX rightFrontDriveMotor = new WPI_TalonFX(RobotPorts.kRightFrontMotor);
  private final WPI_TalonFX rightRearDriveMotor = new WPI_TalonFX(RobotPorts.kRightRearMotor);

   // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(leftFrontDriveMotor, leftRearDriveMotor);

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(rightFrontDriveMotor,rightRearDriveMotor);

  final TalonFXInvertType kInvertType = TalonFXInvertType.Clockwise;


  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The Gyro
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Shifter Solenoid - NOT USED IN 2022
  //private final Solenoid SolarNoise = new Solenoid(RobotPorts.kShifterSolenoid);

  public DrivetrainSubsystem() {
    // Sets the distance per pulse for the encoders
    leftFrontDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightFrontDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    //Sets all motors to brake mode
    leftFrontDriveMotor.setNeutralMode(NeutralMode.Brake);
    leftRearDriveMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontDriveMotor.setNeutralMode(NeutralMode.Brake);
    rightRearDriveMotor.setNeutralMode(NeutralMode.Brake);

    m_drive.setSafetyEnabled(false);

    //Invert right motors
    rightRearDriveMotor.setInverted(kInvertType);
    rightFrontDriveMotor.setInverted(kInvertType);

    //Set open loop ramp - ramps to max throttle over 0.75s 
    leftFrontDriveMotor.configOpenloopRamp(0.75,10);
    leftRearDriveMotor.configOpenloopRamp(0.75,10);
    rightFrontDriveMotor.configOpenloopRamp(0.75,10);
    rightFrontDriveMotor.configOpenloopRamp(0.75,10);
  }

  public void cougarDrive(double fwd, double rot, boolean climbMode, boolean turboMode) {
    //System.out.println("Forward: " + fwd);
    double x = Math.pow(fwd,3.0);
  
    double y = Math.pow(rot,3.0);
    //System.out.println("1st x: " + x);
    m_drive.arcadeDrive(Math.max(-0.7,Math.min(0.7,x)),Math.max(-0.6,Math.min(0.6,y)));
    
    if(climbMode == true){
      m_drive.arcadeDrive(Math.max(-0.5,Math.min(0.5,x)),Math.max(-0.6,Math.min(0.6,y)));
    }
    else if(turboMode == true){
      m_drive.arcadeDrive(Math.max(-0.9,Math.min(0.9,x)),Math.max(-0.6,Math.min(0.6,y)));
    }
    else{
      m_drive.arcadeDrive(Math.max(-0.7,Math.min(0.7,x)),Math.max(-0.6,Math.min(0.6,y)));
    }

  }
  
  public void AutoDroive(double distance) {

    //find absolute error
    //Encoder distance: positive = backwards, negative = forwards
    double error = Math.abs(distance) - Math.abs(getAverageEncoderDistance());
    System.out.println("AutoDrive distance: " + distance +" Error:" + error);

    //if distance is positive and error is greater than 

    if(error > DriveConstants.kAutoDistanceError){   
      
      //Positive 1st argument = backwards
      cougarDrive(-Math.signum(distance)*DriveConstants.kAutoHighSpeedRatio, -m_gyro.getAngle()*DriveConstants.kAutoTurnRatio,false,false);
      System.out.println("Auto High Speed Ratio: " + DriveConstants.kAutoHighSpeedRatio);
    }  
    else if(error <= DriveConstants.kAutoDistanceError){  
      cougarDrive(-Math.signum(distance)*DriveConstants.kAutoLowSpeedRatio, -m_gyro.getAngle()*DriveConstants.kAutoTurnRatio,false,false);
      System.out.println("Near target - slow down:" + DriveConstants.kAutoLowSpeedRatio);
    }
    else {
      System.out.println("Target reached/passed - Stop");
      cougarDrive(0,0,false,false);
    }
  }
  
  public void TurnControl(double AngleTarget){
    double angle = m_gyro.getAngle();
    double target = angle + AngleTarget;

    if(Math.abs(target) > DriveConstants.kAutoAngleError){
      cougarDrive(0, -Math.signum(target)*(Math.abs(target)*DriveConstants.kAutoTurnRatio) + DriveConstants.kAutoMinRotRatio,false,false);
    }
    else {
      cougarDrive(0, 0, false,false);
    }
  }

  public void resetEncoders() {
    leftFrontDriveMotor.setSelectedSensorPosition(0);
    rightFrontDriveMotor.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance() {
    double distancePerUnit=6*Math.PI/2048/DriveConstants.kDrivetrainRatio;
    return (getLeftEncoder()+getRightEncoder())*distancePerUnit / 2.0;
  }

  public double getLeftEncoder() {
    return leftFrontDriveMotor.getSelectedSensorPosition();
  }

  public double getRightEncoder() {
    return rightFrontDriveMotor.getSelectedSensorPosition();
  }

  public double getAngle() {
    return m_gyro.getAngle();
  }

  public void resetAngle(){
    m_gyro.reset();
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }


  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Distance Traveled",getAverageEncoderDistance());
    //SmartDashboard.putNumber("Angle Heading",getAngle());
    SmartDashboard.putNumber("Energy", PowerDistributionPanel.getTotalEnergy());
    //This method will be called once per scheduler run
  }

}