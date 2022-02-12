/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class RobotPorts{

        //CAN IDs
        public static final int kLeftFrontMotor = 1;
        public static final int kLeftRearMotor = 3;
        public static final int kRightFrontMotor = 0;
        public static final int kRightRearMotor = 2;
        public static final int kIntakeMotor = 4;
        public static final int kConveyorMotor = 5;
        public static final int kLeftClimberMotor = 6;
        public static final int kRightClimberMotor = 7;
        public static final int kShooterMotor = 10;

        //Pneumatic Control Module ports
        public static final int kShifterSolenoid = 3;
        public static final int kIntakeSolenoid = 7;



        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};


    }
    public static final class DriveConstants {
        //Drivetrain Parameters
        public static final double kLowSpeedRatio = 0.7;
        public static final double kHighSpeedRatio = 0.7;
        public static final double kLimitTurnRatio = 0.5;

        public static final double kAutoSpeedRatio = 0.5;           //Adjusts how quickly we drive straight in auto
        public static final double kAutoDistanceError = 20;         //Threshold acceptable distance error for auto
        public static final double kAutoMinFwdRatio = 0;            //Minimum power required to move robot

        public static final double kAutoTurnRatio = 0.03;           //Adjusts how quickly we turn in auto         
        public static final double kAutoAngleError = 1;             //Threshold acceptable angle error for auto
        public static final double kAutoMinRotRatio = .07;          //Minimum power required to turn robot
        
        //Vision Paramters
        public static final double kVisionSpeedRatio = 0.2;
        public static final double kVisionTurnRatio = 0.7;

        //Drivetrain Motors

    
        //Encoders
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
    
        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

        //Gyro
    }
    //intake
    public static final class IntakeConstants {
        public static final double kIntakeSpeed = 0.75; // works with bumper
        public static final double kConveyorSpeed = 0.75;
    }

    public static final class ShooterConstants{
        
        public static final double kShooter_LineRPM = 3750; //Initiation
        public static final double kShooter_17RPM = 5000; //Trench
        public static final double kShooter_passRPM = 500; //pass

        public static final int kMotorPort = 5;
        public static final int kTimeoutMs = 30;
    	public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        
        public static final double kF = 0.07;
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0.001;
}
    //Climber motors

    public static final class ClimberConstants{

        public static final double kClimberPwrUp = 0.3;
        public static final double kClimberPwrDown = -0.3;

    }


}
