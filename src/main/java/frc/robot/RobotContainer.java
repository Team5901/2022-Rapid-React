/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cscore.UsbCamera;

import java.util.ResourceBundle.Control;

import edu.wpi.first.cameraserver.CameraServer;
/*

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;
*/
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ActivateIntake;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.LoadCargoIn;
import frc.robot.commands.ReverseLoader;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.AutoDrive;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_DrivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final SendableChooser<Command> auto = new SendableChooser<Command>();


  public XboxController Controller1 = new XboxController(0);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //Add default commands here
    m_DrivetrainSubsystem.setDefaultCommand(new RunCommand(() -> m_DrivetrainSubsystem.cougarDrive(
            Controller1.getLeftY(),
            -Controller1.getRightX(),Controller1.getXButton(),Controller1.getLeftStickButton()), m_DrivetrainSubsystem));
              
    // Configure the button bindings
    configureButtonBindings();

    //private HttpCamera limelightFeed;
    //limelightFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
    //driverShuffleboardTab.add("LL", limelightFeed).withPosition(0, 0).withSize(15, 8).withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

    //Autonomous procedures

    auto.setDefaultOption("Shoot and reverse", new ShootHigh(m_ShooterSubsystem,m_IntakeSubsystem).withTimeout(3) 
    .andThen(new AutoDrive(-150.0, m_DrivetrainSubsystem)));
    auto.addOption("Reverse", new AutoDrive(-100.0, m_DrivetrainSubsystem));
    SmartDashboard.putData("Auto Chooser", auto);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /**##################################
     * ##### CONTROLLER 1 - PRIMARY #####
     * ##################################*/

    
    new JoystickButton(Controller1, Button.kRightBumper.value)
    .whenHeld(new ActivateIntake(m_IntakeSubsystem));

    new JoystickButton(Controller1, Button.kLeftBumper.value)
    .whenHeld(new ReverseLoader(m_IntakeSubsystem));

    new JoystickButton(Controller1, Button.kA.value)
    .whenHeld(new IntakeOut(m_IntakeSubsystem));

    new JoystickButton(Controller1, Button.kY.value)
    .whenHeld(new LoadCargoIn(m_IntakeSubsystem));

    new JoystickButton(Controller1, Button.kB.value)
    .whenHeld(new ShootHigh(m_ShooterSubsystem,m_IntakeSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    System.out.println("RUNNING AUTONOMOUS COMMAND: " + auto.getSelected());
    return auto.getSelected();

  }
}
