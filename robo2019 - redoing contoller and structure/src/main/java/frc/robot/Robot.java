/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.MoveRamp;
import frc.robot.commands.MoveWheels;
import frc.robot.subsystems.S_Arm;
import frc.robot.subsystems.S_Cargo;
import frc.robot.subsystems.S_DriveWheels;
import frc.robot.subsystems.S_Elevator;
import frc.robot.subsystems.S_Hatch;
import frc.robot.subsystems.S_Ramp;

import java.util.ArrayList;
import java.util.List;

//import com.sun.tools.jdeps.Main;
import org.opencv.core.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import frc.robot.Vision;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /////////////////////////////////////////////////////// AHRS gyro = new
  /////////////////////////////////////////////////////// AHRS(Port.kUSB);
  public static OI oi;

  Compressor compressor = new Compressor(RobotMap.compressorPort);
  
  // subsystems
  public static S_Arm sarm = new S_Arm();
  public static S_Cargo scargo = new S_Cargo();
  public static S_DriveWheels sdrive = new S_DriveWheels();
  public static S_Elevator selevator = new S_Elevator();
  public static S_Hatch shatch = new S_Hatch();
  public static S_Ramp sramp = new S_Ramp();

  private static AnalogInput distanceSensor;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser;

  public static Vision leftVisionProcessor = new Vision("camera1");
  public static Vision rightVisionProcessor = new Vision("camera2");

  // MoveWheels wheels;
  // WPI_TalonSRX testMotor;

  // public MoveWheel mWheel;
  // UsbCamera leftCamera;
  // UsbCamera rightCamera;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    System.out.println("robo initiation cerimony");
    oi = new OI();
    m_chooser = new SendableChooser<>();

    compressor.setClosedLoopControl(true);

    distanceSensor = new AnalogInput(RobotMap.distanceSensorPort);

    sdrive.gyro.reset();

/* TODO: put this back
  // encoder set-up
    scargo.configCargoEncoders();
    sdrive.configDriveEncoders();
    selevator.configElevatorEncoders();
    // no hatch encoder
    sramp.configRampEncoders();

    scargo.zeroCargoEncoders();
    sdrive.zeroDriveEncoders();
    selevator.zeroElevatorEncoders();
    // no hatch encoders
    sramp.zeroRampEncoders();

*/


  // openCv and vision stuff
    new Thread(() -> {
      leftVisionProcessor.setupCameraSettings();
      leftVisionProcessor.setupThresholding();
      leftVisionProcessor.setupContours();
      leftVisionProcessor.filterAndDrawContours(); //TODO: might need while loop?
    }).start();

    new Thread(() -> {
      rightVisionProcessor.setupCameraSettings();
      rightVisionProcessor.setupThresholding();
      rightVisionProcessor.setupContours();
      rightVisionProcessor.filterAndDrawContours(); //TODO: might need while loop?
    }).start();


//    CameraServer.getInstance().startAutomaticCapture();
//    CameraServer.getInstance().startAutomaticCapture();
  //  leftCamera = new UsbCamera("Left Camera", 1);
  //  rightCamera = new UsbCamera("Right Camera", 2);


  //wheels = new MoveWheels();
    

    // chooser.addObject("My Auto", new MyAutoCommand());
    //SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
//Code used to reset gyro everytime code is deployed
   // gyro.reset();
  //testMotor.set(0.1);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }


  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    System.out.println("hiiiiiiiii");
    //System.out.println("Value: " + ((distanceSensor.getVoltage()*1024.0)/25.4)); // volatage to inches
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
