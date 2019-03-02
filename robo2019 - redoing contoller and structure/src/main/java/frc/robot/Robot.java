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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.MoveLift;
import frc.robot.commands.MoveWheels;
import frc.robot.subsystems.S_DriveWheels;
import frc.robot.subsystems.S_Lift;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
///////////////////////////////////////////////////////  AHRS gyro = new AHRS(Port.kUSB);
  public static OI oi;
  
  public static S_Lift slift = new S_Lift();
	public static S_DriveWheels sdrive = new S_DriveWheels();

  private static AnalogInput distanceSensor;


  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser;

// MoveWheels wheels;
  //WPI_TalonSRX testMotor;

  //public MoveWheel mWheel;
//    UsbCamera leftCamera;
//    UsbCamera rightCamera;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI();
    m_chooser = new SendableChooser<>();

    distanceSensor = new AnalogInput(RobotMap.distanceSensorPort);

    sdrive.zeroEncoders();

    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(360, 360);
      
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 360, 360);
      CvSource colorStream = CameraServer.getInstance().putVideo("Colorful!", 360, 360);
      Mat source = new Mat();
      Mat output = new Mat();
      
      while(!Thread.interrupted()) {
          cvSink.grabFrame(source, 30);

          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
          Imgproc.threshold(output, output, 0, 255, Imgproc.THRESH_OTSU);
          //Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2RGB);
          List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
          Mat hierarchy = new Mat();
          Imgproc.findContours(output, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
          Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2RGB);
          
          for (int i = 0; i < contours.size(); i++) {
            //...contour code here...
            double contourArea = Imgproc.contourArea(contours.get(i));
            Rect boundRect = Imgproc.boundingRect(contours.get(i));
            double ratio = contourArea/(boundRect.width*boundRect.height); // solidity ratio
          //  double ratio = (double)boundRect.width/boundRect.height; //if not using aspect ratio can move contourArea definition to ration
          //  System.out.println();
            if ((ratio < RobotMap.contourMinRatio || ratio > RobotMap.contourMaxRatio) &&
                  (contourArea < RobotMap.contourMinArea || contourArea > RobotMap.contourMaxArea)){
              continue;
            }
           // SmartDashboard.putNumber("dab", ratio);
           // System.out.println(i+" "+ ratio);
            //Imgproc.drawMarker(output, boundRect.br(), new Scalar(0,0,255));
            //Imgproc.drawMarker(source, boundRect.br(), new Scalar(0,0,255));
            Imgproc.rectangle(output, boundRect.br(),boundRect.tl() , new Scalar(0,0,255), 10);
            Imgproc.rectangle(source, boundRect.br(),boundRect.tl() , new Scalar(0,0,255), 10); 
            Imgproc.drawContours(output, contours, i, new Scalar(255, 0, 0), 10);
            Imgproc.drawContours(source, contours, i, new Scalar(255, 0, 0), 10);
          }
          colorStream.putFrame(source);
          outputStream.putFrame(output);
      }
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
    sdrive.zeroEncoders();
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

    //System.out.println("Value: " + ((distanceSensor.getVoltage()*1024.0)/25.4)); // volatage to inches
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
