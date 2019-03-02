/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  public static final int liftLimitSwitch = 0; // dio port
  public static final int liftTalonPort1 = 0;
  public static final int liftTalonPort2 = 1;

  //controllers
  public static final int logitechCont = 0;

  //left axis
  public static final int leftAxisX = 0;
  public static final int leftAxisY = 1;
  public static final int rightAxisX = 2;
  public static final int rightAxisY = 3;
  

  

  // drive talon ports
  public static final int lfDrive = 0;
  public static final int lbDrive = 2;
  public static final int rfDrive = 1;
  public static final int rbDrive = 3;
  

  



 //cool sensor stuff :>
 public static final int distanceSensorPort = 0;
 public static final int kTimeoutMs = 30;

 //img processing 
 //test later empirically (may not use area in the end)
 public static final int contourMinArea = 1000;
 public static final int contourMaxArea = 100000;
 public static final double contourMinRatio = 0.7;
 public static final double contourMaxRatio = 1.5;



  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
