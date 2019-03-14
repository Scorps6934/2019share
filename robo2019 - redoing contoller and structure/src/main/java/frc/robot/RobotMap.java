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
  
//pneumatics
  public static final int compressorPort = 0;
  public static final int solenoidForward = 0;
  public static final int solenoidReverse = 1;
  //hatch
  public static final int hatchSolenoid = 0;
  //ramp
  public static final int rampSolenoid = 0;

//controllers
  public static final int logitechDriveCont = 0;
  public static final int logitechWeaponsCont = 1;
//logitech button
  public static final int buttonA = 1;
  public static final int buttonB = 2;
  public static final int buttonX = 3;
  public static final int buttonY = 4;
  public static final int leftBumper = 5;
  public static final int rightBumper = 6;
  public static final int backButton = 7;
  public static final int startButton = 8;
//joystick axes
  public static final int leftAxisX = 0;
  public static final int leftAxisY = 1;
  public static final int rightAxisX = 4;
  public static final int rightAxisY = 5;
//triggers
  public static final int leftTrigger = 2;
  public static final int rightTrigger = 3;
  



//cool sensor stuff :>
  public static final int distanceSensorPort = 0;
  public static final int kTimeoutMs = 30;

//img processing 
  //TODO: test later empirically (may not use area in the end)
  public static final int cameraExposure = 0; //??
  public static final int contourMinArea = 1000;
  public static final int contourMaxArea = 100000;
  public static final double contourMinRatio = 0.7;
  public static final double contourMaxRatio = 1.5;

  public static final double FOV = 61.39;
  public static final double imageWidth=360;
  public static final double imageHeight=360;
  public static final double cameraAngle=45;
  public static final double cameraHeight=20;

//talon ports + limitswitches
  // driveWheels (4 motors)
  public static final int lfDrive = 0;
  public static final int lbDrive = 2;
  public static final int rfDrive = 1;
  public static final int rbDrive = 3;

  // ramp (2 motors 1 limitswitch)
  public static final int rampLimitSwitch = 0; // dio port
  public static final int rampTalonPort1 = 4;
  public static final int rampTalonPort2 = 5;
  
  // elevator (1 motor)
  public static final int elevatorTalonPort = 6;

  // cargo (1 motor)
  public static final int cargoTalonPort = 7;

  // arm (1 motor)
  public static final int armTalonPort = 8;


//PID
  //Arm
  public static final int Parm = 0;
  public static final int Iarm = 0;
  public static final int Darm = 0;

  //Wheels
  public static final int Pwheel = 0;
  public static final int Iwheel = 0;
  public static final int Dwheel = 0;

  //Elevator
  public static final int Pelevator = 0;
  public static final int Ielevator = 0;
  public static final int Delevator = 0;
}
