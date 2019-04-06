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
  /*notes:
  * compressorPort/pcmPort should be the same thing (code identifies compressor the can id of the pcm due to their being only one compressor port on a pcm)
  * for each solenoid forward and reverse are condsidered ports on the pcm (eg 2 solenoids should have a total of 4 ports used)
  */
  public static final int compressorPort = 0;
  public static final int pcmPort = 0;
  //hatch
  public static final int hatchSolenoidForward = 4;
  public static final int hatchSolenoidReverse = 7;


//controllers
  public static final int logitechDriveCont = 0;
  public static final int gamecubeWeaponsCont = 1;
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

  //gamecube buttons
  public static final int gcButtonA = 2;
  public static final int gcButtonB = 3;
  public static final int gcButtonX = 1;
  public static final int gcButtonY = 4;
  public static final int gcRightBumper = 8; // z button
  public static final int gcStartButton = 10;
  //dpad buttons
  public static final int gcUp = 13;
  public static final int gcDown = 15;
  public static final int gcLeft = 16;
  public static final int gcRight = 14;
  //joystick axes
  public static final int gcLeftAxisX = 0; // control stick
  public static final int gcLeftAxisY = 1;
  public static final int gcRightAxisX = 5; // c stick
  public static final int gcRightAxisY = 2;
  //triggers analog (probs not used) -- ignore
  public static final int gcLeftTrigger = 3; 
  public static final int gcRightTrigger = 4;
  //trigger digital (buttons)
  public static final int gcLeftTriggerButton = 5;
  public static final int gcRightTriggerButton = 6;



//cool sensor stuff :>
  public static final int distanceSensorPort = 0;
  public static final int kTimeoutMs = 30; // TODO: make sure all used of timeout are appropriate for 30ms

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
  public static final int lfDrive = 3;
  public static final int lbDrive = 2;
  public static final int rfDrive = 15;
  public static final int rbDrive = 14;

  //pump/climb
  public static final int rightClimberTalonPort = 13;
  public static final int leftClimberTalonPort = 12;
  public static final int pump = 1;

 // armTalonPort = 0;


//PID
  //pid slots
  public static final int driveAnglePID = 0;
  public static final int driveDistancePID = 1;


  //Drive (p, i, and d constants and allowable error all per PID port)
  public static final int PdriveAngle = 0;
  public static final int IdriveAngle = 0;
  public static final int DdriveAngle = 0;
  public static final int PdriveDistance = 0;
  public static final int IdriveDistance = 0;
  public static final int DdriveDistance = 0;
  public static final int driveDistanceAllowableError = 42069; //TODO: set these correctly
  public static final int driveAngleAllowableError = 2;
  
  public static final int driveCruiseVelocity = 1;
  public static final int driveAcceleration = 1;


  public static final int minArmForwardEncoderCount = 42069; // when arm is past this point it is considered forward


  public static final double EncoderUnitsToDistanceRatio = 42069.0;
}
