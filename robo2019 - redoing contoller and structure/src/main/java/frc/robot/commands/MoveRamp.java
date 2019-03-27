/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

public class MoveRamp extends CommandBase { // may become depricated (change to lift and flaps)
  private double dir;

  public MoveRamp(double dir) {
    super("ramp with lift movement direction "+dir);
    //System.out.println("constructor");
    requires(Robot.sramp);
    this.dir = dir;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    //System.out.println("initialize");
    Robot.sramp.resetCounter(); // line may break limit switch purpose if button spammed
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("execute");

    // logic works if limitswitch is normally open
    if (Robot.sramp.isSwitchSet() && dir > 0) { 
      Robot.sramp.stopMotors();
    }
    else if (Robot.sramp.isSwitchSet() && dir < 0) {
      Robot.sramp.moveLift(dir);
      Robot.sramp.resetCounter();
    }
    else {
      Robot.sramp.moveLift(dir);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    // im worried this will happen at other times besides unpressing button if more commands are added to subsystem
    // may want to test?
    Robot.sramp.stopMotors();
  }

}

// justin sucks v2
//electric boogaloo

