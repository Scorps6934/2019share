/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


public class MoveLift extends CommandBase {
  private double dir;

  public MoveLift(double dir) {
    super();
    System.out.println("constructor");
    requires(slift);
    this.dir = dir;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("initialize");
    slift.resetCounter(); // line may break limit switch purpose if spammed
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    System.out.println("execute");

    // logic works if limitswitch is normally open
    if (slift.isSwitchSet() && dir > 0) { 
      slift.stopMotor();
    }
    else if (slift.isSwitchSet() && dir < 0) {
      slift.moveMotor(dir);
      slift.resetCounter();
    }
    else {
      slift.moveMotor(dir);
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
    slift.stopMotor();
  }

}

// justin sucks v2
//electric boogaloo

