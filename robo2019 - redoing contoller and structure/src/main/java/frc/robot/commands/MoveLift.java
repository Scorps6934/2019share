/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Command;

public class MoveLift extends CommandBase {
  private double dir;
  public MoveLift(double dir) {
    requires(slift);
    this.dir = dir;
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    slift.resetCounter();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  //EZ CLAP GG 4 STOCK CLUTCHBOX
  protected void execute() {
    // going up
    if (slift.isSwitchSet() && dir > 0){
      slift.stopMotor();//Motor 0
      //OverrideGoesHere -switch (Justin Sucks)
    } 
    //Going Down
    else if (slift.isSwitchSet() && dir < 0){
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
  }
}
