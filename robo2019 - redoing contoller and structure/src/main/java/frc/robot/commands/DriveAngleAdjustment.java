/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveAngleAdjustment extends CommandBase {
  private double angle;
  public DriveAngleAdjustment(double angle) {
    super("DriveAngleAdjustment");
    requires(Robot.sdrive);
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.sdrive.setDriveAngle(this.angle);
  }

  // Make this return true when this Command no longer needs to run execute()
  //TODO: set isFinished to work properly with scheduler (when will it return true?) also change allowable error?
  @Override
  protected boolean isFinished() {
    return Math.abs(this.angle-Robot.sdrive.getCurrentAngle())<0.1;
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
