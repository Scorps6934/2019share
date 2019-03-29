/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;

public class MoveWheels extends CommandBase {

  public MoveWheels() {
    super("moveWheels with stick");
    requires(Robot.sdrive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("starts");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    Robot.sdrive.adjustSpeed(Robot.oi.driveController.getRawAxis(RobotMap.leftAxisY), Robot.oi.driveController.getRawAxis(RobotMap.rightAxisY));

   // System.out.println(Robot.sdrive.getDriveEncoderUnits());
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
