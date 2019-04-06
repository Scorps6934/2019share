/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveToDistance extends Command {
  private int targetDistanceInEncoderUnits;
  public DriveToDistance(double distanceInInches) {
    super("DriveToDistance");
    requires(Robot.sdrive);
    targetDistanceInEncoderUnits = Robot.sdrive.getDriveEncoderUnits() + (int)(distanceInInches*RobotMap.EncoderUnitsToDistanceRatio);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.sdrive.selectDriveDistancePID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.sdrive.setDriveDistance(targetDistanceInEncoderUnits);
  }

  // Make this return true when this Command no longer needs to run execute() 
  @Override
  protected boolean isFinished() {
    return Robot.sdrive.getCurrentError()<RobotMap.driveDistanceAllowableError;
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
