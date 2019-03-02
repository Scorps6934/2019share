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

public class MoveWheels extends CommandBase {
  private Double speed = null;
  private Double dist = null;

  public MoveWheels() {
    requires(Robot.sdrive);
  }
  public MoveWheels(double speed){
    requires(Robot.sdrive);
    this.speed = speed;
  }
  public MoveWheels(double speed, double dist){
    this.speed = speed;
    this.dist = dist;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("starts");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // System.out.println(Robot.oi.stick.getRawAxis(RobotMap.leftAxisY));
    if (dist != null && speed != null){
      while(Math.abs(Robot.sdrive.getDriveEncoderUnits()) < dist){
        Robot.sdrive.adjustSpeed(speed, 0);
      }
      Robot.sdrive.adjustSpeed(0, 0);
    }
    else if (speed != null){
      Robot.sdrive.adjustSpeed(speed, 0);
    }
    else{
      Robot.sdrive.adjustSpeed(Robot.oi.stick.getRawAxis(RobotMap.leftAxisY), Robot.oi.stick.getRawAxis(RobotMap.rightAxisY));
    }
  }


  protected void execute(double speed) {
    // System.out.println(Robot.oi.stick.getRawAxis(RobotMap.leftAxisY));
    Robot.sdrive.adjustSpeed(speed, 0);
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
