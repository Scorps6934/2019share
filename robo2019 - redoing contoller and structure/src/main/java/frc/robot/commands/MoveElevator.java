/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;

public class MoveElevator extends CommandBase {
  private Integer elevatorHeight;

  public MoveElevator() {
    super("MoveElevator - teleop");
    requires(Robot.selevator);
  }
  public MoveElevator(int height) {
    super("MoveElevator - auto");
    requires(Robot.selevator);
    this.elevatorHeight = height + Robot.selevator.getElevatorEncoderUnits();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.selevator.moveElevator(Robot.oi.weaponsController.getRawAxis(RobotMap.gcLeftAxisY));
    
    /*  if(Robot.sarm.getArmEncoderUnits() > RobotMap.minArmForwardEncoderCount){ // making sure arm is in a place where we can move elevator
      if (this.elevatorHeight == null){
        int setpoint = Robot.selevator.getElevatorEncoderUnits() + (int)(Robot.oi.weaponsController.getRawAxis(RobotMap.gcLeftAxisY)*RobotMap.elevatorJoystickStep);
        setpoint = Math.min(setpoint, RobotMap.elevatorUpperLimit);
        setpoint = Math.max(setpoint, RobotMap.elevatorLowerLimit);
        Robot.selevator.setElevatorHeight(setpoint);
      }
      else{
        Robot.selevator.setElevatorHeight(this.elevatorHeight);
      }
    }
*/
    
    
  }
  //Robot.sdrive.adjustSpeed(Robot.oi.driveController.getRawAxis(RobotMap.leftAxisY), Robot.oi.driveController.getRawAxis(RobotMap.rightAxisY));

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.selevator.getCurrentError()<RobotMap.elevatorAllowableError;
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
