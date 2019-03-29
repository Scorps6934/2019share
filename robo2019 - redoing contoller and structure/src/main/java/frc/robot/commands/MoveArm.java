/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;

public class MoveArm extends CommandBase {
  private Integer armPosition;

  public MoveArm() {
    super("MoveArm - teleop");
    requires(Robot.sarm);
  }
  public MoveArm(int position){
    super("MoveArm - auto");
    requires(Robot.sarm);
    this.armPosition = position + Robot.sarm.getArmEncoderUnits(); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.sarm.moveArm(Robot.oi.weaponsController.getRawAxis(RobotMap.gcRightAxisY));
  /*  if(this.armPosition == null){
     // System.out.println("sees joystick input - enc: "+ Robot.sarm.getArmEncoderUnits());
     double joystickInput = Robot.oi.weaponsController.getRawAxis(RobotMap.gcRightAxisY);
      if (Math.abs(joystickInput) < .15){
        joystickInput = 0;
      }
        int setpoint = Robot.sarm.getArmEncoderUnits() + (int)(joystickInput*RobotMap.armJoystickStep);

      System.out.println("arm enc pos: "+ Robot.sarm.getArmEncoderUnits());
      System.out.println("setpoint: " +setpoint);
      if (setpoint < RobotMap.armForwardLimit && setpoint > RobotMap.armBackwardLimit){
        Robot.sarm.setArmPosition(setpoint);
      }
    }
    else {
      Robot.sarm.setArmPosition(this.armPosition);
    }
    */
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.sarm.getCurrentError()<RobotMap.armAllowableError;
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
