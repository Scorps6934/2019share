/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MoveWheels;

/**
 * Add your docs here.
 */
public class S_DriveWheels extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonSRX lfMoto = new WPI_TalonSRX(RobotMap.lfDrive); // left front
  WPI_TalonSRX lbMoto = new WPI_TalonSRX(RobotMap.lbDrive); // left back
  WPI_TalonSRX rfMoto = new WPI_TalonSRX(RobotMap.rfDrive); // right front
  WPI_TalonSRX rbMoto = new WPI_TalonSRX(RobotMap.rbDrive); // right back

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MoveWheels());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void adjustSpeed(double leftInput, double rightInput){
    lfMoto.set(leftInput);
    lbMoto.set(leftInput);
    rfMoto.set(rightInput);
    rbMoto.set(rightInput);
  }

}
