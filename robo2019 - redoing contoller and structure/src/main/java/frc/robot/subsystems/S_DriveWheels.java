/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MoveWheels;

/**
 * Add your docs here.
 */
public class S_DriveWheels extends Subsystem {
  TalonSRX lfMoto = new TalonSRX(RobotMap.lfDrive); // left front
  TalonSRX lbMoto = new TalonSRX(RobotMap.lbDrive); // left back
  TalonSRX rfMoto = new TalonSRX(RobotMap.rfDrive); // right front
  TalonSRX rbMoto = new TalonSRX(RobotMap.rbDrive); // right back

  //gyro??

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MoveWheels());
  }

  public void adjustSpeed(double leftInput, double rightInput){
    lfMoto.set(ControlMode.PercentOutput, leftInput);
    lbMoto.set(ControlMode.PercentOutput, leftInput);
    rfMoto.set(ControlMode.PercentOutput, rightInput);
    rbMoto.set(ControlMode.PercentOutput, rightInput);

  }

  public void stopWheels(){
    lfMoto.set(ControlMode.PercentOutput, 0);
    lbMoto.set(ControlMode.PercentOutput, 0);
    rfMoto.set(ControlMode.PercentOutput, 0);
    rbMoto.set(ControlMode.PercentOutput, 0);
  }

  public void configDriveEncoders(){
    lfMoto.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    lbMoto.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rfMoto.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rbMoto.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    //    System.out.println(lfMoto.getSelectedSensorPosition());
  }
  public void zeroDriveEncoders()
  {
    lfMoto.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
    lbMoto.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
    rfMoto.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
    rbMoto.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
  }
//TODO: get encoder values from other side maybe? and/or average the two sides?
  public int getDriveEncoderUnits(){
    return lfMoto.getSelectedSensorPosition();
  }


}
