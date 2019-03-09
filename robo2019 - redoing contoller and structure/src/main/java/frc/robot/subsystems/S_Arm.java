/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class S_Arm extends Subsystem {
  TalonSRX motor = new TalonSRX(RobotMap.armTalonPort);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void moveArm(double speed){
    motor.set(ControlMode.PercentOutput, speed);
  }
  public void stopArm(){
    motor.set(ControlMode.PercentOutput, 0);
  }

//encoders
  public void configArmEncoders(){
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  public void zeroArmEncoders(){
    motor.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
  }

  public int getArmEncoderUnits(){
    return motor.getSelectedSensorPosition();
  }
}
