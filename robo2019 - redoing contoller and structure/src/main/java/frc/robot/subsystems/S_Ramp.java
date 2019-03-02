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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Counter;

/**
 * Add your docs here.
 */
public class S_Ramp extends Subsystem {
  DigitalInput limitswitch = new DigitalInput(RobotMap.rampLimitSwitch);
  Counter counter = new Counter(limitswitch);
  TalonSRX leftMotor = new TalonSRX(RobotMap.rampTalonPort1);
  TalonSRX rightMotor = new TalonSRX(RobotMap.rampTalonPort2);

  @Override
   
  public void initDefaultCommand() {
    
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  
  
  public void resetCounter(){
    counter.reset();
  } 

  //sees if limit switch has been hit
  public boolean isSwitchSet(){
    return counter.get() > 0;
  }
  

  public void moveMotor(double speed){
     // limit switch on always voltage aka top on limit switch
     leftMotor.set(ControlMode.PercentOutput, speed);
     rightMotor.set(ControlMode.PercentOutput, speed);
    
  }

  public void stopMotor() {
    leftMotor.set(ControlMode.PercentOutput, 0);
    rightMotor.set(ControlMode.PercentOutput, 0);
  }
  public void configRampEncoders(){
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  public void zeroRampEncoders(){
    leftMotor.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
    rightMotor.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
  }

  //TODO: get encoder values from other side maybe? or average the two sides?
  public int getRampEncoderUnits(){
    return leftMotor.getSelectedSensorPosition();
  }
}
