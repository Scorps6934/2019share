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
public class S_Cargo extends Subsystem {
  //TODO: Change to Victor
  TalonSRX motor = new TalonSRX(RobotMap.cargoVictorPort);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setShooterSpeed(double speed){
    motor.set(ControlMode.PercentOutput, speed);
  }
  public void stopShooter(){
    motor.set(ControlMode.PercentOutput, 0);
  }
  
  //encoder stuff
  public void configCargoEncoders(){
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  public void zeroCargoEncoders(){
    motor.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
  }

  public int getCargoEncoderUnits(){
    return motor.getSelectedSensorPosition();
  }
}
