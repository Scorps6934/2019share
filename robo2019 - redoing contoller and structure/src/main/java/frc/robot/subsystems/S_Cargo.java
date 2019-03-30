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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class S_Cargo extends Subsystem {
  DigitalInput limitswitch = new DigitalInput(RobotMap.cargoLimitSwitch);
 // private TalonSRX motor = new TalonSRX();
  WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.cargoTalonPort);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setShooterSpeed(double speed){
  //  System.out.println(speed);
    motor.set(ControlMode.PercentOutput, speed);
  /*  if (!limitswitch.get()){ //TODO: make sure limitswitch in correct position
   // System.out.println("shooter");
      motor.set(ControlMode.PercentOutput, speed);
    }
    else {
      motor.set(ControlMode.PercentOutput, 0);
    }
*/
  }
  public void stopShooter(){
    motor.set(ControlMode.PercentOutput, 0);
  }

  public void resetCargoTalon(){
    motor.configFactoryDefault();
  }
}
