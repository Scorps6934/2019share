/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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
  WPI_VictorSPX motor = new WPI_VictorSPX(RobotMap.cargoVictorPort);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setShooterSpeed(double speed){
    System.out.println("before shooter");
    System.out.println();
    if (!limitswitch.get()){ //TODO: make sure limitswitch in correct position
    System.out.println("shooter");
      motor.set(speed*.5);
    }
    else {
      motor.set(0);
    }
  }
  public void stopShooter(){
    motor.set(0);
  }
}
