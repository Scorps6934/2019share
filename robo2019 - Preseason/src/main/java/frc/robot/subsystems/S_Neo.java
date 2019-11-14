/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.RunNeo;

/**
 * Add your docs here.
 */
public class S_Neo extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax motor1;

  public S_Neo(){
    motor1 = new CANSparkMax(RobotMap.NEO1, MotorType.kBrushless);

    //motor1.restoreFactoryDefaults() doesn't work 

  }
  public void runMotor(double percent){
    //System.out.println(percent);
    //System.out.println("?"+ motor1.get());
    motor1.set(percent);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new RunNeo());
  }
}
