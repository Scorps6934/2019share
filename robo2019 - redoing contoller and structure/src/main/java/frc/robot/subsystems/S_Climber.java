/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class S_Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  TalonSRX mpump;
  TalonSRX rightClimberMotor; 
  //TalonSRX leftClimberMotor;
    public S_Climber(){
      mpump = new TalonSRX(RobotMap.pump);
      rightClimberMotor = new TalonSRX(RobotMap.rightClimberTalonPort);
    //  leftClimberMotor = new TalonSRX(RobotMap.leftClimberTalonPort);

      mpump.setNeutralMode(NeutralMode.Brake);
    }

    public void setClimbRaw(double power){
      rightClimberMotor.set(ControlMode.PercentOutput, power);
      //leftClimberMotor.set(ControlMode.PercentOutput, -power); // TODO: make sure both go in same direction physically
    }
    public void setPumpRaw(double power){
      mpump.set(ControlMode.PercentOutput, power);
    }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }



}
