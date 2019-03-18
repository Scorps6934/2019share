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

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MoveElevator;

/**
 * Add your docs here.
 */
public class S_Elevator extends PIDSubsystem {
  TalonSRX motor = new TalonSRX(RobotMap.elevatorTalonPort);

  public S_Elevator() {
    super("S_Elevator", RobotMap.Parm, RobotMap.Iarm, RobotMap.Darm);
    setAbsoluteTolerance(0.05);
    getPIDController().setContinuous(false);

    /* top PID */
    motor.config_kP(RobotMap.elevatorTalonPort, RobotMap.Pelevator, RobotMap.kTimeoutMs);
		motor.config_kI(RobotMap.elevatorTalonPort, RobotMap.Ielevator, RobotMap.kTimeoutMs);
    motor.config_kD(RobotMap.elevatorTalonPort, RobotMap.Delevator, RobotMap.kTimeoutMs);	
  //motor.config_kI(RobotMap.elevatorTalonPort, RobotMap.Pelevator, 10);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new MoveElevator());
  }

  public void moveElevator(double speed){
    motor.set(ControlMode.PercentOutput, speed);
  }
  public void stopElevator(){
    //TODO:PID loop
  }

  //encoder stuff
  public void configElevatorEncoders(){
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }
  
  public void zeroElevatorEncoders(){
    motor.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
  }

  public int getElevatorEncoderUnits(){
    return motor.getSelectedSensorPosition();
  }

  public double returnPIDInput() {
    return motor.getSelectedSensorPosition(); //returns the sensor value that is providing the feedback for the system
}

  public void usePIDOutput(double output) {
      motor.set(ControlMode.Position, output); // this is where the computed output value fromthe PIDController is applied to the motor
}
}
