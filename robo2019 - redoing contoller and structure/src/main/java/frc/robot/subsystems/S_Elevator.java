/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MoveElevator;

/**
 * Add your docs here.
 */
public class S_Elevator extends Subsystem {
  TalonSRX motor = new TalonSRX(RobotMap.elevatorTalonPort);
/*
  public S_Elevator() {
    super("S_Elevator", RobotMap.Parm, RobotMap.Iarm, RobotMap.Darm);
    setAbsoluteTolerance(0.05);
    getPIDController().setContinuous(false);

    // top PID 
    motor.config_kP(RobotMap.elevatorTalonPort, RobotMap.Pelevator, RobotMap.kTimeoutMs);
		motor.config_kI(RobotMap.elevatorTalonPort, RobotMap.Ielevator, RobotMap.kTimeoutMs);
    motor.config_kD(RobotMap.elevatorTalonPort, RobotMap.Delevator, RobotMap.kTimeoutMs);	
  //motor.config_kI(RobotMap.elevatorTalonPort, RobotMap.Pelevator, 10);
  }
*/

  public S_Elevator() {
//    motor = TalonSRXFactory.createDefaultTalon(Constants.kElevatorMasterId); maybe?

    
            motor.configSelectedFeedbackSensor(
                    FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
          

    
            motor.configForwardLimitSwitchSource( //TODO: make sure limitswitches aren't breaking thing or just understand this
                    LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
                    RobotMap.kTimeoutMs);
          

    
            motor.configForwardSoftLimitThreshold(
                    RobotMap.elevatorUpperLimit, RobotMap.kTimeoutMs);


    
            motor.configForwardSoftLimitEnable(true, RobotMap.kTimeoutMs);
      

    
            motor.configVoltageCompSaturation(11.0, RobotMap.kTimeoutMs); // 254 put at 12.0 ¯\_(ツ)_/¯
     

    
            motor.configReverseSoftLimitThreshold(
                    RobotMap.elevatorLowerLimit, RobotMap.kTimeoutMs);
      

    
            motor.configReverseSoftLimitEnable(
                    true, RobotMap.kTimeoutMs);
      

    //configure magic motion
    
            motor.config_kP(
                    RobotMap.elevatorPID, RobotMap.Pelevator, RobotMap.kTimeoutMs);
    

    
            motor.config_kI(
                    RobotMap.elevatorPID, RobotMap.Ielevator , RobotMap.kTimeoutMs);


    
            motor.config_kD(
                    RobotMap.elevatorPID, RobotMap.Delevator, RobotMap.kTimeoutMs);
   

    
//            motor.config_kF(
//                    RobotMap.elevatorPID, Constants.kElevatorHighGearKf, RobotMap.kTimeoutMs);
      

    
//            motor.configMaxIntegralAccumulator(
//                   RobotMap.elevatorPID, Constants.kElevatorHighGearMaxIntegralAccumulator, RobotMap.kTimeoutMs);
        

    
//            motor.config_IntegralZone(
//                    RobotMap.elevatorPID, Constants.kElevatorHighGearIZone, RobotMap.kTimeoutMs);
         

    
            motor.configAllowableClosedloopError(
                    RobotMap.elevatorPID, RobotMap.elevatorAllowableError, RobotMap.kTimeoutMs);

    
            motor.configMotionAcceleration(
                    RobotMap.elevatorAcceleration, RobotMap.kTimeoutMs);

    
            motor.configMotionCruiseVelocity(
                    RobotMap.elevatorCruiseVelocity, RobotMap.kTimeoutMs);
/*
    //configure position PID
    
            motor.config_kP(
                    kPositionControlSlot, Constants.kElevatorJogKp, RobotMap.kTimeoutMs);

    
            motor.config_kI(
                    kPositionControlSlot, Constants.kElevatorHighGearKi, RobotMap.kTimeoutMs);

    
            motor.config_kD(
                    kPositionControlSlot, Constants.kElevatorJogKd, RobotMap.kTimeoutMs);

    
            motor.configMaxIntegralAccumulator(
                    kPositionControlSlot, Constants.kElevatorHighGearMaxIntegralAccumulator, RobotMap.kTimeoutMs);

    
            motor.config_IntegralZone(
                    kPositionControlSlot, Constants.kElevatorHighGearIZone, RobotMap.kTimeoutMs);

    
            motor.configAllowableClosedloopError(
                    kPositionControlSlot, Constants.kElevatorHighGearDeadband, RobotMap.kTimeoutMs);
*/

 /*   TODO: add this?
            motor.configClosedloopRamp(
                    Constants.kElevatorRampRate, RobotMap.kTimeoutMs);

    
            motor.configOpenloopRamp(
                    Constants.kElevatorRampRate, RobotMap.kTimeoutMs);

  */  

  //TODO: may need to adjust values?
            motor.configContinuousCurrentLimit(20, RobotMap.kTimeoutMs);

    
            motor.configPeakCurrentLimit(35, RobotMap.kTimeoutMs);

    
            motor.configPeakCurrentDuration(200, RobotMap.kTimeoutMs);

    motor.enableCurrentLimit(true);


    motor.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
    motor.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);


    motor.selectProfileSlot(RobotMap.elevatorPID, 0);

    motor.overrideLimitSwitchesEnable(true); //TODO: wth
    motor.overrideSoftLimitsEnable(false);

    motor.enableVoltageCompensation(true);

    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);

    motor.setSensorPhase(true);

    motor.set(ControlMode.PercentOutput, 0);

    //TODO: do talonObject.setInverted(true) with correct motor
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new MoveElevator());
  }

  public void setElevatorHeight(double setpoint){
    motor.set(ControlMode.MotionMagic, setpoint);
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
/*
  public double returnPIDInput() {
    return motor.getSelectedSensorPosition(); //returns the sensor value that is providing the feedback for the system
}

  public void usePIDOutput(double output) {
      motor.set(ControlMode.Position, output); // this is where the computed output value fromthe PIDController is applied to the motor
}
*/




}
