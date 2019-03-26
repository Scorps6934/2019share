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

/**
 * Add your docs here.
 */
public class S_Arm extends Subsystem {
  TalonSRX motor = new TalonSRX(RobotMap.armTalonPort);
/*
  public S_Arm(){
    super("S_Arm", RobotMap.Parm, RobotMap.Iarm, RobotMap.Darm);
    setAbsoluteTolerance(0.05);
    getPIDController().setContinuous(false);

    // top PID
    motor.config_kP(RobotMap.armTalonPort, RobotMap.Parm, RobotMap.kTimeoutMs);
		motor.config_kI(RobotMap.armTalonPort, RobotMap.Iarm, RobotMap.kTimeoutMs);
    motor.config_kD(RobotMap.armTalonPort, RobotMap.Darm, RobotMap.kTimeoutMs);	
  //motor.config_kI(RobotMap.armTalonPort, RobotMap.Parm, 10);
  } 
*/
  public S_Arm(){
    //    motor = TalonSRXFactory.createDefaultTalon(Constants.karmMasterId); maybe?

    
    motor.configSelectedFeedbackSensor(
      FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);



    motor.configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
          RobotMap.kTimeoutMs);



    motor.configForwardSoftLimitThreshold(
          RobotMap.armForwardLimit, RobotMap.kTimeoutMs);



    motor.configForwardSoftLimitEnable(true, RobotMap.kTimeoutMs);



    motor.configVoltageCompSaturation(11.0, RobotMap.kTimeoutMs); // 254 put at 12.0 ¯\_(ツ)_/¯



    motor.configReverseSoftLimitThreshold(
          RobotMap.armBackwardLimit, RobotMap.kTimeoutMs);



    motor.configReverseSoftLimitEnable(
          true, RobotMap.kTimeoutMs);


    //configure magic motion

    motor.config_kP(
          RobotMap.armPID, RobotMap.Parm, RobotMap.kTimeoutMs);



    motor.config_kI(
          RobotMap.armPID, RobotMap.Iarm , RobotMap.kTimeoutMs);



    motor.config_kD(
          RobotMap.armPID, RobotMap.Darm, RobotMap.kTimeoutMs);



    //            motor.config_kF(
    //                    RobotMap.armPID, Constants.karmHighGearKf, RobotMap.kTimeoutMs);



    //            motor.configMaxIntegralAccumulator(
    //                   RobotMap.armPID, Constants.karmHighGearMaxIntegralAccumulator, RobotMap.kTimeoutMs);



    //            motor.config_IntegralZone(
    //                    RobotMap.armPID, Constants.karmHighGearIZone, RobotMap.kTimeoutMs);



    motor.configAllowableClosedloopError(
          RobotMap.armPID, RobotMap.armAllowableError, RobotMap.kTimeoutMs);


    motor.configMotionAcceleration(
          RobotMap.armAcceleration, RobotMap.kTimeoutMs);


    motor.configMotionCruiseVelocity(
          RobotMap.armCruiseVelocity, RobotMap.kTimeoutMs);
    /*
    //configure position PID

    motor.config_kP(
          kPositionControlSlot, Constants.karmJogKp, RobotMap.kTimeoutMs);


    motor.config_kI(
          kPositionControlSlot, Constants.karmHighGearKi, RobotMap.kTimeoutMs);


    motor.config_kD(
          kPositionControlSlot, Constants.karmJogKd, RobotMap.kTimeoutMs);


    motor.configMaxIntegralAccumulator(
          kPositionControlSlot, Constants.karmHighGearMaxIntegralAccumulator, RobotMap.kTimeoutMs);


    motor.config_IntegralZone(
          kPositionControlSlot, Constants.karmHighGearIZone, RobotMap.kTimeoutMs);


    motor.configAllowableClosedloopError(
          kPositionControlSlot, Constants.karmHighGearDeadband, RobotMap.kTimeoutMs);
    */

    /*   TODO: add this?
    motor.configClosedloopRamp(
          Constants.karmRampRate, RobotMap.kTimeoutMs);


    motor.configOpenloopRamp(
          Constants.karmRampRate, RobotMap.kTimeoutMs);

    */  

    //TODO: may need to adjust values?
    motor.configContinuousCurrentLimit(20, RobotMap.kTimeoutMs);


    motor.configPeakCurrentLimit(35, RobotMap.kTimeoutMs);


    motor.configPeakCurrentDuration(200, RobotMap.kTimeoutMs);

    motor.enableCurrentLimit(true);


    motor.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
    motor.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);


    motor.selectProfileSlot(RobotMap.armPID, 0);

    motor.overrideLimitSwitchesEnable(true);
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
  }

  public void setArmPosition(double setpoint){
    motor.set(ControlMode.MotionMagic, setpoint);
  }
 /* 
  public void moveArm(double speed){
    motor.set(ControlMode.PercentOutput, speed);
  }
*/
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

  public double returnPIDInput() {
    return motor.getSelectedSensorPosition(); //returns the sensor value that is providing the feedback for the system
  }

  public void usePIDOutput(double output) {
      motor.set(ControlMode.Position, output); // this is where the computed output value fromthe PIDController is applied to the motor
  }
}
