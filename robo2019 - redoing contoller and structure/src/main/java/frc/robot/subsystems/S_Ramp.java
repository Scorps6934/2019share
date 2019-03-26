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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Counter;

/**
 * Ramp subsystem includes lift motors and pneumatics for flaps
 */
public class S_Ramp extends Subsystem {
//lift  
  DigitalInput limitswitch = new DigitalInput(RobotMap.rampLimitSwitch);
  Counter counter = new Counter(limitswitch);
  TalonSRX leftMotor = new TalonSRX(RobotMap.rampTalonPort1);
  TalonSRX rightMotor = new TalonSRX(RobotMap.rampTalonPort2);
//flaps
 DoubleSolenoid solenoid = new DoubleSolenoid(RobotMap.pcmPort, RobotMap.rampSolenoidForward, RobotMap.rampSolenoidReverse);

 /*
  public S_Ramp(){
    //    leftMotor = TalonSRXFactory.createDefaultTalon(Constants.kRampMasterId); maybe?

    
    leftMotor.configSelectedFeedbackSensor(
      FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);



    leftMotor.configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
          RobotMap.kTimeoutMs);



    leftMotor.configForwardSoftLimitThreshold(
          RobotMap.rampUpperLimit, RobotMap.kTimeoutMs);



    leftMotor.configForwardSoftLimitEnable(true, RobotMap.kTimeoutMs);



    leftMotor.configVoltageCompSaturation(11.0, RobotMap.kTimeoutMs); // 254 put at 12.0 ¯\_(ツ)_/¯



    leftMotor.configReverseSoftLimitThreshold(
          RobotMap.rampLowerLimit, RobotMap.kTimeoutMs);



    leftMotor.configReverseSoftLimitEnable(
          true, RobotMap.kTimeoutMs);


    //configure magic motion

    leftMotor.config_kP(
          RobotMap.rampPID, RobotMap.Pramp, RobotMap.kTimeoutMs);



    leftMotor.config_kI(
          RobotMap.rampPID, RobotMap.Iramp , RobotMap.kTimeoutMs);



    leftMotor.config_kD(
          RobotMap.rampPID, RobotMap.Dramp, RobotMap.kTimeoutMs);



    //            leftMotor.config_kF(
    //                    RobotMap.rampPID, Constants.kRampHighGearKf, RobotMap.kTimeoutMs);



    //            leftMotor.configMaxIntegralAccumulator(
    //                   RobotMap.rampPID, Constants.kRampHighGearMaxIntegralAccumulator, RobotMap.kTimeoutMs);



    //            leftMotor.config_IntegralZone(
    //                    RobotMap.rampPID, Constants.kRampHighGearIZone, RobotMap.kTimeoutMs);



    leftMotor.configAllowableClosedloopError(
          RobotMap.rampPID, RobotMap.rampAllowableError, RobotMap.kTimeoutMs);


    leftMotor.configMotionAcceleration(
          RobotMap.rampAcceleration, RobotMap.kTimeoutMs);


    leftMotor.configMotionCruiseVelocity(
          RobotMap.rampCruiseVelocity, RobotMap.kTimeoutMs);
    /*
    //configure position PID

    leftMotor.config_kP(
          kPositionControlSlot, Constants.kRampJogKp, RobotMap.kTimeoutMs);


    leftMotor.config_kI(
          kPositionControlSlot, Constants.kRampHighGearKi, RobotMap.kTimeoutMs);


    leftMotor.config_kD(
          kPositionControlSlot, Constants.kRampJogKd, RobotMap.kTimeoutMs);


    leftMotor.configMaxIntegralAccumulator(
          kPositionControlSlot, Constants.kRampHighGearMaxIntegralAccumulator, RobotMap.kTimeoutMs);


    leftMotor.config_IntegralZone(
          kPositionControlSlot, Constants.kRampHighGearIZone, RobotMap.kTimeoutMs);


    leftMotor.configAllowableClosedloopError(
          kPositionControlSlot, Constants.kRampHighGearDeadband, RobotMap.kTimeoutMs);
    */

    /*   TODO: add this?
    leftMotor.configClosedloopRamp(
          Constants.kRampRampRate, RobotMap.kTimeoutMs);


    leftMotor.configOpenloopRamp(
          Constants.kRampRampRate, RobotMap.kTimeoutMs);

    //end multiline comment here 

    //TODO: may need to adjust values?
    leftMotor.configContinuousCurrentLimit(20, RobotMap.kTimeoutMs);


    leftMotor.configPeakCurrentLimit(35, RobotMap.kTimeoutMs);


    leftMotor.configPeakCurrentDuration(200, RobotMap.kTimeoutMs);

    leftMotor.enableCurrentLimit(true);


    leftMotor.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
    leftMotor.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);


    leftMotor.selectProfileSlot(RobotMap.rampPID, 0);

    leftMotor.overrideLimitSwitchesEnable(true);
    leftMotor.overrideSoftLimitsEnable(false);

    leftMotor.enableVoltageCompensation(true);

    leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
    leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);

    
    leftMotor.setSensorPhase(true);

    leftMotor.set(ControlMode.PercentOutput, 0);

    rightMotor.set(ControlMode.Follower, RobotMap.rampTalonPort1); // following leftMotor
    
    //TODO: do talonObject.setInverted(true) with correct motor
  }
*/

  @Override
  public void initDefaultCommand() {
    
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }



  public void setLiftPosition(double setpoint){
    leftMotor.set(ControlMode.MotionMagic, setpoint);
  }
  
  public void resetCounter(){
    counter.reset();
  } 

  //sees if limit switch has been hit
  public boolean isSwitchSet(){
    return counter.get() > 0;
  }
  

	public void openFlaps() {
		solenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void closeFlaps() {
		solenoid.set(DoubleSolenoid.Value.kReverse);
	}

  // ?????
  public void stopFlaps() {
		solenoid.set(DoubleSolenoid.Value.kOff);
	}
	
  public void moveLift(double speed){
    // limit switch on always voltage aka top on limit switch
    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopMotors() {
    leftMotor.set(ControlMode.PercentOutput, 0);
    rightMotor.set(ControlMode.PercentOutput, 0);
  }
  public void configRampEncoders(){
  //  leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  public void zeroRampEncoders(){
  //  leftMotor.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
    rightMotor.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
  }

  //TODO: get encoder values from other side maybe? or average the two sides?
  public int getRampEncoderUnits(){
    return rightMotor.getSelectedSensorPosition();
  }
}
