/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.RobotMap;
import frc.robot.commands.MoveWheels;

/**
 * Add your docs here.
 */
public class S_DriveWheels extends Subsystem {
  TalonSRX lfMoto = new TalonSRX(RobotMap.lfDrive); // left front
  TalonSRX lbMoto = new TalonSRX(RobotMap.lbDrive); // left back
  TalonSRX rfMoto = new TalonSRX(RobotMap.rfDrive); // right front
  TalonSRX rbMoto = new TalonSRX(RobotMap.rbDrive); // right back
  public AHRS gyro = new AHRS(I2C.Port.kOnboard); //TODO: check if kOnboard is correct (vs kMPX)

/*
  public S_DriveWheels() {
    super("S_DriveWheels", 1.0, 1.0, 1.0);
    setAbsoluteTolerance(0.05);
    getPIDController().setContinuous(false);
  }
*/
  
  public S_DriveWheels(){
    //    lfMoto = TalonSRXFactory.createDefaultTalon(Constants.kDriveMasterId); maybe?


    /*
    lfMoto.configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
          RobotMap.kTimeoutMs);



    lfMoto.configForwardSoftLimitThreshold(
          RobotMap.driveUpperLimit, RobotMap.kTimeoutMs);



    lfMoto.configForwardSoftLimitEnable(true, RobotMap.kTimeoutMs);

    lfMoto.configReverseSoftLimitThreshold(
          RobotMap.driveLowerLimit, RobotMap.kTimeoutMs);



    lfMoto.configReverseSoftLimitEnable(
          true, RobotMap.kTimeoutMs);
    */


    lfMoto.configVoltageCompSaturation(11.0, RobotMap.kTimeoutMs); // 254 put at 12.0 ¯\_(ツ)_/¯


  //configure magic motion
  // pid slots and constants config
    //angle pid
    lfMoto.configSelectedFeedbackSensor( //TODO: check that analog feedback is correct
      FeedbackDevice.Analog, 0, RobotMap.kTimeoutMs);
      
    lfMoto.config_kP(
          RobotMap.driveAnglePID, RobotMap.PdriveAngle, RobotMap.kTimeoutMs);

    lfMoto.config_kI(
          RobotMap.driveAnglePID, RobotMap.IdriveAngle, RobotMap.kTimeoutMs);

    lfMoto.config_kD(
          RobotMap.driveAnglePID, RobotMap.DdriveAngle, RobotMap.kTimeoutMs);

    lfMoto.configAllowableClosedloopError(
          RobotMap.driveAnglePID, RobotMap.driveAngleAllowableError, RobotMap.kTimeoutMs);
      
    //distance pid
    lfMoto.configSelectedFeedbackSensor(
      FeedbackDevice.QuadEncoder, 0, RobotMap.kTimeoutMs);
      
    lfMoto.config_kP(
          RobotMap.driveDistancePID, RobotMap.PdriveDistance, RobotMap.kTimeoutMs);

    lfMoto.config_kI(
          RobotMap.driveDistancePID, RobotMap.IdriveDistance, RobotMap.kTimeoutMs);

    lfMoto.config_kD(
          RobotMap.driveDistancePID, RobotMap.DdriveDistance, RobotMap.kTimeoutMs);

    lfMoto.configAllowableClosedloopError(
          RobotMap.driveDistancePID, RobotMap.driveDistanceAllowableError, RobotMap.kTimeoutMs);

    //            lfMoto.config_kF(
    //                    RobotMap.drivePID, Constants.kDriveHighGearKf, RobotMap.kTimeoutMs);



    //            lfMoto.configMaxIntegralAccumulator(
    //                   RobotMap.drivePID, Constants.kDriveHighGearMaxIntegralAccumulator, RobotMap.kTimeoutMs);



    //            lfMoto.config_IntegralZone(
    //                    RobotMap.drivePID, Constants.kDriveHighGearIZone, RobotMap.kTimeoutMs);
//TODO: may need to fix by putting with select slot in commands
    lfMoto.configMotionAcceleration(
          RobotMap.driveAcceleration, RobotMap.kTimeoutMs);


    lfMoto.configMotionCruiseVelocity(
          RobotMap.driveCruiseVelocity, RobotMap.kTimeoutMs);
    /*
    //configure position PID

    lfMoto.config_kP(
          kPositionControlSlot, Constants.kDriveJogKp, RobotMap.kTimeoutMs);


    lfMoto.config_kI(
          kPositionControlSlot, Constants.kDriveHighGearKi, RobotMap.kTimeoutMs);


    lfMoto.config_kD(
          kPositionControlSlot, Constants.kDriveJogKd, RobotMap.kTimeoutMs);


    lfMoto.configMaxIntegralAccumulator(
          kPositionControlSlot, Constants.kDriveHighGearMaxIntegralAccumulator, RobotMap.kTimeoutMs);


    lfMoto.config_IntegralZone(
          kPositionControlSlot, Constants.kDriveHighGearIZone, RobotMap.kTimeoutMs);


    lfMoto.configAllowableClosedloopError(
          kPositionControlSlot, Constants.kDriveHighGearDeadband, RobotMap.kTimeoutMs);
    */

    /*   TODO: add this?
    lfMoto.configClosedloopRamp(
          Constants.kDriveRampRate, RobotMap.kTimeoutMs);


    lfMoto.configOpenloopRamp(
          Constants.kDriveRampRate, RobotMap.kTimeoutMs);

    */  

    //TODO: may need to adjust values?
    lfMoto.configContinuousCurrentLimit(20, RobotMap.kTimeoutMs);


    lfMoto.configPeakCurrentLimit(35, RobotMap.kTimeoutMs);


    lfMoto.configPeakCurrentDuration(200, RobotMap.kTimeoutMs);

    lfMoto.enableCurrentLimit(true);


    lfMoto.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
    lfMoto.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);


    lfMoto.overrideLimitSwitchesEnable(true);
    lfMoto.overrideSoftLimitsEnable(false);

    lfMoto.enableVoltageCompensation(true);

    lfMoto.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20); // may not need?
    lfMoto.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);


    lfMoto.setSensorPhase(true);

    lfMoto.set(ControlMode.PercentOutput, 0);
    rfMoto.set(ControlMode.PercentOutput, 0);
    lbMoto.set(ControlMode.PercentOutput, 0);
    rbMoto.set(ControlMode.PercentOutput, 0);

//following lfMoto
    lbMoto.set(ControlMode.Follower, RobotMap.lfDrive);
    rfMoto.set(ControlMode.Follower, RobotMap.lfDrive);
    rbMoto.set(ControlMode.Follower, RobotMap.rfDrive);
    //TODO: do talonObject.setInverted(true) with correct motor (invert lf or rf)
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MoveWheels());
  }

  public void setDriveDistance(double setpoint){
    lfMoto.set(ControlMode.MotionMagic, setpoint);
  }

  public void setDriveAngle(double setpoint){
    lfMoto.set(ControlMode.MotionMagic, setpoint);
    rfMoto.set(ControlMode.MotionMagic, -setpoint);
  }


  public void adjustSpeed(double leftInput, double rightInput){
    lfMoto.set(ControlMode.PercentOutput, leftInput);
    lbMoto.set(ControlMode.PercentOutput, leftInput);
    rfMoto.set(ControlMode.PercentOutput, rightInput);
    rbMoto.set(ControlMode.PercentOutput, rightInput);

  }

  public void stopWheels(){
    lfMoto.set(ControlMode.PercentOutput, 0);
    lbMoto.set(ControlMode.PercentOutput, 0);
    rfMoto.set(ControlMode.PercentOutput, 0);
    rbMoto.set(ControlMode.PercentOutput, 0);
  }

  public void configDriveEncoders(){ // maybe use FeedbackDevice.CTRE_MagEncoder_Relative instead
    lfMoto.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    lbMoto.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rfMoto.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rbMoto.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    //    System.out.println(lfMoto.getSelectedSensorPosition());
  }
  public void zeroDriveEncoders()
  {
    lfMoto.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
    lbMoto.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
    rfMoto.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
    rbMoto.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
  }

  //TODO: get encoder values from other side maybe? and/or average the two sides?
  public int getDriveEncoderUnits(){
    return lfMoto.getSelectedSensorPosition();
  }

  public double getCurrentAngle(){
    return gyro.getYaw() + 180.0; // TODO: may want to change to not be continuous (go over 360 or under 0)
  }
  public int getCurrentError(){
      return lfMoto.getClosedLoopError();
  }

  public void selectDriveDistancePID(){
      lfMoto.selectProfileSlot(RobotMap.driveDistancePID, 0);
  }
  public void selectDriveAnglePID(){
      lfMoto.selectProfileSlot(RobotMap.driveAnglePID, 0);
  }

}
