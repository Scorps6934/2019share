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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class S_Hatch extends Subsystem {
	TalonSRX pump = new TalonSRX(RobotMap.hatchPumpTalon);
  DoubleSolenoid solenoid = new DoubleSolenoid(RobotMap.pcmPort, RobotMap.hatchSolenoidForward, RobotMap.hatchSolenoidReverse);

	public S_Hatch(){
		pump.setNeutralMode(NeutralMode.Brake);
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

	public void setHatchPumpRaw(double power){
		pump.set(ControlMode.PercentOutput, power);
	}

	public void useSuctionValve() {
		solenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void useFreeValve() {
		solenoid.set(DoubleSolenoid.Value.kReverse);
	}

  // ????
	public void stopHatch() {
		solenoid.set(DoubleSolenoid.Value.kOff);
	}
	
}
