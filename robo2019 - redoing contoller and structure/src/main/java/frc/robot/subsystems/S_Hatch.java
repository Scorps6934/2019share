/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class S_Hatch extends Subsystem {
  DoubleSolenoid solenoid = new DoubleSolenoid(RobotMap.pcmPort, RobotMap.hatchSolenoidForward, RobotMap.hatchSolenoidReverse);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


	public void openHatch() {
		solenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void closeHatch() {
		solenoid.set(DoubleSolenoid.Value.kReverse);
	}

  // ????
	public void stopHatch() {
		solenoid.set(DoubleSolenoid.Value.kOff);
	}
	
}
