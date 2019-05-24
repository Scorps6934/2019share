/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class ToggleHatch extends InstantCommand {
  private boolean solenoidIsHolding = false; 
  public ToggleHatch() {
    super("ToggleHatch");
    requires(Robot.shatch);
    Robot.shatch.useSuctionValve(); 
    Robot.shatch.setHatchPumpRaw(1);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    solenoidIsHolding = !solenoidIsHolding;
    if (solenoidIsHolding){
      Robot.shatch.useSuctionValve(); 
      Robot.shatch.setHatchPumpRaw(1);
    }  
    else {
      Robot.shatch.useFreeValve();
      Robot.shatch.setHatchPumpRaw(1);
    }
  }

}
