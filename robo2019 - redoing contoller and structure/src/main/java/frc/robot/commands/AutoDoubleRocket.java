/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoDoubleRocket extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoDoubleRocket() {
    
    addSequential(new DriveToDistance(86.3125));
    addSequential(new DriveAngleAdjustment(90.0000));
    addSequential(new DriveToDistance(61.6850));
    addSequential(new DriveAngleAdjustment(-71.5884));
    addSequential(new DriveToDistance(53.9828));
    addSequential(new ToggleHatch());
    addSequential(new DriveToDistance(-53.9828));
    addSequential(new DriveAngleAdjustment(71.5884));
    addSequential(new DriveToDistance(14.7924));
    addSequential(new DriveAngleAdjustment(90.0000));
    addSequential(new DriveToDistance(133.3125));
    addSequential(new ToggleHatch());
    addSequential(new DriveToDistance(-23.38));
    addSequential(new DriveAngleAdjustment(-5.8761));
    addSequential(new DriveToDistance(-268.0283));
    addSequential(new DriveAngleAdjustment(95.8761));
    addSequential(new DriveToDistance(13.72));
    addSequential(new DriveAngleAdjustment(63.8108));
    addSequential(new DriveToDistance(51.2087));
    addSequential(new ToggleHatch());

  }
}
