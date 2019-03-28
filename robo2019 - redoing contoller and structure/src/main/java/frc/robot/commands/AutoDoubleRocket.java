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
    
    addSequential(new DriveToDistance(76.8025));
    addSequential(new DriveAngleAdjustment(55.3238));
    addSequential(new DriveToDistance(150.3987));
    addSequential(new DriveAngleAdjustment(36.9164));
    addSequential(new DriveToDistance(11.82));
    addSequential(new ToggleHatch());
    addSequential(new DriveToDistance(-11.82));
    addSequential(new DriveAngleAdjustment(161.5884));
    addSequential(new DriveToDistance(201.13));
    addSequential(new ToggleHatch());
    addSequential(new DriveToDistance(-23.38));
    addSequential(new DriveAngleAdjustment(174.1239));
    addSequential(new DriveToDistance(-268.0283));
    addSequential(new DriveAngleAdjustment(95.8761));
    addSequential(new DriveToDistance(13.72));
    addSequential(new DriveAngleAdjustment(63.8108));
    addSequential(new DriveToDistance(51.2087));
    addSequential(new ToggleHatch());

  }
}
