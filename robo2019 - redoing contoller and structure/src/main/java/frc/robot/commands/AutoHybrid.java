/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoHybrid extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoHybrid() {
    
    addSequential(new DriveToDistance(116.25));// 173.25-24.5 when arm is extended out. fully extend the arm?
    addSequential(new ToggleHatch());
   //TODO: addSequential(new moveArm()); // add value
    //need to do math to find out how far back you need to move, because it does not take into account robot perimeter
    addSequential(new DriveAngleAdjustment(124.2131));
    addSequential(new DriveToDistance(149.7850));
    addSequential(new DriveAngleAdjustment(55.7869));
  //TODO:  addSequential(new moveArm()); // add value
    addSequential(new DriveToDistance(95.28));
    addSequential(new ToggleHatch());
    addSequential(new DriveToDistance(-133.3125));
    addSequential(new DriveAngleAdjustment(-90.0000));
    addSequential(new DriveToDistance(-14.7924));
    addSequential(new DriveAngleAdjustment(-71.5884));
    addSequential(new DriveToDistance(53.9828));
    addSequential(new ToggleHatch());

  }
}
