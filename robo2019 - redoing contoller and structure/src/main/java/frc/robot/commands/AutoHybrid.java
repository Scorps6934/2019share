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
    
    addSequential(new DriveToDistance(173.25));// 173.25-24.5 when arm is extended out. fully extend the arm?
    addSequential(new ToggleHatch());
    //need to do math to find out how far back you need to move, because it does not take into account robot perimeter
    addSequential(new DriveToDistance(-10.88));//backwords distance is negative? 
    addSequential(new DriveAngleAdjustment(132.69));
    addSequential(new DriveToDistance(168.27));
    addSequential(new DriveAngleAdjustment(47.31));
    addSequential(new DriveToDistance(95.28));
    // addSequential(new MoveArm(position)); set proper position
    addSequential(new ToggleHatch());
    addSequential(new DriveToDistance(-177.75));
    addSequential(new DriveAngleAdjustment(161.5884));
    addSequential(new DriveToDistance(11.82));
    // addSequential(new MoveArm(position)); set proper position
    addSequential(new ToggleHatch());

  }
}
