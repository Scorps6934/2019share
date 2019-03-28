/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class AutoDoubleShuttle extends CommandGroup {

  public AutoDoubleShuttle() {
    //is the elevator going to be in the right height initially?--> add the elevator height change
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
    addSequential(new DriveToDistance(-95.28));
    addSequential(new DriveAngleAdjustment(128.11));
    addSequential(new DriveToDistance(184.86));
    addSequential(new DriveAngleAdjustment(51.89));
    addSequential(new DriveToDistance(10.88));
    // addSequential(new MoveArm(position));
    addSequential(new ToggleHatch());



    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
