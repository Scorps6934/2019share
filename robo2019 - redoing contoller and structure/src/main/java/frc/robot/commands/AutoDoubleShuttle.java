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
    addSequential(new DriveToDistance(116.25));// 173.25-24.5 when arm is extended out. fully extend the arm?
    addSequential(new ToggleHatch());
  //TODO:  addSequential(new moveArm()); // add value
    //need to do math to find out how far back you need to move, because it does not take into account robot perimeter
    addSequential(new DriveAngleAdjustment(124.2131));
    addSequential(new DriveToDistance(149.7850));
    addSequential(new DriveAngleAdjustment(55.7869));
  //TODO:  addSequential(new moveArm()); // add value
    addSequential(new DriveToDistance(95.28));
    addSequential(new ToggleHatch());
    addSequential(new DriveToDistance(-95.28));
    addSequential(new DriveAngleAdjustment(118.5776));
    addSequential(new DriveToDistance(165.6114));
    addSequential(new DriveAngleAdjustment(61.4224));
    addSequential(new DriveToDistance(5.0000));
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
