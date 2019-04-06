/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.Vision;

public class AlignToObjective extends CommandGroup {
  /**
   * Add your docs here.
   */
  private double angleToTurn;
  private Point center;
  private double hypotenuseC;
  private double depthDistance;
  private double horizontalDistance;


  public AlignToObjective(){ //only does low level hatch
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    center= Robot.leftVisionProcessor.findCenter();
    if (center != null){
      angleToTurn=Robot.leftVisionProcessor.calculateAngleToTurn(center);
    }
    else{
      center = Robot.rightVisionProcessor.findCenter();
      if(center != null){
        angleToTurn = Robot.rightVisionProcessor.calculateAngleToTurn(center);
      }
    }

    if(center != null){
      hypotenuseC = Vision.calculateHypotenuseC(center);
      depthDistance = Vision.calculateDepthDistance(angleToTurn, hypotenuseC);
      horizontalDistance = Vision.calculateHorizontalDistance(angleToTurn, hypotenuseC);

      addSequential(new DriveAngleAdjustment(angleToTurn));
      addSequential(new DriveToDistance(horizontalDistance));
      addSequential(new DriveAngleAdjustment(90.0));
      addSequential(new DriveToDistance(depthDistance)); // TODO: Add translation math to account for arm and center
      
      //TODO: auto align for grabbing/placing hatches - maybe

    }

  
  
  
  }

    //addSequential(new DriveAngleAdjustment());




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
