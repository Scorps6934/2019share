/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Add your docs here.
 */
public class ChangeAutoSettings extends InstantCommand { 
  public static int heightIndex = 0; // lowest height
  public static int modeIndex = 1; //hatch mode (might want dashboard changes)

  private AutoSettings option;

  public enum AutoSettings{
    UP,
    DOWN,
    CARGO,
    HATCH
  }

  public ChangeAutoSettings(AutoSettings option) {
    super("ChangeAutoSettings: "+option);
    //hopefully not using requires doesn't throw an error
    this.option = option;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    switch(option){
      case UP:
        if(heightIndex < 2) heightIndex += 1;
        break;
      case DOWN:
        if(heightIndex > 0) heightIndex -= 1;
        break;     
      case CARGO:
        modeIndex = 1;
        break;
      case HATCH:
        modeIndex = 0;
        break;
    }
  
  }
}
