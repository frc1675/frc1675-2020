/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;
import frc.robot.Constants;

public class RotationControl extends CommandBase {
  private ColorWheel colorWheel;
  private int colorCounts;

  public RotationControl(ColorWheel colorWheel, int colorCounts) {
    this.colorWheel = colorWheel;
    this.colorCounts = colorCounts;
    addRequirements(colorWheel);
  }

  @Override
  public void initialize() {
    colorWheel.resetColorCount();
  }

  @Override
  public void execute() {
    colorWheel.turnWheel();
  }

  @Override
  public void end(boolean interrupted) {
    colorWheel.stopWheel();
  }

  @Override
  public boolean isFinished() {
    boolean requiredRotations = colorWheel.getColorCount() >= colorCounts ? true : false;
    // boolean requiredRotations = colorCounts == Constants.ROTATION_COUNTS_NEEDED ? true : false;
    SmartDashboard.putBoolean("Has the spinner reached the required amount of rotations?", requiredRotations);
    return requiredRotations;
  }
}