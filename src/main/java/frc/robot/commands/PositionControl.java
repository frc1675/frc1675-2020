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

public class PositionControl extends CommandBase {
  private ColorWheel colorWheel;

  public PositionControl(ColorWheel colorWheel) {
    this.colorWheel = colorWheel;
    addRequirements(colorWheel);
  }

  @Override
  public void initialize() {
    colorWheel.getTargetColor();
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
    boolean colorMatch = colorWheel.getTargetColor() == colorWheel.getWheelColor() ? true : false;
    SmartDashboard.putBoolean("Target Color Match", colorMatch);
    return colorMatch;
  }
}
